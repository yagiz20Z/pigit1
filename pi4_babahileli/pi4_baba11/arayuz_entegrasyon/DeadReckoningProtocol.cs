using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace Arayuz.Telemetri;

public readonly record struct GeoTrackPoint(double Latitude, double Longitude, DateTime TimestampUtc);

public readonly record struct NavigationFrame(
    double Latitude,
    double Longitude,
    double GroundSpeedMps,
    double SetpointSpeedMps,
    double RollDeg,
    double PitchDeg,
    double YawDeg,
    double SetpointYawDeg,
    int VehicleMode);

public readonly record struct DeadReckoningFrame(
    double Latitude,
    double Longitude,
    double AbsoluteYawDeg,
    double RelativeYawDeg,
    double ReferenceYawDeg,
    double ForwardSpeedMps,
    double LateralSpeedMps,
    double TotalDistanceM,
    double GpsDifferenceM,
    bool Active);

public enum TelemetryFrameKind
{
    None,
    Navigation,
    DeadReckoning,
    Motor
}

public static class IdaTelemetryProtocol
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    public static TelemetryFrameKind ParseLine(
        string line,
        out NavigationFrame navigation,
        out DeadReckoningFrame deadReckoning)
    {
        navigation = default;
        deadReckoning = default;

        if (!TryGetCheckedPayload(line, out string payload))
            return TelemetryFrameKind.None;

        if (payload.StartsWith("NAV:", StringComparison.Ordinal))
        {
            string[] values = payload[4..].Split(',');
            if (values.Length != 9)
                return TelemetryFrameKind.None;

            if (!TryDouble(values[0], out double lat) ||
                !TryDouble(values[1], out double lon) ||
                !TryDouble(values[2], out double groundSpeed) ||
                !TryDouble(values[3], out double setpointSpeed) ||
                !TryDouble(values[4], out double roll) ||
                !TryDouble(values[5], out double pitch) ||
                !TryDouble(values[6], out double yaw) ||
                !TryDouble(values[7], out double setpointYaw) ||
                !int.TryParse(values[8], NumberStyles.Integer, Inv, out int mode))
            {
                return TelemetryFrameKind.None;
            }

            navigation = new NavigationFrame(
                lat, lon, groundSpeed, setpointSpeed,
                roll, pitch, yaw, setpointYaw, mode);
            return TelemetryFrameKind.Navigation;
        }

        if (payload.StartsWith("DR:", StringComparison.Ordinal))
        {
            string[] values = payload[3..].Split(',');
            if (values.Length != 10)
                return TelemetryFrameKind.None;

            if (!TryDouble(values[0], out double lat) ||
                !TryDouble(values[1], out double lon) ||
                !TryDouble(values[2], out double absoluteYaw) ||
                !TryDouble(values[3], out double relativeYaw) ||
                !TryDouble(values[4], out double referenceYaw) ||
                !TryDouble(values[5], out double forwardSpeed) ||
                !TryDouble(values[6], out double lateralSpeed) ||
                !TryDouble(values[7], out double totalDistance) ||
                !TryDouble(values[8], out double gpsDifference) ||
                !int.TryParse(values[9], NumberStyles.Integer, Inv, out int active))
            {
                return TelemetryFrameKind.None;
            }

            deadReckoning = new DeadReckoningFrame(
                lat, lon, absoluteYaw, relativeYaw, referenceYaw,
                forwardSpeed, lateralSpeed, totalDistance, gpsDifference,
                active != 0);
            return TelemetryFrameKind.DeadReckoning;
        }

        return payload.StartsWith("MOT:", StringComparison.Ordinal)
            ? TelemetryFrameKind.Motor
            : TelemetryFrameKind.None;
    }

    public static string BuildCommand(string payload)
    {
        if (string.IsNullOrWhiteSpace(payload))
            throw new ArgumentException("Komut boş olamaz.", nameof(payload));

        string normalized = payload.Trim();
        return $"{normalized}*{Checksum(normalized)}\n";
    }

    public static string BuildDeadReckoningResetCommand() =>
        BuildCommand("CMD:DR:RESET");

    private static bool TryGetCheckedPayload(string line, out string payload)
    {
        payload = string.Empty;
        if (string.IsNullOrWhiteSpace(line))
            return false;

        string clean = line.Trim();
        int star = clean.LastIndexOf('*');
        if (star <= 0 || star == clean.Length - 1)
            return false;

        string candidate = clean[..star];
        string received = clean[(star + 1)..].Trim();
        if (!Checksum(candidate).Equals(received, StringComparison.OrdinalIgnoreCase))
            return false;

        payload = candidate;
        return true;
    }

    private static string Checksum(string payload)
    {
        int sum = payload.Sum(ch => (int)(byte)ch);
        return (sum & 0xFF).ToString("X2", Inv);
    }

    private static bool TryDouble(string value, out double result) =>
        double.TryParse(value, NumberStyles.Float, Inv, out result) &&
        double.IsFinite(result);
}

public sealed class DualTrackState
{
    private readonly object _sync = new();
    private readonly List<GeoTrackPoint> _gpsTrail = new();
    private readonly List<GeoTrackPoint> _estimatedTrail = new();

    public int MaximumPointCount { get; set; } = 6000;
    public double MinimumPointDistanceM { get; set; } = 0.05;

    public NavigationFrame LastNavigation { get; private set; }
    public DeadReckoningFrame LastDeadReckoning { get; private set; }

    public IReadOnlyList<GeoTrackPoint> GpsTrail
    {
        get { lock (_sync) return _gpsTrail.ToArray(); }
    }

    public IReadOnlyList<GeoTrackPoint> EstimatedTrail
    {
        get { lock (_sync) return _estimatedTrail.ToArray(); }
    }

    public bool ProcessTelemetryLine(string line)
    {
        TelemetryFrameKind kind = IdaTelemetryProtocol.ParseLine(
            line, out NavigationFrame nav, out DeadReckoningFrame dr);

        lock (_sync)
        {
            switch (kind)
            {
                case TelemetryFrameKind.Navigation:
                    LastNavigation = nav;
                    if (CoordinateValid(nav.Latitude, nav.Longitude))
                    {
                        AddPoint(_gpsTrail, nav.Latitude, nav.Longitude);
                        return true;
                    }
                    break;

                case TelemetryFrameKind.DeadReckoning:
                    LastDeadReckoning = dr;
                    if (dr.Active && CoordinateValid(dr.Latitude, dr.Longitude))
                    {
                        AddPoint(_estimatedTrail, dr.Latitude, dr.Longitude);
                        return true;
                    }
                    break;
            }
        }

        return false;
    }

    public void ClearTrails()
    {
        lock (_sync)
        {
            _gpsTrail.Clear();
            _estimatedTrail.Clear();
        }
    }

    private void AddPoint(List<GeoTrackPoint> trail, double latitude, double longitude)
    {
        var point = new GeoTrackPoint(latitude, longitude, DateTime.UtcNow);
        if (trail.Count > 0)
        {
            GeoTrackPoint previous = trail[^1];
            if (DistanceM(previous.Latitude, previous.Longitude, latitude, longitude) <
                MinimumPointDistanceM)
            {
                return;
            }
        }

        trail.Add(point);
        int overflow = trail.Count - Math.Max(100, MaximumPointCount);
        if (overflow > 0)
            trail.RemoveRange(0, overflow);
    }

    private static bool CoordinateValid(double latitude, double longitude) =>
        double.IsFinite(latitude) && double.IsFinite(longitude) &&
        latitude is >= -90.0 and <= 90.0 &&
        longitude is >= -180.0 and <= 180.0 &&
        !(latitude == 0.0 && longitude == 0.0);

    private static double DistanceM(double lat1, double lon1, double lat2, double lon2)
    {
        double meanLatitudeRad = ((lat1 + lat2) * 0.5) * Math.PI / 180.0;
        double north = (lat2 - lat1) * 111_320.0;
        double east = (lon2 - lon1) * 111_320.0 * Math.Cos(meanLatitudeRad);
        return Math.Sqrt(north * north + east * east);
    }
}
