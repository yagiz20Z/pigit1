using System;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;

namespace Arayuz.Telemetri;

/// <summary>
/// DR sıfırlandığı andaki yönü üst/0° kabul eder ve aracın bu merkeze göre
/// ne tarafa döndüğünü gösterir. Pozitif açı sağa, negatif açı sola dönüşür.
/// </summary>
public sealed class RelativeHeadingGauge : Control
{
    public static readonly StyledProperty<double> RelativeYawProperty =
        AvaloniaProperty.Register<RelativeHeadingGauge, double>(nameof(RelativeYaw));

    public static readonly StyledProperty<double> AbsoluteYawProperty =
        AvaloniaProperty.Register<RelativeHeadingGauge, double>(nameof(AbsoluteYaw));

    static RelativeHeadingGauge()
    {
        AffectsRender<RelativeHeadingGauge>(RelativeYawProperty, AbsoluteYawProperty);
    }

    public double RelativeYaw
    {
        get => GetValue(RelativeYawProperty);
        set => SetValue(RelativeYawProperty, value);
    }

    public double AbsoluteYaw
    {
        get => GetValue(AbsoluteYawProperty);
        set => SetValue(AbsoluteYawProperty, value);
    }

    public override void Render(DrawingContext context)
    {
        base.Render(context);

        double size = Math.Min(Bounds.Width, Bounds.Height);
        if (size < 24.0)
            return;

        Point center = Bounds.Center;
        double radius = size * 0.42;
        var rimPen = new Pen(Brushes.Gray, Math.Max(1.0, size * 0.012));
        var tickPen = new Pen(Brushes.DimGray, Math.Max(1.0, size * 0.008));
        var referencePen = new Pen(Brushes.DodgerBlue, Math.Max(2.0, size * 0.018));
        var headingPen = new Pen(Brushes.OrangeRed, Math.Max(2.5, size * 0.024));

        context.DrawEllipse(null, rimPen, center, radius, radius);

        // Dört ana yön çizgisi. Üst çizgi, sıfırlama anındaki merkez yönüdür.
        for (int deg = 0; deg < 360; deg += 45)
        {
            double rad = deg * Math.PI / 180.0;
            double inner = radius * (deg % 90 == 0 ? 0.82 : 0.88);
            Point p1 = new(
                center.X + Math.Sin(rad) * inner,
                center.Y - Math.Cos(rad) * inner);
            Point p2 = new(
                center.X + Math.Sin(rad) * radius,
                center.Y - Math.Cos(rad) * radius);
            context.DrawLine(tickPen, p1, p2);
        }

        // Başlangıç/referans yönü: daima göstergenin üstü.
        context.DrawLine(
            referencePen,
            center,
            new Point(center.X, center.Y - radius * 0.82));

        double relative = NormalizeSigned(RelativeYaw);
        double relativeRad = relative * Math.PI / 180.0;
        Point tip = new(
            center.X + Math.Sin(relativeRad) * radius * 0.76,
            center.Y - Math.Cos(relativeRad) * radius * 0.76);
        context.DrawLine(headingPen, center, tip);
        context.DrawEllipse(Brushes.OrangeRed, null, center, size * 0.035, size * 0.035);
    }

    private static double NormalizeSigned(double degree)
    {
        if (!double.IsFinite(degree))
            return 0.0;

        degree %= 360.0;
        if (degree > 180.0)
            degree -= 360.0;
        if (degree < -180.0)
            degree += 360.0;
        return degree;
    }
}
