#!/usr/bin/env bash
set -Eeuo pipefail

SERVICE_NAME="ida-karar-verme.service"
INSTALL_DIR="/opt/ida/pi4_baba"
BINARY_PATH="/usr/local/bin/ida-karar-verme"
ENV_DIR="/etc/ida"
ENV_FILE="${ENV_DIR}/ida.env"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}"

hata() {
    echo "HATA: $*" >&2
    exit 1
}

if [[ ${EUID} -ne 0 ]]; then
    command -v sudo >/dev/null 2>&1 || hata "Bu kurulum root yetkisi ister. sudo bulunamadı."
    exec sudo -E bash "$0" "$@"
fi

SOURCE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUN_USER="${IDA_USER:-${SUDO_USER:-}}"

if [[ -z "${RUN_USER}" || "${RUN_USER}" == "root" ]]; then
    RUN_USER="$(logname 2>/dev/null || true)"
fi
if [[ -z "${RUN_USER}" || "${RUN_USER}" == "root" ]]; then
    hata "Çalıştırılacak kullanıcı belirlenemedi. Örnek: sudo IDA_USER=alinux ./install_autostart.sh"
fi

getent passwd "${RUN_USER}" >/dev/null || hata "Kullanıcı bulunamadı: ${RUN_USER}"
RUN_GROUP="$(id -gn "${RUN_USER}")"
RUN_HOME="$(getent passwd "${RUN_USER}" | cut -d: -f6)"
[[ -d "${RUN_HOME}" ]] || hata "Kullanıcı ev dizini bulunamadı: ${RUN_HOME}"

command -v systemctl >/dev/null 2>&1 || hata "systemd/systemctl bulunamadı."
command -v tar >/dev/null 2>&1 || hata "tar bulunamadı."

CARGO_BIN="$(runuser -u "${RUN_USER}" -- bash -lc 'command -v cargo' 2>/dev/null | tail -n 1 || true)"
if [[ -z "${CARGO_BIN}" ]]; then
    hata "${RUN_USER} kullanıcısı için cargo bulunamadı. Önce Rust/Cargo kurun."
fi

if systemctl is-active --quiet "${SERVICE_NAME}"; then
    echo "Mevcut servis güvenli biçimde durduruluyor..."
    systemctl stop "${SERVICE_NAME}"
fi

SUPPLEMENTARY_GROUPS_LINE=""
if getent group dialout >/dev/null 2>&1; then
    usermod -aG dialout "${RUN_USER}"
    SUPPLEMENTARY_GROUPS_LINE="SupplementaryGroups=dialout"
else
    echo "UYARI: dialout grubu bulunamadı; seri port izinlerini ayrıca kontrol edin."
fi

# Betik daha önce /opt altına kurulmuş kopyadan tekrar çağrılırsa kaynak dizin,
# silinecek hedefle aynı olabilir. Önce geçici bir kaynak kopyası oluştur.
COPY_SOURCE="${SOURCE_DIR}"
STAGE_DIR=""
if [[ "$(readlink -f "${SOURCE_DIR}")" == "$(readlink -m "${INSTALL_DIR}")" ]]; then
    STAGE_DIR="$(mktemp -d /tmp/ida-autostart-source.XXXXXX)"
    trap '[[ -n "${STAGE_DIR:-}" ]] && rm -rf "${STAGE_DIR}"' EXIT
    tar -C "${SOURCE_DIR}" --exclude='./target' --exclude='./.git' -cf - . | \
        tar -C "${STAGE_DIR}" -xf -
    COPY_SOURCE="${STAGE_DIR}"
fi

rm -rf "${INSTALL_DIR}"
install -d -m 0755 -o "${RUN_USER}" -g "${RUN_GROUP}" "${INSTALL_DIR}"

# target ve .git taşınmaz; Pi üzerinde temiz release derlemesi yapılır.
tar -C "${COPY_SOURCE}" \
    --exclude='./target' \
    --exclude='./.git' \
    --exclude='./pi4_baba11_OTONOM_FIX_AUTOSTART.zip' \
    -cf - . | tar -C "${INSTALL_DIR}" -xf -
chown -R "${RUN_USER}:${RUN_GROUP}" "${INSTALL_DIR}"

echo "Release sürümü derleniyor..."
runuser -u "${RUN_USER}" -- env \
    HOME="${RUN_HOME}" \
    CARGO_HOME="${RUN_HOME}/.cargo" \
    "${CARGO_BIN}" build \
    --release \
    --locked \
    --manifest-path "${INSTALL_DIR}/Cargo.toml"

[[ -x "${INSTALL_DIR}/target/release/pi4_baba" ]] || \
    hata "Derleme tamamlandı fakat pi4_baba ikili dosyası bulunamadı."
install -m 0755 "${INSTALL_DIR}/target/release/pi4_baba" "${BINARY_PATH}"

install -d -m 0755 "${ENV_DIR}"
if [[ ! -f "${ENV_FILE}" ]]; then
    sed "s#__IDA_HOME__#${RUN_HOME}#g" \
        "${INSTALL_DIR}/systemd/ida.env.example" > "${ENV_FILE}"
    chmod 0644 "${ENV_FILE}"
    echo "Ortam ayar dosyası oluşturuldu: ${ENV_FILE}"
else
    echo "Mevcut ortam ayar dosyası korundu: ${ENV_FILE}"
fi

sed \
    -e "s#__IDA_USER__#${RUN_USER}#g" \
    -e "s#__IDA_GROUP__#${RUN_GROUP}#g" \
    -e "s#__IDA_HOME__#${RUN_HOME}#g" \
    -e "s#__IDA_SUPPLEMENTARY_GROUPS__#${SUPPLEMENTARY_GROUPS_LINE}#g" \
    -e "s#__IDA_WORKDIR__#${INSTALL_DIR}#g" \
    -e "s#__IDA_ENVFILE__#${ENV_FILE}#g" \
    -e "s#__IDA_BINARY__#${BINARY_PATH}#g" \
    "${INSTALL_DIR}/systemd/ida-karar-verme.service.template" > "${SERVICE_FILE}"
chmod 0644 "${SERVICE_FILE}"

systemctl daemon-reload
systemctl enable --now "${SERVICE_NAME}"

sleep 1

echo
echo "Kurulum tamamlandı."
echo "Kullanıcı     : ${RUN_USER}"
echo "Program       : ${BINARY_PATH}"
echo "Ayar dosyası : ${ENV_FILE}"
echo "Servis        : ${SERVICE_NAME}"
echo
echo "Durum:"
systemctl --no-pager --full status "${SERVICE_NAME}" || true

echo
echo "Canlı log: journalctl -u ${SERVICE_NAME} -f"
echo "Port ayarı: sudo nano ${ENV_FILE}"
echo "Ayar sonrası: sudo systemctl restart ${SERVICE_NAME}"
