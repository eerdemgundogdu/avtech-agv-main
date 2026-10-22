#!/bin/bash
# AVTECH AGV - Jetson Orin Nano kurulum betigi (JetPack 6, Ubuntu 22.04)
# Kullanim (Jetson uzerinde, jetson_software klasorunun icinden):
#   chmod +x install_jetson.sh && ./install_jetson.sh
# Yapar:
#   1) Python bagimliliklari (manuel kontrol: pyserial + Flask-SocketIO + websocket)
#   2) Kullaniciyi dialout grubuna ekler (seri port erisimi)
#   3) ModemManager'in Teensy / USB-TTL portlarini kurcalamasini engeller (udev)
#   4) avtech-brain.service'i kurar ve acilista baslatir
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_USER="${SUDO_USER:-$USER}"
echo "== AVTECH kurulum: kullanici=$RUN_USER dizin=$DIR"

echo "== 1) Paketler"
sudo apt-get update -y
sudo apt-get install -y python3-pip
# simple-websocket: GUI gercek WebSocket kullansin (yoksa yavas long-polling'e duser)
python3 -m pip install --user "pyserial>=3.5" "flask>=2.3" "flask-socketio>=5.3" "simple-websocket>=1.0"

echo "== 2) dialout grubu"
sudo usermod -aG dialout "$RUN_USER"

echo "== 3) udev: ModemManager seri portlari yoklamasin"
sudo tee /etc/udev/rules.d/99-avtech-serial.rules >/dev/null <<'EOF'
# Teensy (PJRC)
ATTRS{idVendor}=="16c0", ENV{ID_MM_DEVICE_IGNORE}="1", MODE="0666"
# Yaygin USB-TTL donusturuculer (kumanda alicisi): CH340, CP210x, FTDI, PL2303
ATTRS{idVendor}=="1a86", ENV{ID_MM_DEVICE_IGNORE}="1", MODE="0666"
ATTRS{idVendor}=="10c4", ENV{ID_MM_DEVICE_IGNORE}="1", MODE="0666"
ATTRS{idVendor}=="0403", ENV{ID_MM_DEVICE_IGNORE}="1", MODE="0666"
ATTRS{idVendor}=="067b", ENV{ID_MM_DEVICE_IGNORE}="1", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "== 4) systemd servisi"
sed -e "s#__USER__#${RUN_USER}#g" -e "s#__DIR__#${DIR}#g" "$DIR/avtech-brain.service" \
    | sudo tee /etc/systemd/system/avtech-brain.service >/dev/null
if systemctl list-unit-files | grep -q '^avtech-rc.service'; then
    sudo systemctl disable --now avtech-rc.service || true
fi
sudo systemctl daemon-reload
sudo systemctl enable --now avtech-brain.service

IP="$(hostname -I | awk '{print $1}')"
echo
echo "Kurulum tamam."
echo "  GUI:  http://${IP:-<jetson-ip>}:5000"
echo "  Log:  journalctl -u avtech-brain -f"
echo "  Not:  dialout grubu icin bir kez cikis/giris yapin (servis zaten grupla calisir)."
