#!/usr/bin/env bash
set -euo pipefail
# 无 sudo 时，将 Ubuntu 官方驱动包解压到本用户专用目录。
oled_root="$HOME/.local/share/ustrov_oled"
mkdir -p "$oled_root/downloads" "$oled_root/deps"
cd "$oled_root/downloads"
apt-get download python3-luma.oled python3-luma.core python3-smbus2 \
  python3-cbor2 python3-spidev python3-ftdi python3-usb python3-serial
for oled_deb in ./*.deb; do
  dpkg-deb --extract "$oled_deb" "$oled_root/deps"
done
PYTHONPATH="$oled_root/deps/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -c 'from luma.core.interface.serial import i2c; from luma.oled.device import ssd1306; print("OLED driver imports OK")'
