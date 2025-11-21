#!/bin/bash
set -e

echo "=== [1/9] Удаляем старые следы EtherCAT Master ==="
sudo systemctl stop ethercat.service 2>/dev/null || true
sudo systemctl disable ethercat.service 2>/dev/null || true
sudo rm -f /etc/systemd/system/ethercat.service
sudo rm -rf /usr/local/etherlab /usr/local/bin/ethercat /usr/local/lib/ethercat \
             /lib/modules/$(uname -r)/kernel/ethercat /etc/ethercat.conf \
             /usr/src/ethercat* /etc/systemd/system/ethercat-modules.service

sudo modprobe -r ec_master ec_generic 2>/dev/null || true
sudo systemctl daemon-reload

echo "=== [2/9] Устанавливаем зависимости ==="
sudo apt update
sudo apt install -y build-essential git autoconf libtool pkg-config dkms \
                    linux-headers-$(uname -r) rsync

echo "=== [3/9] Клонируем и собираем EtherCAT Master ==="
cd /tmp
sudo rm -rf ethercat
git clone https://gitlab.com/etherlab.org/ethercat.git
cd ethercat
git checkout stable-1.6

./bootstrap
./configure --enable-generic --prefix=/usr/local --sysconfdir=/etc
make all modules
sudo make install
sudo ldconfig

echo "=== [4/9] Настраиваем DKMS для автоматической пересборки ==="
sudo mkdir -p /usr/src/ethercat-stable-1.6
sudo rsync -a . /usr/src/ethercat-stable-1.6/

cat <<EOF | sudo tee /usr/src/ethercat-stable-1.6/dkms.conf
PACKAGE_NAME="ethercat"
PACKAGE_VERSION="stable-1.6"
CLEAN="make clean"
MAKE="make modules"
BUILT_MODULE_NAME[0]="ec_master"
BUILT_MODULE_LOCATION[0]="master/"
BUILT_MODULE_NAME[1]="ec_generic"
BUILT_MODULE_LOCATION[1]="devices/"
DEST_MODULE_LOCATION[0]="/kernel/ethercat"
DEST_MODULE_LOCATION[1]="/kernel/ethercat"
AUTOINSTALL="yes"
EOF

sudo dkms add -m ethercat -v stable-1.6 || true
sudo dkms build -m ethercat -v stable-1.6
sudo dkms install -m ethercat -v stable-1.6

echo "=== [5/9] Создаем конфиг EtherCAT ==="
IFACE=$(ip -o link show | awk -F': ' '/enp/{print $2; exit}')
MAC=$(cat /sys/class/net/$IFACE/address)
echo "Используется интерфейс: $IFACE ($MAC)"
cat <<EOF | sudo tee /etc/ethercat.conf
MASTER0_DEVICE="$MAC"
DEVICE_MODULES="generic"
EOF

echo "=== [6/9] Создаем systemd unit для модулей ==="
cat <<EOF | sudo tee /etc/systemd/system/ethercat-modules.service
[Unit]
Description=Load EtherCAT kernel modules
After=network.target

[Service]
Type=oneshot
ExecStart=/sbin/modprobe ec_generic
ExecStart=/sbin/modprobe ec_master
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable ethercat-modules.service
sudo systemctl start ethercat-modules.service

echo "=== [7/9] Поднимаем интерфейс и проверяем состояние ==="
sudo ip link set $IFACE up || true
sleep 2
ip link show $IFACE

echo "=== [8/9] Проверка устройства EtherCAT ==="
if [ -e /dev/EtherCAT0 ]; then
  echo "✅ /dev/EtherCAT0 найден — мастер работает!"
else
  echo "⚠️  /dev/EtherCAT0 не найден. Проверь подключение кабеля и интерфейс: $IFACE"
fi

echo "=== [9/9] Проверка команды ethercat ==="
sudo ethercat version || echo "⚠️ Утилита ethercat не найдена, проверь /usr/local/bin"

echo "=== Установка завершена! ==="
echo "Попробуй команду: sudo ethercat master"

