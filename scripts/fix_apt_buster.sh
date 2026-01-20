#!/usr/bin/env bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

echo "[apt] fix buster repos"

# Основной репозиторий Raspberry Pi OS (архив)
cat >/etc/apt/sources.list <<'EOF'
deb http://archive.raspberrypi.com/debian/ buster main
EOF

# Репозиторий firmware/ядра от Raspberry Pi (оставляем)
mkdir -p /etc/apt/sources.list.d
cat >/etc/apt/sources.list.d/raspi.list <<'EOF'
deb http://archive.raspberrypi.org/debian buster main
EOF

# Отключаем Valid-Until, потому что buster EOL и метаданные могут быть "просрочены"
mkdir -p /etc/apt/apt.conf.d
cat >/etc/apt/apt.conf.d/99no-check-valid-until <<'EOF'
Acquire::Check-Valid-Until "false";
EOF

# Разрешаем принять изменение Suite без интерактива
apt-get update --allow-releaseinfo-change -o Acquire::Check-Valid-Until=false

echo "[apt] done"
