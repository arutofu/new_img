#!/usr/bin/env bash
set -euo pipefail

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  ca-certificates curl wget xz-utils unzip jq \
  parted kpartx util-linux e2fsprogs dosfstools \
  qemu-user-static binfmt-support \
  rsync dos2unix

# Регистрируем qemu-arm в binfmt_misc вручную
if [[ ! -e /proc/sys/fs/binfmt_misc/qemu-arm ]]; then
  sudo tee /proc/sys/fs/binfmt_misc/register >/dev/null <<'EOF'
:qemu-arm:M::\x7fELF\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x28\x00:\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff\xff:/usr/bin/qemu-arm-static:CF
EOF
fi

dos2unix config/build.env
dos2unix Makefile scripts/*.sh

echo "OK: deps installed"
echo "Note: Docker не нужен. Всё делаем нативно в WSL через loop устройства."
