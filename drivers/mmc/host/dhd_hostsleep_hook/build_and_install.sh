#!/usr/bin/env bash
# build_and_install.sh [device-ip]
#
# Cross-compiles dhd_hostsleep_hook.ko against this kernel tree, then --
# if device-ip is given (or $DEVICE_IP is set) -- installs it on the real
# device so it auto-loads on every boot:
#   1. copies the .ko next to dhd.ko (native /drivers/ntx508/wifi/)
#   2. backs up /etc/init.d/rcS, then inserts an insmod for it right after
#      each existing "insmod .../dhd.ko" line (idempotent -- safe to re-run)
#
# Without a device-ip, only builds and leaves the .ko in this directory for
# manual inspection/testing.
#
# See dhd_hostsleep_hook.c's own header comment for what this module does
# and why it has to be a separate module attached at runtime rather than a
# dhd.ko rebuild. See ../../../../mds/wifi-hostsleep/ (KoboWM project root)
# for the full investigation history.
#
# Needs: the KoboWM project's cross-compiler toolchain (default: assumes
# this kernel tree is checked out at <KoboWM-root>/kobo-kernel-2.6.35.3-marek/,
# override via TOOLCHAIN_DIR) and, for the --install step, marek's password
# in <KoboWM-root>/bin/device-askpass.sh (override via ASKPASS).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KERNEL_DIR="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
KOBOWM_DIR="$(cd "$KERNEL_DIR/.." && pwd)"

TOOLCHAIN_DIR="${TOOLCHAIN_DIR:-$KOBOWM_DIR/bin/toolchain2/vero2-toolchain/vero2-tc~28012016/amlogic-kernel-gcc}"
ASKPASS="${ASKPASS:-$KOBOWM_DIR/bin/device-askpass.sh}"
DEVICE_IP="${1:-${DEVICE_IP:-}}"

CROSS="$TOOLCHAIN_DIR/bin/arm-none-linux-gnueabi-"

echo "=== Building dhd_hostsleep_hook.ko ==="
make -C "$KERNEL_DIR" M="$SCRIPT_DIR" \
  ARCH=arm CROSS_COMPILE="$CROSS" CC="${CROSS}gcc -B$TOOLCHAIN_DIR/wrap/" \
  modules

KO="$SCRIPT_DIR/dhd_hostsleep_hook.ko"
if [ ! -f "$KO" ]; then
  echo "Build did not produce $KO" >&2
  exit 1
fi
echo "Built: $KO"

if [ -z "$DEVICE_IP" ]; then
  echo "=== No device IP given -- build only, not installing. ==="
  echo "Re-run with a device IP (or set \$DEVICE_IP) to install on real hardware."
  exit 0
fi

if [ ! -x "$ASKPASS" ]; then
  echo "Missing $ASKPASS -- copy bin/device-askpass.sh.example there and fill in marek's password." >&2
  exit 1
fi

ssh_cmd() {
  setsid -w env DISPLAY=:0 SSH_ASKPASS="$ASKPASS" SSH_ASKPASS_REQUIRE=force \
    ssh -F /dev/null -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 \
    -o HostKeyAlgorithms=+ssh-rsa -o PubkeyAcceptedAlgorithms=+ssh-rsa \
    "marek@$DEVICE_IP" "$@" < /dev/null
}

NATIVE_KO_PATH=/drivers/ntx508/wifi/dhd_hostsleep_hook.ko
RCS=/host/etc/init.d/rcS

echo "=== Transferring $KO to device ==="
# No sftp-server on the device, so plain scp/modern-SFTP transfers fail --
# pipe the file over a plain ssh command instead (same trick deploy_kernel.sh
# uses for the uImage).
setsid -w env DISPLAY=:0 SSH_ASKPASS="$ASKPASS" SSH_ASKPASS_REQUIRE=force \
  ssh -F /dev/null -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 \
  -o HostKeyAlgorithms=+ssh-rsa -o PubkeyAcceptedAlgorithms=+ssh-rsa \
  "marek@$DEVICE_IP" "sudo sh -c 'cat > $NATIVE_KO_PATH && chown root:root $NATIVE_KO_PATH && chmod 644 $NATIVE_KO_PATH'" \
  < "$KO"

LOCAL_MD5=$(md5sum "$KO" | cut -d' ' -f1)
REMOTE_MD5=$(ssh_cmd "md5sum $NATIVE_KO_PATH" | cut -d' ' -f1)
if [ "$LOCAL_MD5" != "$REMOTE_MD5" ]; then
  echo "Transfer checksum mismatch! local=$LOCAL_MD5 remote=$REMOTE_MD5" >&2
  exit 1
fi
echo "Transfer verified: $LOCAL_MD5"

echo "=== Wiring auto-load into $RCS (native side) ==="
ALREADY_WIRED=$(ssh_cmd "grep -c 'insmod $NATIVE_KO_PATH' $RCS || true")
if [ "$ALREADY_WIRED" -gt 0 ]; then
  echo "Already wired into rcS ($ALREADY_WIRED insmod line(s)) -- nothing to do."
else
  ssh_cmd "sudo cp $RCS ${RCS}.pre-hostsleep-hook-autoload-backup"
  # Plain-indented insert (no attempt to match the tab-indentation of the
  # surrounding insmod lines -- cosmetic only, sh doesn't care).
  ssh_cmd "sudo sed -i '/insmod \\/drivers\\/ntx508\\/wifi\\/dhd\\.ko/a insmod $NATIVE_KO_PATH' $RCS"
  ssh_cmd "sudo sh -n $RCS"
  ADDED=$(ssh_cmd "grep -c 'insmod $NATIVE_KO_PATH' $RCS || true")
  echo "Inserted $ADDED insmod line(s) into rcS (syntax verified), backup at ${RCS}.pre-hostsleep-hook-autoload-backup"
fi

echo "=== Done. Reboot the device to pick it up: ssh marek@$DEVICE_IP sudo reboot ==="
