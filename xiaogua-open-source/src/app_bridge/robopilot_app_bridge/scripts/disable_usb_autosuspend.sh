#!/usr/bin/env bash
set -u

# Keep USB devices awake. Some hubs/serial chips reset during Linux autosuspend,
# which can leave the motor controller running the last command until the
# serial link comes back.
SYS_USB_DIR="${SYS_USB_DIR:-/sys/bus/usb/devices}"

if [ ! -d "$SYS_USB_DIR" ]; then
  echo "WARN: USB sysfs directory not found: $SYS_USB_DIR"
  exit 0
fi

changed=0
failed=0

if [ -f /sys/module/usbcore/parameters/autosuspend ]; then
  echo -1 > /sys/module/usbcore/parameters/autosuspend 2>/dev/null || true
fi

for dev in "$SYS_USB_DIR"/*; do
  [ -d "$dev" ] || continue
  [ -f "$dev/power/control" ] || continue

  dev_name="$(basename "$dev")"
  if echo "$dev_name" | grep -q ':'; then
    continue
  fi

  if echo on > "$dev/power/control" 2>/dev/null; then
    changed=$((changed + 1))
  else
    failed=$((failed + 1))
  fi

  if [ -f "$dev/power/autosuspend_delay_ms" ]; then
    echo -1 > "$dev/power/autosuspend_delay_ms" 2>/dev/null || true
  fi
done

echo "USB autosuspend disabled for $changed device(s); failed=$failed."
exit 0
