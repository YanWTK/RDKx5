#!/usr/bin/env bash
set -eo pipefail

# Re-enumerate the XVF3800 after the rest of the USB tree has settled.
# This mimics physically plugging the microphone in after boot, before ROS
# opens the ALSA capture device.

XVF3800_USB_VID="${XVF3800_USB_VID:-2886}"
XVF3800_USB_PID="${XVF3800_USB_PID:-001e}"
XVF3800_SYSFS_ROOT="${XVF3800_SYSFS_ROOT:-/sys/bus/usb/devices}"
XVF3800_STARTUP_SETTLE_SEC="${XVF3800_STARTUP_SETTLE_SEC:-2.0}"
XVF3800_DISCONNECT_SEC="${XVF3800_DISCONNECT_SEC:-2.0}"
XVF3800_WAIT_SEC="${XVF3800_WAIT_SEC:-15.0}"
XVF3800_POST_ENUM_SETTLE_SEC="${XVF3800_POST_ENUM_SETTLE_SEC:-3.0}"
XVF3800_ALSA_NAME="${XVF3800_ALSA_NAME:-C16K6Ch}"
XVF3800_ALSA_DEVICE="${XVF3800_ALSA_DEVICE:-hw:C16K6Ch,0}"
XVF3800_HEALTH_CHECK="${XVF3800_HEALTH_CHECK:-true}"
XVF3800_HEALTH_DURATION_SEC="${XVF3800_HEALTH_DURATION_SEC:-1}"
XVF3800_MAX_ZERO_FRACTION="${XVF3800_MAX_ZERO_FRACTION:-0.80}"
XVF3800_HEALTH_RETRIES="${XVF3800_HEALTH_RETRIES:-3}"
export XVF3800_WAIT_SEC XVF3800_MAX_ZERO_FRACTION

find_xvf_device() {
  local entry vid pid
  for entry in "$XVF3800_SYSFS_ROOT"/*; do
    [ -d "$entry" ] || continue
    case "$(basename "$entry")" in
      *:*) continue ;;
    esac
    [ -r "$entry/idVendor" ] && [ -r "$entry/idProduct" ] || continue
    vid="$(tr '[:upper:]' '[:lower:]' < "$entry/idVendor" 2>/dev/null || true)"
    pid="$(tr '[:upper:]' '[:lower:]' < "$entry/idProduct" 2>/dev/null || true)"
    if [ "$vid" = "$XVF3800_USB_VID" ] && [ "$pid" = "$XVF3800_USB_PID" ]; then
      printf '%s\n' "$entry"
      return 0
    fi
  done
  return 1
}

wait_for_xvf_device() {
  local deadline device
  deadline="$(python3 - <<'PY'
import os, time
print(time.monotonic() + float(os.environ.get("XVF3800_WAIT_SEC", "15.0")))
PY
)"
  while python3 - "$deadline" <<'PY'
import sys, time
raise SystemExit(0 if time.monotonic() < float(sys.argv[1]) else 1)
PY
  do
    device="$(find_xvf_device || true)"
    if [ -n "$device" ]; then
      printf '%s\n' "$device"
      return 0
    fi
    sleep 0.2
  done
  return 1
}

wait_for_alsa_card() {
  local deadline
  deadline="$(python3 - <<'PY'
import os, time
print(time.monotonic() + float(os.environ.get("XVF3800_WAIT_SEC", "15.0")))
PY
)"
  while python3 - "$deadline" <<'PY'
import sys, time
raise SystemExit(0 if time.monotonic() < float(sys.argv[1]) else 1)
PY
  do
    if arecord -l 2>/dev/null | grep -q "$XVF3800_ALSA_NAME"; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

find_alsa_card_index() {
  awk -v name="$XVF3800_ALSA_NAME" '
    $0 ~ "\\[" name "[[:space:]]*\\]" { print $1; exit }
  ' /proc/asound/cards 2>/dev/null || true
}

wait_for_pcm_capture_closed() {
  local card status_file deadline state owner
  card="$(find_alsa_card_index)"
  [ -n "$card" ] || return 0
  status_file="/proc/asound/card${card}/pcm0c/sub0/status"
  [ -e "$status_file" ] || return 0
  deadline="$(python3 - <<'PY'
import os, time
print(time.monotonic() + float(os.environ.get("XVF3800_WAIT_SEC", "15.0")))
PY
)"
  while python3 - "$deadline" <<'PY'
import sys, time
raise SystemExit(0 if time.monotonic() < float(sys.argv[1]) else 1)
PY
  do
    state="$(cat "$status_file" 2>/dev/null || true)"
    if echo "$state" | grep -q '^closed$'; then
      return 0
    fi
    owner="$(echo "$state" | awk '/owner_pid/ { print $3; exit }')"
    if [ -n "$owner" ] && ! kill -0 "$owner" 2>/dev/null; then
      sleep 0.2
      continue
    fi
    sleep 0.2
  done
  echo "WARN: XVF3800 capture endpoint is still busy after waiting:" >&2
  cat "$status_file" >&2 || true
  return 1
}

pcm_health_check_once() {
  local wav result status timeout_sec
  wav="$(mktemp /tmp/xvf3800_health_XXXXXX.wav)"
  timeout_sec="$(awk -v duration="$XVF3800_HEALTH_DURATION_SEC" 'BEGIN { printf "%.1f", duration + 3.0 }')"
  if ! timeout "$timeout_sec" \
    arecord -q -D "$XVF3800_ALSA_DEVICE" -f S16_LE -c 6 -r 16000 \
      -d "$XVF3800_HEALTH_DURATION_SEC" "$wav"; then
    rm -f "$wav"
    return 2
  fi

  set +e
  result="$(python3 - "$wav" <<'PY'
import struct
import sys
import wave

path = sys.argv[1]
with wave.open(path, "rb") as wav:
    data = wav.readframes(wav.getnframes())

if not data:
    print("zero_fraction=1.0000 max_abs=0 mean_abs=0.000")
    raise SystemExit(1)

count = len(data) // 2
nonzero = 0
max_abs = 0
abs_sum = 0
for (sample,) in struct.iter_unpack("<h", data[: count * 2]):
    if sample:
        nonzero += 1
    value = abs(sample)
    if value > max_abs:
        max_abs = value
    abs_sum += value

zero_fraction = 1.0 - (nonzero / max(count, 1))
mean_abs = abs_sum / max(count, 1)
print(f"zero_fraction={zero_fraction:.4f} max_abs={max_abs} mean_abs={mean_abs:.3f}")
raise SystemExit(0 if zero_fraction <= float(__import__("os").environ["XVF3800_MAX_ZERO_FRACTION"]) else 1)
PY
  )"
  status=$?
  set -e
  rm -f "$wav"
  echo "XVF3800 PCM health: $result"
  return "$status"
}

echo "XVF3800 startup re-enumerate: settle=${XVF3800_STARTUP_SETTLE_SEC}s disconnect=${XVF3800_DISCONNECT_SEC}s"
sleep "$XVF3800_STARTUP_SETTLE_SEC"

device="$(wait_for_xvf_device || true)"
if [ -z "$device" ]; then
  echo "WARN: XVF3800 USB device ${XVF3800_USB_VID}:${XVF3800_USB_PID} not found; skip re-enumerate." >&2
  exit 0
fi

if [ ! -w "$device/authorized" ]; then
  echo "WARN: no permission to write $device/authorized; skip re-enumerate." >&2
else
  echo "XVF3800 re-enumerate device: $device"
  echo 0 > "$device/authorized"
  sleep "$XVF3800_DISCONNECT_SEC"
  echo 1 > "$device/authorized"
fi

device="$(wait_for_xvf_device || true)"
if [ -z "$device" ]; then
  echo "ERROR: XVF3800 did not reappear in sysfs after re-enumerate." >&2
  exit 1
fi
if [ -w "$device/power/control" ]; then
  echo on > "$device/power/control" 2>/dev/null || true
fi

if ! wait_for_alsa_card; then
  echo "ERROR: XVF3800 ALSA capture card did not appear: $XVF3800_ALSA_NAME" >&2
  exit 1
fi
echo "XVF3800 ALSA card ready: $XVF3800_ALSA_NAME"
sleep "$XVF3800_POST_ENUM_SETTLE_SEC"
wait_for_pcm_capture_closed || true

if [ "$XVF3800_HEALTH_CHECK" = "true" ]; then
  for _attempt in $(seq 1 "$XVF3800_HEALTH_RETRIES"); do
    if pcm_health_check_once; then
      exit 0
    fi
    echo "WARN: XVF3800 PCM health check failed (attempt ${_attempt}/${XVF3800_HEALTH_RETRIES})." >&2
    wait_for_pcm_capture_closed || true
    sleep 2.0
  done
  echo "ERROR: XVF3800 PCM health check did not pass after startup re-enumerate." >&2
  exit 1
fi
