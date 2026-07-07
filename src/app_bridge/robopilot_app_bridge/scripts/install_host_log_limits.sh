#!/usr/bin/env bash
set -euo pipefail

if [ "${EUID}" -ne 0 ]; then
  echo "ERROR: run this script as root" >&2
  exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="$(cd -- "$SCRIPT_DIR/../config" && pwd)"

install -d -m 0755 /etc/rsyslog.d /etc/systemd/journald.conf.d
changed=false

install_if_changed() {
  local source_path="$1"
  local target_path="$2"
  if [ ! -f "$target_path" ] || ! cmp -s "$source_path" "$target_path"; then
    install -m 0644 "$source_path" "$target_path"
    changed=true
  fi
}

install_if_changed \
  "$CONFIG_DIR/10-robopilot-xhci-noise.conf" \
  /etc/rsyslog.d/10-robopilot-xhci-noise.conf
install_if_changed \
  "$CONFIG_DIR/99-robopilot-journal.conf" \
  /etc/systemd/journald.conf.d/99-robopilot-journal.conf

# The xHCI driver can emit thousands of capture warnings per second. Filtering
# them after imklog has read them still burns a measurable amount of CPU, so
# keep normal syslog input but leave kernel diagnostics in the dmesg ring only.
if grep -Eq '^[[:space:]]*module\(load="imklog"' /etc/rsyslog.conf; then
  if [ ! -f /etc/rsyslog.conf.robopilot-backup ]; then
    cp -a /etc/rsyslog.conf /etc/rsyslog.conf.robopilot-backup
  fi
  sed -Ei \
    's@^([[:space:]]*module\(load="imklog".*)$@# Disabled by Robopilot USB log protection: \1@' \
    /etc/rsyslog.conf
  changed=true
fi

rsyslogd -N1
dmesg -n 3
if [ "$changed" = "true" ]; then
  systemctl restart systemd-journald
  systemctl restart rsyslog
fi

if [ "${CLEAN_OLD_LOGS:-true}" = "true" ]; then
  journalctl --rotate
  journalctl --vacuum-size=256M
  truncate -s 0 /var/log/kern.log
fi

echo "Robopilot host log limits ready (changed=$changed)."
