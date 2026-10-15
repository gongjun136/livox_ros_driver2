#!/usr/bin/env bash
# Explicit maintenance operation. Does not stop/start services or reboot LiDARs.
set -euo pipefail
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
if [[ ${EUID} != 0 || $# != 1 || $1 != --install ]]; then
  echo "Usage: sudo bash $0 --install (services must already be stopped)" >&2
  exit 1
fi
script_dir=$(cd -- "$(dirname -- "$0")" && pwd)
for iface in eth1 eth2 eth3; do
  for unit in "ptp_master_${iface}" "ptp_master_${iface}_sys"; do
    [[ $(systemctl show "$unit.service" -p LoadState --value) == loaded ]]
    state=$(systemctl show "$unit.service" -p ActiveState --value)
    [[ $state == inactive || $state == failed ]] || {
      echo "Stop $unit.service before installing; no changes made" >&2; exit 1;
    }
  done
done
python3 "$script_dir/sany_ptp_guard.py" --check --timeout 30
command -v phc_ctl >/dev/null
command -v phc2sys >/dev/null
backup=$(mktemp -d /var/backups/sany-ptp-guard.XXXXXXXX)
cp -a /etc/chrony/chrony.conf "$backup/chrony.conf"
cp -a /etc/adjtime "$backup/adjtime"
for iface in eth1 eth2 eth3; do
  for unit in "ptp_master_${iface}" "ptp_master_${iface}_sys"; do
    systemctl cat "$unit.service" > "$backup/$unit.effective.txt"
    dropin="/etc/systemd/system/$unit.service.d/50-sany-time-guard.conf"
    if [[ -e $dropin ]]; then cp -a "$dropin" "$backup/$unit.dropin"; fi
  done
done
if [[ -e /usr/local/sbin/sany-ptp-guard ]]; then
  cp -a /usr/local/sbin/sany-ptp-guard "$backup/sany-ptp-guard"
fi
echo "Backup: $backup"
# Validate the expanded configuration too: includes must not override this policy.
expanded=$(chronyd -p -f /etc/chrony/chrony.conf)
[[ $(printf '%s\n' "$expanded" | grep -c '^makestep ') == 1 ]]
grep -Eq '^[[:space:]]*makestep[[:space:]]' /etc/chrony/chrony.conf
sed -i -E 's/^[[:space:]]*makestep[[:space:]].*/makestep 1.0 3/' /etc/chrony/chrony.conf
install -m 0755 "$script_dir/sany_ptp_guard.py" /usr/local/sbin/sany-ptp-guard
for iface in eth1 eth2 eth3; do
  mkdir -p "/etc/systemd/system/ptp_master_${iface}.service.d"
  cat > "/etc/systemd/system/ptp_master_${iface}.service.d/50-sany-time-guard.conf" <<EOF
[Unit]
BindsTo=chrony.service
After=chrony.service
Wants=ptp_master_${iface}_sys.service

[Service]
ExecStartPre=/usr/local/sbin/sany-ptp-guard --prepare $iface
TimeoutStartSec=240
EOF
  mkdir -p "/etc/systemd/system/ptp_master_${iface}_sys.service.d"
  cat > "/etc/systemd/system/ptp_master_${iface}_sys.service.d/50-sany-time-guard.conf" <<EOF
[Unit]
BindsTo=ptp_master_${iface}.service
PartOf=ptp_master_${iface}.service
After=ptp_master_${iface}.service

[Service]
ExecStart=
ExecStart=$(command -v phc2sys) -s CLOCK_REALTIME -c $iface -w -E linreg -S 0 -F 0 -O 0
EOF
done
# Current time was checked above; update RTC, never adjust system time from RTC.
timedatectl set-local-rtc 0
chronyc makestep 1 0
systemctl daemon-reload
echo "Installed; no services restarted. Start ptp_master_eth{1,2,3}.service in the maintenance window."
echo "Keep $backup for rollback; see docs/operations/time_sync.md."
