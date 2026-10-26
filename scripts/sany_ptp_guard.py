#!/usr/bin/env python3
"""Fail closed before SANY PTP starts; --check never changes a clock."""
import argparse
import csv
import math
import os
import re
import subprocess
import time


def run(*args):
    return subprocess.check_output(args, text=True, timeout=5,
                                   env={**os.environ, 'LC_ALL': 'C'}).strip()


def tracking_ok(text, now):
    """Validate chronyc -c tracking, including freshness and remaining correction."""
    try:
        row = next(csv.reader([text]))
        if len(row) != 14:
            return False
        values = [float(row[i]) for i in (3, 4, 5, 9, 10, 11)]
        if not all(math.isfinite(v) for v in values):
            return False
        ref, correction, offset, skew, delay, dispersion = values
        # [livox-ptp-quality]
        return (now >= 1735689600 and row[0] not in ('00000000', '7F7F0101')
                and 1 <= int(row[2]) < 16 and row[13] == 'Normal'
                and 0 <= now - ref <= 180
                and abs(correction) <= .01 and abs(offset) <= .01
                and 0 <= skew <= 100 and delay >= 0 and dispersion >= 0
                and abs(correction) + delay / 2 + dispersion <= .1)
        # [livox-ptp-quality]
    except (ValueError, IndexError, StopIteration):
        return False


def wait_ready(timeout, stable_seconds=5, command=run,
               realtime=time.time, monotonic=time.monotonic, sleep=time.sleep):
    deadline = monotonic() + timeout
    stable_since = None
    previous = None
    while monotonic() < deadline:
        now, mono = realtime(), monotonic()
        try:
            good = tracking_ok(command('chronyc', '-c', 'tracking'), now)
        except (OSError, subprocess.SubprocessError):
            good = False
        # A step in either direction resets the stability window.
        # [livox-ptp-stability]
        if previous and abs((now - previous[0]) - (mono - previous[1])) > .01:
            good = False
        previous = now, mono
        if good:
            if stable_since is None:
                stable_since = mono
            if mono - stable_since >= stable_seconds:
                return
        else:
            stable_since = None
        # [livox-ptp-stability]
        sleep(1)
    raise RuntimeError('No fresh, stable Chrony time; PTP startup refused')


def prepare(interface, timeout, command=run, waiter=wait_ready):
    if not re.fullmatch(r'eth[0-9]+', interface):
        raise ValueError('Expected an ethN interface')
    # Never step a PHC that is already being served or controlled.
    # [livox-ptp-prepare]
    for unit in (f'ptp_master_{interface}.service', f'ptp_master_{interface}_sys.service'):
        if command('systemctl', 'show', unit, '-p', 'MainPID', '--value') != '0':
            raise RuntimeError(f'{unit} is running; refusing PHC initialization')
    waiter(timeout)
    # Two-argument form changes future policy; it does NOT step CLOCK_REALTIME.
    command('chronyc', 'makestep', '1', '0')
    waiter(15)
    command('phc_ctl', interface, 'set')
    # [livox-ptp-prepare]
    print(f'{interface}: fresh Chrony time, automatic steps disabled, PHC initialized', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--check', action='store_true')
    group.add_argument('--prepare', metavar='ethN')
    parser.add_argument('--timeout', type=float, default=180)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be positive and finite')
    try:
        if args.check:
            wait_ready(args.timeout)
            print('Chrony startup gate passed (read-only)', flush=True)
        else:
            if os.geteuid() != 0:
                raise RuntimeError('--prepare requires root')
            prepare(args.prepare, args.timeout)
    except (RuntimeError, ValueError, OSError, subprocess.SubprocessError) as exc:
        parser.exit(1, f'{exc}\n')


if __name__ == '__main__':
    main()
