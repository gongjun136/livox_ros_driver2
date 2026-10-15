#!/usr/bin/env python3
"""Clock-gate regressions without root, ROS, hardware, or real clock changes."""
import importlib.util
from pathlib import Path
import subprocess
import os
import re
import shutil
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('guard', Path(__file__).resolve().parents[1] / 'scripts/sany_ptp_guard.py')
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)
NOW = 1791984000


def tracking(**overrides):
    row = ['C0A82C65', '192.168.44.101', '5', str(NOW-2), '.00001', '.00001',
           '90', '-13', '0', '.2', '.04', '.01', '64', 'Normal']
    for key, value in overrides.items():
        row[int(key)] = str(value)
    return ','.join(row)


class Clock:
    def __init__(self, jump=0):
        self.t = 0
        self.jump = jump

    def real(self):
        return NOW + self.t + (self.jump if self.t >= 3 else 0)

    def sleep(self, seconds):
        self.t += seconds


class GateTest(unittest.TestCase):
    def test_valid(self):
        self.assertTrue(guard.tracking_ok(tracking(), NOW))

    def test_bad_time_and_tracking(self):
        for change in ({'0':'7F7F0101'}, {'0':'00000000'}, {'2':16}, {'13':'Not synchronised'},
                       {'3':NOW-181}, {'3':NOW+1}, {'4':.1}, {'4':'nan'}, {'5':-.02},
                       {'9':101}, {'10':-1}, {'11':1}):
            with self.subTest(change=change):
                self.assertFalse(guard.tracking_ok(tracking(**change), NOW))
        self.assertFalse(guard.tracking_ok(tracking(), 294))
        self.assertFalse(guard.tracking_ok('506 Cannot talk to daemon', NOW))

    def wait(self, clock, cmd, timeout=12):
        guard.wait_ready(timeout, command=cmd, realtime=clock.real,
                         monotonic=lambda:clock.t, sleep=clock.sleep)

    def test_initial_bad_epoch_then_sync(self):
        clock = Clock()
        self.wait(clock, lambda *a: tracking(**{'13':'Not synchronised'}) if clock.t < 3 else tracking())
        self.assertGreaterEqual(clock.t, 8)

    def test_step_restarts_stability_window(self):
        clock = Clock(jump=.5)
        self.wait(clock, lambda *a: tracking())
        self.assertGreaterEqual(clock.t, 9)

    def test_unsynced_times_out_without_side_effects(self):
        for cmd in (lambda *a:'invalid', lambda *a: (_ for _ in ()).throw(subprocess.TimeoutExpired(a,5))):
            with self.assertRaises(RuntimeError):
                self.wait(Clock(), cmd, timeout=4)

    def test_prepare_order(self):
        events=[]
        def command(*args):
            events.append(args)
            return '0' if args[0]=='systemctl' else '200 OK'
        guard.prepare('eth1', 180, command, lambda n:events.append(('wait',n)))
        self.assertEqual(events[-4:], [('wait',180),('chronyc','makestep','1','0'),
                                      ('wait',15),('phc_ctl','eth1','set')])

    def test_running_controller_refuses_clock_write(self):
        with self.assertRaises(RuntimeError):
            guard.prepare('eth1', 1, lambda *a:'123')

    def test_failed_policy_change_prevents_phc_write(self):
        events=[]
        def command(*args):
            events.append(args)
            if args[0]=='chronyc':raise subprocess.CalledProcessError(1,args)
            return '0'
        with self.assertRaises(subprocess.CalledProcessError):
            guard.prepare('eth1', 1, command, lambda n:None)
        self.assertFalse(any(e[0]=='phc_ctl' for e in events))

    @unittest.skipUnless(shutil.which('systemd-analyze'), 'Linux systemd tooling required')
    def test_generated_service_dependency_graph(self):
        source = (Path(__file__).resolve().parents[1] / 'scripts/install_sany_ptp_guard.sh').read_text()
        templates = re.findall(r'<<EOF\n(.*?)\nEOF', source, re.S)
        self.assertEqual(len(templates), 2)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'chrony.service').write_text('[Service]\nExecStart=/bin/sleep infinity\n')
            units = []
            for iface in ('eth1', 'eth2', 'eth3'):
                for suffix, template in zip(('', '_sys'), templates):
                    name = f'ptp_master_{iface}{suffix}.service'
                    units.append(str(root/name))
                    (root/name).write_text('[Service]\nExecStart=/bin/sleep infinity\n')
                    dropin = root/(name+'.d')
                    dropin.mkdir()
                    # Validate the real installer dependency graph on hosts without
                    # PHC hardware; substitute only executable paths, not ordering.
                    content = template.replace('${iface}',iface).replace('$iface',iface)
                    content = content.replace('$(command -v phc2sys)', '/bin/true')
                    content = content.replace('/usr/local/sbin/sany-ptp-guard','/bin/true')
                    (dropin/'50-sany-time-guard.conf').write_text(content)
            result = subprocess.run(['systemd-analyze','verify',*units], text=True,
                capture_output=True, timeout=20,
                env={**os.environ,'SYSTEMD_UNIT_PATH':directory+':/usr/lib/systemd/system:/lib/systemd/system'})
            self.assertEqual(result.returncode,0,result.stderr)


if __name__ == '__main__':
    unittest.main()
