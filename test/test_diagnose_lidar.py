#!/usr/bin/env python3
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
import diagnose_lidar as d


class DiagnoseTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.run = self.root / 'lidar_20261030_095912_master'
        (self.run / 'checks').mkdir(parents=True)
        (self.run / 'logs').mkdir()
        self.meta = dict(owner=d.OWNER, roles=['front', 'left', 'right'], role='master',
                         hostname='ubuntuM', status='running', started_at='2026-10-30T01:59:12+00:00',
                         ros_environment=dict(ROS_DOMAIN_ID='42', FASTRTPS_DEFAULT_PROFILES_FILE='/profile.xml'))
        self.write('run_metadata.json', self.meta)
        self.process = dict(pid=42, start_ticks=100, run=str(self.run))

    def write(self, name, value):
        path = self.run / name
        path.parent.mkdir(exist_ok=True, parents=True)
        path.write_text(json.dumps(value), encoding='utf-8')

    def device(self, state=4, hms=None, when='2026-10-30T01:59:39+00:00'):
        return dict(collected_at=when, parameters={k: dict(return_code=0, value=v) for k, v in
            dict(serial_number='TEST_SN', work_state=state,
                 hms=hms if hms is not None else ['0x02160003', '0x02100003', '0x04050002']).items()})

    def evidence(self):
        topics = {}
        for role, ip in d.IPS.items():
            if role == 'rear':
                continue
            for kind in ('lidar', 'imu'):
                topics['/livox/' + kind + '_' + ip.replace('.', '_')] = dict(
                    count=0 if role == 'right' else 200, hz=0 if role == 'right' else 10,
                    passed=role != 'right', nonmonotonic=0, empty_clouds=0)
        self.write('checks/startup_check.json', dict(passed=False, topics=topics))
        self.write('checks/device_state.json', {role: self.device(4 if role == 'right' else 1,
            None if role == 'right' else ['0x04050002']) for role in self.meta['roles']})

    def test_native_scan_errors_and_history_are_distinct(self):
        self.evidence()
        report = d.analyze(self.run)
        self.assertFalse(report['active'])
        self.assertEqual([r['role'] for r in report['roles']], ['front', 'left', 'right'])
        right = report['roles'][2]
        self.assertEqual(right['startup']['cloud']['count'], 0)
        self.assertEqual(right['device']['work_state_name'], 'Error')
        self.assertEqual(right['device']['hms'][0]['category'], 'scan_module')
        self.assertEqual(right['device']['hms'][0]['level'], 3)
        history = [f for f in report['findings'] if '0x04050002' in f['message']]
        self.assertTrue(history)
        self.assertTrue(all(f['severity'] == 'info' for f in history))
        self.assertIn('/profile.xml', d.render(report))
        self.assertIn('历史设备快照', d.render(report))

    def test_explicit_path_never_refreshes_even_when_process_exists(self):
        self.evidence()
        run, process, _ = d.select(str(self.run), processes=[self.process])
        self.assertEqual(run, self.run)
        self.assertIsNone(process)
        query = Mock(side_effect=AssertionError('network forbidden'))
        report = d.analyze(run, process, refresh=True, query_fn=query)
        query.assert_not_called()
        self.assertFalse(report['roles'][2]['device']['live'])

    def test_live_refresh_is_bounded_to_expected_roles_and_reports_query_errors(self):
        self.evidence()
        query = Mock(side_effect=[self.device(1, []), self.device(1, []),
                                 dict(parameters={'work_state': {'error': 'timed out'}})])
        with patch.object(d, 'process_alive', return_value=True):
            report = d.analyze(self.run, self.process, refresh=True, query_fn=query)
        self.assertEqual(query.call_count, 3)
        self.assertEqual([call.args[0] for call in query.call_args_list], [d.IPS[r] for r in self.meta['roles']])
        right = report['roles'][2]['device']
        self.assertIsNone(right['work_state'])
        self.assertTrue(right['live'])
        self.assertIn('timed out', right['errors']['work_state'])
        self.assertNotEqual(right['work_state'], 1)

    def test_no_refresh_does_not_claim_current_data_healthy(self):
        self.evidence()
        query = Mock(side_effect=AssertionError('network forbidden'))
        with patch.object(d, 'process_alive', return_value=True):
            report = d.analyze(self.run, self.process, refresh=False, query_fn=query)
        query.assert_not_called()
        self.assertTrue(report['active'])
        self.assertIn('不能证明当前仍在出数', '\n'.join(report['warnings']))

    def test_newer_stored_recheck_wins_per_role(self):
        self.evidence()
        self.write('recheck_1005/checks/failure_state.json',
                   {'right': self.device(1, [], '2026-10-30T02:05:00+00:00')})
        report = d.analyze(self.run)
        right = report['roles'][2]['device']
        self.assertEqual(right['work_state'], 1)
        self.assertIn('recheck_1005', right['source'])
        self.assertEqual(report['roles'][0]['device']['work_state'], 1)
        self.assertEqual(report['roles'][2]['startup']['cloud']['count'], 0)

    def test_missing_state_does_not_invent_normal_or_hms_clear(self):
        report = d.analyze(self.run)
        self.assertFalse(report['has_evidence'])
        self.assertIsNone(report['roles'][0]['device']['work_state'])
        self.assertIsNone(report['roles'][0]['device']['hms'])
        self.assertTrue(any(f['code'] == 'device_evidence_incomplete' for f in report['findings']))

    def test_identity_failure_survives_other_success_and_uses_recorded_role(self):
        self.meta['roles'] = ['front']
        self.write('run_metadata.json', self.meta)
        self.evidence()
        self.write('checks/result.json', dict(passed=False, data_ok=True, device_state_ok=True, identity_ok=False))
        self.write('checks/identity.json', dict(collected_at_utc='2026-10-30T02:00:00+00:00',
                   devices=[dict(role='front', identity_valid=False, status='invalid_response')]))
        report = d.analyze(self.run)
        failure = next(f for f in report['findings'] if f['code'] == 'startup_identity_failed')
        self.assertEqual(failure['role'], 'front')
        self.assertIn('该次身份检查未通过', failure['message'])
        self.assertEqual(failure['evidence'][0]['time'], '2026-10-30T02:00:00+00:00')
        self.assertTrue(failure['evidence'][0]['path'].endswith('result.json'))
        self.assertTrue(failure['evidence'][1]['path'].endswith('identity.json'))
        self.assertFalse(any(f['code'] == 'startup_stream_missing' for f in report['findings']))
        self.write('checks/result.json', dict(identity_ok=True))
        self.assertFalse(any(f['code'] == 'startup_identity_failed' for f in d.analyze(self.run)['findings']))

    def test_summary_failure_is_evidence_when_detail_files_are_missing(self):
        self.write('checks/result.json', dict(passed=False, data_ok=False, device_state_ok=False, identity_ok=False))
        report = d.analyze(self.run)
        self.assertTrue(report['has_evidence'])
        failures = [f for f in report['findings'] if f['code'] in ('startup_identity_failed', 'startup_check_failed')]
        self.assertEqual(len(failures), 3)
        self.assertTrue(all(f['role'] is None for f in failures))
        self.assertTrue(all(f['evidence'][0]['time'] is None for f in failures))
        self.assertTrue(all('检查时间未记录' in f['message'] for f in failures))

    def test_reused_or_ended_worker_cannot_refresh(self):
        query = Mock(side_effect=AssertionError('network forbidden'))
        with patch.object(d, 'process_alive', return_value=False):
            report = d.analyze(self.run, self.process, refresh=True, query_fn=query)
        query.assert_not_called()
        self.assertFalse(report['active'])

    def test_worker_identity_uses_argv_and_start_ticks_not_thread_name(self):
        proc = self.root / 'proc'
        entry = proc / '42'
        entry.mkdir(parents=True)
        (entry / 'cmdline').write_bytes(('python3\0/install/run_lidar_bundle.py\0--worker\0' +
                                       str(self.run) + '\0launch.py\0vehicle.yaml\0').encode())
        (entry / 'stat').write_text('42 (renamed worker) ' + ' '.join(['0'] * 19 + ['1234']))
        (entry / 'environ').write_bytes(b'ROS_DOMAIN_ID=42\0SECRET=not-exported\0')
        result = d.worker_processes(proc)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]['start_ticks'], 1234)
        self.assertEqual(result[0]['ros_environment'], {'ROS_DOMAIN_ID': '42'})

    def test_active_worker_wins_over_newer_historical_directory(self):
        selected, process, _ = d.select(root=str(self.root), processes=[self.process])
        self.assertEqual(selected, self.run)
        self.assertEqual(process['pid'], 42)

    def test_no_process_selects_latest_start_time(self):
        older = self.root / 'lidar_older'
        older.mkdir()
        (older / 'run_metadata.json').write_text(json.dumps(dict(self.meta, started_at='2026-10-29T00:00:00+00:00')))
        selected, process, _ = d.select(root=str(self.root), processes=[])
        self.assertEqual(selected, self.run)
        self.assertIsNone(process)

    def test_multiple_workers_are_disclosed(self):
        selected, process, selection = d.select(processes=[self.process, dict(self.process, pid=21)])
        with patch.object(d, 'process_alive', return_value=True):
            report = d.analyze(selected, process, selection)
        self.assertIn('2 个本机雷达 worker', '\n'.join(report['warnings']))
        self.assertIn('PID 42', '\n'.join(report['warnings']))

    def test_sigint_before_nonzero_exit_is_shutdown_not_runtime_root(self):
        self.write('run_metadata.json', dict(self.meta, stop_requested=True))
        (self.run / 'events.jsonl').write_text(json.dumps(dict(event='node_exited', pid=42, name='right', returncode=-11, time='t')) + '\n')
        (self.run / 'logs/driver.log').write_text(
            '[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)\n'
            '[ERROR] [right]: process has died [pid 42, exit code -11, cmd right].\n')
        report = d.analyze(self.run)
        exit_row = next(f for f in report['findings'] if f['code'] == 'node_nonzero_exit')
        self.assertTrue(report['has_evidence'])
        self.assertEqual(exit_row['stage'], 'shutdown')
        self.assertIn('不能作为此前缺数据的根因', exit_row['message'])

    def test_stop_requested_flag_alone_does_not_assign_exit_phase(self):
        self.write('run_metadata.json', dict(self.meta, stop_requested=True))
        (self.run / 'events.jsonl').write_text(json.dumps(dict(event='node_exited', pid=42, returncode=-9)) + '\n')
        findings = d.lifecycle(self.run, dict(stop_requested=True), [])
        self.assertEqual(findings[0]['stage'], 'lifecycle_unknown')

    def test_json_is_plain_and_success_does_not_mean_healthy(self):
        self.evidence()
        with patch.object(d, 'low_impact', return_value={}), patch('sys.stdout', new_callable=io.StringIO) as output:
            status = d.main([str(self.run), '--json', '--color', 'always'])
        self.assertEqual(status, 0)
        self.assertNotIn('\033', output.getvalue())
        report = json.loads(output.getvalue())
        self.assertEqual(report['kind'], 'livox_driver_diagnosis')
        self.assertTrue(any(f['severity'] == 'error' for f in report['findings']))

    def test_bad_path_json_is_failure_without_false_health(self):
        with patch.object(d, 'low_impact', return_value={}), patch('sys.stdout', new_callable=io.StringIO) as output:
            status = d.main([str(self.root / 'missing'), '--json'])
        self.assertEqual(status, 2)
        self.assertFalse(json.loads(output.getvalue())['has_evidence'])

    def test_source_wrapper_dispatch_precedes_ros_setup_and_forwards_options(self):
        text = (SCRIPTS / 'run_lidar.sh').read_text()
        self.assertLess(text.index('== --diagnose'), text.index('source /opt/ros'))
        self.assertIn('exec python3 "$script_dir/diagnose_lidar.py" "$@"', text)
        self.assertIn('--vehicle-config "${SANY_VEHICLE_CONFIG:-/opt/conf/vehicle_config.yaml}" "$@"', text)


if __name__ == '__main__':
    unittest.main()
