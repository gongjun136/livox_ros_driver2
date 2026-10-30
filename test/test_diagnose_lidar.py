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

    def state_row(self, seq=1, state=1, hms=(), session='42-first', **changes):
        fields = {key: None for key in d.HISTORY_FIELDS}
        fields.update(cur_work_state=state, hms_code=list(hms) + [0] * (8 - len(hms)))
        statuses = {key: 'valid' if value is not None else 'missing' for key, value in fields.items()}
        statuses.update(voltage='not_available', uptime='not_available')
        row = dict(schema=1, kind='livox_raw_state', role='right', pid=42, session=session,
                   seq=seq, wall_ns=1793325600000000000 + seq * 1_000_000_000,
                   steady_ns=seq * 1_000_000_000, source='query', query_result='success',
                   sdk_status=0, ret_code=0, fields=fields, field_status=statuses,
                   field_sources={key: 'query' if value is not None else 'unavailable' for key, value in fields.items()},
                   core_temp_unit='sdk_raw_unit_unspecified', dropped_total=0,
                   write_errors_total=0, suppressed_push_total=0)
        row.update(changes)
        return row

    def state_rows(self, rows, suffix='', tail=b''):
        path = self.run / ('diagnostics/right_state.jsonl' + suffix)
        path.parent.mkdir(exist_ok=True)
        path.write_bytes(b''.join(json.dumps(row).encode() + b'\n' for row in rows) + tail)
        return path

    def test_state_history_records_error_and_recovery_without_inventing_physical_cause(self):
        self.state_rows([self.state_row(), self.state_row(2, 4, (0x02160003, 0x02100003, 0x04050002)),
                         self.state_row(3)])
        report = d.analyze(self.run)
        history = report['device_history'][2]
        self.assertTrue(report['has_evidence'])
        self.assertEqual(history['records'], 3)
        self.assertEqual([t['changes']['cur_work_state']['after'] for t in history['transitions']], [1, 4, 1])
        self.assertFalse(history['sessions'][0]['error_at_first_visible_or_after_gap'])
        self.assertEqual(history['transitions'][1]['changes']['cur_work_state']['kind'], 'observed_change')
        self.assertEqual(history['transitions'][1]['evidence']['line'], 2)
        self.assertIsNone(report['roles'][2]['device']['work_state'])
        text = d.render(report)
        self.assertIn('历史最新值不代表当前状态', text)
        self.assertIn('0x04050002 是历史授时警告', text)
        self.assertIn('不能据此确定 PTP 是故障根因', text)

    def test_first_visible_error_is_not_fault_onset_and_hms_order_is_not_event_order(self):
        self.state_rows([self.state_row(8, 4, (0x02160003, 0x02100003)),
                         self.state_row(9, 4, (0x02100003, 0x02160003))])
        history = d.device_history(self.run, 'right')
        self.assertEqual(history['transition_count'], 1)
        self.assertTrue(history['sessions'][0]['error_at_first_visible_or_after_gap'])
        self.assertIn('不能把本行时间当故障开始', d.history_findings(history)[0]['message'])
        self.assertEqual(history['transitions'][0]['changes']['hms_code']['kind'], 'first_observed')

    def test_field_observation_times_sources_and_units_survive_query_missing_extra_fields(self):
        push = self.state_row(source='state_push')
        values = dict(core_temp=350, time_sync_type=1, local_time_now=10, last_sync_time=9,
                      time_offset=-123, powerup_cnt=9, sn='ARMCP580033938', version_app=[1, 2, 3, 4])
        push['fields'].update(values)
        push['field_status'].update({key: 'valid' for key in values})
        push['field_sources'].update({key: 'state_push' for key in values})
        query = self.state_row(2)
        query['field_status'].update({key: 'no_response_length' for key in values})
        self.state_rows([push, query])
        history = d.device_history(self.run, 'right')
        last = history['sessions'][0]['last_valid_fields']
        self.assertEqual(last['core_temp']['value'], 350)
        self.assertEqual(last['core_temp']['evidence']['seq'], 1)
        self.assertEqual(last['core_temp']['source'], 'state_push')
        self.assertEqual(last['core_temp']['unit'], 'sdk_raw_unit_unspecified')
        self.assertEqual(last['cur_work_state']['evidence']['seq'], 2)
        self.assertIsNone(history['latest']['fields']['core_temp'])
        self.assertEqual(history['missing_field_counts']['core_temp:no_response_length'], 1)
        message = d.history_findings(history)[0]['message']
        self.assertIn('core_temp 原始值=350，SDK 单位未声明', message)
        self.assertNotIn('350°C', message)
        self.assertIn('电压/设备 uptime 本 schema 不提供', message)
        self.assertIn('原始 time_offset=-123', message)

    def test_unknown_schema_invalid_fields_and_missing_hms_do_not_report_clear(self):
        missing = self.state_row(3)
        missing['fields']['hms_code'] = None
        missing['field_status']['hms_code'] = 'missing'
        missing['fields']['core_temp'] = 'overheat'
        missing['field_status']['core_temp'] = 'valid'
        self.state_rows([self.state_row(1, 4, (0x02160003,)), self.state_row(2, schema=99),
                         missing, self.state_row(4)])
        history = d.device_history(self.run, 'right')
        self.assertEqual(history['records'], 3)
        self.assertEqual(history['issue_counts']['unsupported_schema'], 1)
        self.assertEqual(history['missing_field_counts']['core_temp:invalid'], 1)
        self.assertIsNone(history['sessions'][0]['last_valid_fields'].get('core_temp'))
        last = history['transitions'][-1]['changes']['hms_code']
        self.assertIsNone(last['before'])
        self.assertEqual(last['kind'], 'observed_after_gap')
        self.assertTrue(history['gaps'])

    def test_multi_session_and_wall_clock_jump_use_seq_without_cross_session_recovery(self):
        first = self.state_row(1, 4, (0x02160003,))
        second = self.state_row(2, wall_ns=first['wall_ns'] - 20_000_000_000)
        third = self.state_row(1, session='99-restart', pid=99, wall_ns=first['wall_ns'] - 40_000_000_000)
        self.state_rows([first, second, third])
        history = d.device_history(self.run, 'right')
        self.assertEqual([s['session'] for s in history['sessions']], ['42-first', '99-restart'])
        self.assertEqual(history['latest']['pid'], 99)
        self.assertTrue(any(g['kind'] == 'wall_clock_jump' for g in history['gaps']))
        self.assertEqual(history['transitions'][1]['changes']['cur_work_state']['before'], 4)
        self.assertEqual(history['transitions'][2]['changes']['cur_work_state']['kind'], 'first_observed')
        self.assertIsNone(history['transitions'][2]['changes']['cur_work_state']['before'])

    def test_drop_and_write_error_break_continuity_but_suppressed_identical_push_does_not(self):
        self.state_rows([self.state_row(), self.state_row(2, suppressed_push_total=99),
                         self.state_row(3, 4, dropped_total=1),
                         self.state_row(4, write_errors_total=1, dropped_total=1)])
        history = d.device_history(self.run, 'right')
        self.assertEqual(history['gap_count'], 2)
        self.assertEqual(history['gaps'][0]['before']['seq'], 2)
        self.assertIn('dropped_total', history['gaps'][0]['reasons'])
        self.assertIn('write_errors_total', history['gaps'][1]['reasons'])
        self.assertIsNone(history['transitions'][1]['changes']['cur_work_state']['before'])
        self.assertTrue(history['sessions'][0]['error_at_first_visible_or_after_gap'])
        findings = d.history_findings(history)
        self.assertEqual(findings[-1]['code'], 'device_history_incomplete')
        self.assertIn('range_start', findings[-1]['evidence'][0])

    def test_rotation_budget_and_incomplete_tail_keep_newest_complete_evidence(self):
        self.state_rows([self.state_row(1)], '.3')
        self.state_rows([self.state_row(2)], '.2')
        self.state_rows([self.state_row(3)], '.1')
        current = self.state_rows([self.state_row(4, 4), self.state_row(5)], tail=b'{"schema":1')
        budget = current.stat().st_size + 50
        history = d.device_history(self.run, 'right', budget=budget)
        self.assertLessEqual(history['read_bytes'], budget)
        self.assertEqual(history['records'], 2)
        self.assertEqual(history['latest']['evidence']['seq'], 5)
        self.assertEqual(history['issue_counts']['incomplete_line'], 1)
        self.assertTrue(history['issue_counts']['scan_limit'])
        self.assertEqual(history['transitions'][0]['evidence']['line'], 1)
        tail_history = d.device_history(self.run, 'right', budget=current.stat().st_size - 10)
        self.assertEqual(tail_history['records'], 1)
        evidence = tail_history['latest']['evidence']
        self.assertIsNone(evidence['line'])
        self.assertGreater(evidence['byte_offset'], 0)

    def test_bad_middle_line_breaks_transition_and_all_invalid_history_is_not_health(self):
        good = self.state_row()
        path = self.state_rows([good])
        with path.open('ab') as stream:
            stream.write(b'not-json\n')
            stream.write(json.dumps(self.state_row(3, 4)).encode() + b'\n')
        history = d.device_history(self.run, 'right')
        self.assertEqual(history['issue_counts']['invalid_json'], 1)
        self.assertEqual(history['transitions'][1]['changes']['cur_work_state']['kind'], 'observed_after_gap')
        self.state_rows([dict(schema=999)])
        report = d.analyze(self.run)
        self.assertFalse(report['has_evidence'])
        self.assertEqual(report['device_history'][2]['status'], 'invalid')
        self.assertTrue(any(f['code'] == 'device_history_incomplete' for f in report['findings']))

    def test_old_bundle_has_no_new_fault_and_history_never_overwrites_live_query(self):
        self.evidence()
        old = d.analyze(self.run)
        self.assertTrue(all(h['status'] == 'missing' for h in old['device_history']))
        self.assertFalse(any(f['stage'] == 'device_history' for f in old['findings']))
        self.state_rows([self.state_row(1, 4, (0x02160003,))])
        query = Mock(return_value=self.device(1, []))
        with patch.object(d, 'process_alive', return_value=True):
            report = d.analyze(self.run, self.process, refresh=True, query_fn=query)
        self.assertEqual(query.call_count, 3)
        self.assertTrue(report['roles'][2]['device']['live'])
        self.assertEqual(report['roles'][2]['device']['work_state'], 1)
        self.assertEqual(report['device_history'][2]['latest']['fields']['cur_work_state'], 4)

    def test_requested_recorder_missing_or_without_observations_is_collection_failure(self):
        self.meta['device_state_history'] = dict(requested=True, directory='diagnostics',
                                                max_file_bytes=8 * d.MIB, files_per_role=4)
        self.write('run_metadata.json', self.meta)
        report = d.analyze(self.run)
        missing = [f for f in report['findings'] if f['code'] == 'device_history_unavailable']
        self.assertEqual(len(missing), 3)
        self.assertTrue(all('时间线文件缺失' in f['message'] for f in missing))
        self.assertFalse(report['has_evidence'])
        startup = self.state_row(source='startup', query_result='not_applicable')
        startup['fields'] = {key: None for key in d.HISTORY_FIELDS}
        startup['field_status'] = {key: 'missing' for key in d.HISTORY_FIELDS}
        error = dict(startup, seq=2, source='error', query_result='submit_failed')
        self.state_rows([startup, error])
        report = d.analyze(self.run)
        right = [f for f in report['findings'] if f['code'] == 'device_history_unavailable' and f['role'] == 'right']
        self.assertEqual(len(right), 1)
        self.assertIn('未包含有效 work_state/HMS', right[0]['message'])
        self.assertIn('不能据此认定雷达故障或健康', right[0]['message'])
        self.assertTrue(report['has_evidence'])  # recorded collector error, not healthy device evidence

    def test_partial_push_preserves_field_comparison_but_invalid_measurement_breaks_it(self):
        partial = self.state_row(2, source='state_push')
        partial['fields'] = {key: None for key in d.HISTORY_FIELDS}
        partial['field_status'] = {key: 'missing' for key in d.HISTORY_FIELDS}
        partial['fields']['core_temp'], partial['field_status']['core_temp'] = 12, 'valid'
        self.state_rows([self.state_row(), partial, self.state_row(3, 4)])
        history = d.device_history(self.run, 'right')
        change = history['transitions'][-1]['changes']['cur_work_state']
        self.assertEqual(change['before'], 1)
        self.assertEqual(change['after'], 4)
        self.assertEqual(change['kind'], 'observed_change')
        self.assertEqual(change['before_evidence']['seq'], 1)
        self.assertEqual(change['after_evidence']['seq'], 3)
        self.assertFalse(history['sessions'][0]['error_at_first_visible_or_after_gap'])
        partial['field_status']['cur_work_state'] = 'invalid'
        self.state_rows([self.state_row(), partial, self.state_row(3, 4)])
        history = d.device_history(self.run, 'right')
        self.assertIsNone(history['transitions'][-1]['changes']['cur_work_state']['before'])
        self.assertTrue(history['sessions'][0]['error_at_first_visible_or_after_gap'])

    def test_sync_type_and_powerup_changes_are_observed_without_offset_event_flood(self):
        rows = [self.state_row(seq) for seq in (1, 2, 3)]
        for row in rows:
            row['fields'].update(time_sync_type=1 if row['seq'] < 3 else 0,
                                 powerup_cnt=9 if row['seq'] < 3 else 10, time_offset=row['seq'] * 1000)
            row['field_status'].update(time_sync_type='valid', powerup_cnt='valid', time_offset='valid')
        self.state_rows(rows)
        history = d.device_history(self.run, 'right')
        self.assertEqual(history['transition_count'], 2)
        changes = history['transitions'][-1]['changes']
        self.assertEqual(changes['time_sync_type']['before'], 1)
        self.assertEqual(changes['time_sync_type']['after'], 0)
        self.assertEqual(changes['time_sync_type']['before_evidence']['seq'], 2)
        self.assertEqual(changes['powerup_cnt']['before'], 9)
        self.assertEqual(changes['powerup_cnt']['after'], 10)
        self.assertNotIn('time_offset', changes)
        message = d.history_findings(history)[0]['message']
        self.assertIn('原始授时类型 time_sync_type 1 → 0', message)
        self.assertIn('上电次数 powerup_cnt（不是 uptime） 9 → 10', message)

    def test_unknown_hms_level_is_not_called_error_and_failed_query_breaks_comparison(self):
        self.state_rows([self.state_row(hms=(0xFFFFFFFF,))])
        history = d.device_history(self.run, 'right')
        self.assertFalse(history['sessions'][0]['error_at_first_visible_or_after_gap'])
        failed = self.state_row(2, query_result='timeout')
        failed['fields'] = {key: None for key in d.HISTORY_FIELDS}
        failed['field_status'] = {key: 'no_response_length' for key in d.HISTORY_FIELDS}
        self.state_rows([self.state_row(), failed, self.state_row(3, 4)])
        history = d.device_history(self.run, 'right')
        change = history['transitions'][-1]['changes']['cur_work_state']
        self.assertIsNone(change['before'])
        self.assertEqual(change['kind'], 'observed_after_gap')
        self.assertIn('timeout', d.history_findings(history)[0]['message'])

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
