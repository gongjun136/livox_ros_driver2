#!/usr/bin/env python3
"""Read existing LiDAR evidence; live refresh uses bounded read-only device queries."""
import argparse
from collections import Counter
import datetime as dt
import json
import os
from pathlib import Path
import re
import sys
import time

from lidar_bundle_check import IPS, query
from run_lidar_bundle import OWNER

MIB = 1024 * 1024
PROC = Path('/proc')
ROS_KEYS = ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY',
            'FASTRTPS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI')
WORK_STATES = {1: 'Normal', 2: 'WakeUp', 3: 'Sleep', 4: 'Error',
               5: 'PowerOnSelfTest', 6: 'MotorStarting', 7: 'MotorStopping', 8: 'Upgrade'}


def read_text(path, limit=MIB):
    with Path(path).open('rb') as stream:
        return stream.read(limit).decode('utf-8', errors='replace')


def read_json(path, warnings=None):
    try:
        if Path(path).stat().st_size > MIB:
            raise ValueError('JSON exceeds 1 MiB budget')
        value = json.loads(read_text(path))
        if not isinstance(value, dict):
            raise ValueError('expected JSON object')
        return value
    except (OSError, ValueError) as error:
        if warnings is not None and Path(path).exists():
            warnings.append(f'无法读取 {path}: {error}')
        return {}


def timestamp(value):
    try:
        parsed = dt.datetime.fromisoformat(value.replace('Z', '+00:00'))
        return parsed.timestamp() if parsed.tzinfo else None
    except (AttributeError, ValueError):
        return None


def low_impact():
    result = {}
    try:
        os.nice(max(0, 10 - os.getpriority(os.PRIO_PROCESS, 0)))
        result['nice'] = os.getpriority(os.PRIO_PROCESS, 0)
        isolated = set()
        for item in read_text('/sys/devices/system/cpu/isolated').strip().split(','):
            if item:
                ends = [int(n) for n in item.split('-')]
                isolated.update(range(ends[0], ends[-1] + 1))
        allowed = os.sched_getaffinity(0) - isolated
        if not allowed:
            raise RuntimeError('检查器仅允许使用隔离核，请从普通终端运行。')
        os.sched_setaffinity(0, allowed)
        result.update(cpus=sorted(allowed), isolated=sorted(isolated))
    except (AttributeError, OSError) as error:
        result['note'] = '调度限制未完全应用: ' + str(error)
    return result


def worker_processes(proc=PROC):
    found = []
    if not proc.is_dir():
        return found
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            args = read_text(entry / 'cmdline', 65536).split('\0')
            if not any(Path(a).name == 'run_lidar_bundle.py' for a in args) or '--worker' not in args:
                continue
            run = Path(args[args.index('--worker') + 1])
            if not run.is_absolute():
                run = Path(os.readlink(entry / 'cwd')) / run
            run = run.resolve()
            if read_json(run / 'run_metadata.json').get('owner') != OWNER:
                continue
            ticks = int(read_text(entry / 'stat').rsplit(')', 1)[1].split()[19])
            env = dict(item.split('=', 1) for item in read_text(entry / 'environ').split('\0') if '=' in item)
            found.append(dict(pid=int(entry.name), run=str(run), start_ticks=ticks,
                              ros_environment={k: env[k] for k in ROS_KEYS if k in env}))
        except (OSError, ValueError, IndexError):
            continue
    return sorted(found, key=lambda p: p['start_ticks'], reverse=True)


def process_alive(process):
    return bool(process and any(p['pid'] == process['pid'] and
                               p['start_ticks'] == process['start_ticks'] and
                               p['run'] == process['run'] for p in worker_processes()))


def default_roots(env=None, home=None):
    env = dict(os.environ if env is None else env)
    home = Path.home() if home is None else Path(home)
    config = {'HOME': str(home)}
    try:
        for line in read_text(home / '.config/sany/workspace.bash').splitlines():
            match = re.match(r'^export\s+(\w+)=(.+?)\s*$', line)
            if match:
                value = match[2].strip('"\'')
                value = re.sub(r'\$\{(\w+)\}|\$(\w+)', lambda m: config.get(m[1] or m[2], m[0]), value)
                if '$' not in value and '`' not in value:
                    config[match[1]] = value
    except OSError:
        pass
    config.update(env)
    return list(dict.fromkeys(Path(p).expanduser().resolve() for p in
        (config.get('LIGHTNING_LM_OUT_ROOT'),
         str(Path(config['SANY_WS']) / 'runs') if config.get('SANY_WS') else None,
         str(home / 'Sany/runs')) if p))


def select(path=None, root=None, processes=None):
    if path:
        selected = Path(path).expanduser().resolve()
        if selected.name == 'run_metadata.json':
            selected = selected.parent
        if not (selected / 'run_metadata.json').is_file():
            raise ValueError('指定路径不是雷达运行包: ' + str(selected))
        return selected, None, '指定历史路径（离线）'
    processes = worker_processes() if processes is None else processes
    if processes:
        selected = dict(processes[0], instance_count=len(processes))
        return Path(selected['run']), selected, '运行中的本机雷达实例'
    for directory in ([Path(root).expanduser().resolve()] if root else default_roots()):
        candidates = []
        for candidate in directory.glob('lidar_*'):
            meta = read_json(candidate / 'run_metadata.json')
            if meta.get('owner') == OWNER:
                candidates.append((timestamp(meta.get('started_at')) or candidate.stat().st_mtime, candidate))
        if candidates:
            return max(candidates, key=lambda item: item[0])[1], None, '无雷达进程，默认目录最新包（历史）'
    raise ValueError('未找到本机雷达 worker 或运行包；可指定雷达包路径或 --root。')


def hms_detail(raw):
    try:
        value = int(raw, 16) if isinstance(raw, str) else int(raw)
        if not 0 < value <= 0xFFFFFFFF:
            raise ValueError()
    except (ValueError, TypeError, OverflowError):
        return dict(code=str(raw), category='unknown', label='无法解析原生 HMS', level=None, historical=False)
    identifier, level = value >> 16, value & 0xFF
    history = identifier in (0x0401, 0x0405)
    if 0x0210 <= identifier <= 0x0219 or identifier == 0x021C:
        category, label = 'scan_module', '扫描模块异常'
    elif identifier == 0x0405:
        category, label = 'synchronization_history', '曾发生时间同步异常（历史项）'
    elif identifier == 0x0401:
        category, label = 'ethernet_history', '以太网历史项'
    elif identifier in (0x0402, 0x0403, 0x0404) or 0x0406 <= identifier <= 0x040A:
        category, label = 'time_sync', '时间同步诊断'
    else:
        category, label = 'other_native', '其他原生 HMS（未进一步解释）'
    return dict(code=f'0x{value:08x}', category=category, label=label, level=level,
                level_name={2: 'Warning', 3: 'Error', 4: 'Fatal'}.get(level, 'Unknown'),
                historical=history)


def device_record(raw, source, live=False):
    parameters = raw.get('parameters', {})
    values, errors = {}, {}
    for key in ('serial_number', 'work_state', 'hms'):
        item = parameters.get(key, {})
        if item.get('return_code') == 0 and 'value' in item:
            values[key] = item['value']
        else:
            errors[key] = item.get('error', '返回码 ' + str(item.get('return_code', '未知')))
    collected = raw.get('collected_at')
    when = timestamp(collected)
    hms = values.get('hms')
    if hms is not None and not isinstance(hms, list):
        errors['hms'] = 'HMS 字段格式无效'
        hms = None
    state = values.get('work_state')
    return dict(source=source, collected_at=collected, age_sec=max(0, time.time() - when) if when else None,
                serial_number=values.get('serial_number'), work_state=state,
                work_state_name=WORK_STATES.get(state, '未知'),
                hms=[hms_detail(c) for c in hms] if hms is not None else None,
                errors=errors, live=live,
                evidence_scope='live_query' if live else 'stored_snapshot' if raw else 'missing')


def history_lines(run, role, budget=4 * MIB):
    """Read the newest bounded bytes of the four recorder files, in file order."""
    remaining, records_left, files, chunks, issues = budget, 20000, [], [], []
    base = run / 'diagnostics' / (role + '_state.jsonl')
    for path in [base] + [Path(str(base) + '.' + str(i)) for i in range(1, 4)]:
        try:
            with path.open('rb') as stream:
                size = os.fstat(stream.fileno()).st_size
                take = min(size, remaining)
                start = size - take
                stream.seek(start)
                data = stream.read(take)
            remaining -= len(data)
            files.append(dict(path=str(path), size_bytes=size, read_bytes=len(data),
                              start_byte=start, truncated=start > 0))
            if start:
                issues.append(dict(kind='scan_limit', path=str(path), start_byte=start,
                                   message='容量限制：该文件更早记录未读取。'))
                # A tail may start inside UTF-8 or JSON; never join it to another record.
                cut = data.find(b'\n')
                data = data[cut + 1:] if cut >= 0 else b''
                start += cut + 1 if cut >= 0 else take
            full_prefix = files[-1]['start_byte'] == 0
            line_count = data.count(b'\n') + bool(data and not data.endswith(b'\n'))
            if line_count > records_left:
                tail = b'\n'.join(data.rsplit(b'\n', records_left)[1:]) if records_left else b''
                start += len(data) - len(tail)
                data, full_prefix = tail, False
                issues.append(dict(kind='record_limit', path=str(path), start_byte=start,
                                   message='仅分析最近最多 20000 行原始状态记录。'))
            records_left -= data.count(b'\n') + bool(data and not data.endswith(b'\n'))
            chunks.append((path, start, data, full_prefix))
        except FileNotFoundError:
            continue
        except OSError as error:
            issues.append(dict(kind='read_error', path=str(path), message=str(error)))
    lines = []
    for path, offset, data, full_prefix in reversed(chunks):
        for index, raw in enumerate(data.splitlines(keepends=True), 1):
            source = dict(path=str(path), line=index if full_prefix else None,
                          line_in_slice=index, byte_offset=offset)
            offset += len(raw)
            if not raw.endswith(b'\n') or len(raw) > 65536:
                issues.append(dict(kind='incomplete_line' if not raw.endswith(b'\n') else 'oversized_line',
                                   **source, message='跳过未写完或超过 64 KiB 的原始状态行。'))
                lines.append((None, source))  # break continuity across missing evidence
                continue
            try:
                value = json.loads(raw)
                if not isinstance(value, dict):
                    raise ValueError('expected object')
            except (ValueError, UnicodeError) as error:
                issues.append(dict(kind='invalid_json', **source, message=str(error)))
                value = None
            lines.append((value, source))
    return lines, files, issues


HISTORY_FIELDS = ('hms_code', 'cur_work_state', 'time_sync_type', 'local_time_now',
                  'last_sync_time', 'time_offset', 'core_temp', 'powerup_cnt', 'sn',
                  'version_app', 'version_loader', 'version_hardware', 'voltage', 'uptime')


def history_time(wall_ns):
    try:
        return dt.datetime.fromtimestamp(wall_ns / 1e9, dt.timezone.utc).astimezone().isoformat(timespec='milliseconds')
    except (ValueError, OSError, OverflowError):
        return '墙钟时间无法转换'


def history_field_valid(key, value):
    if key in ('voltage', 'uptime'):
        return False  # schema 1 does not expose these measurements
    if key == 'sn':
        return isinstance(value, str) and 0 < len(value) <= 128
    if key == 'hms_code' or key.startswith('version_'):
        length, maximum = (8, 0xFFFFFFFF) if key == 'hms_code' else (4, 255)
        return isinstance(value, list) and len(value) == length and all(type(v) is int and 0 <= v <= maximum for v in value)
    bounds = {'cur_work_state': (0, 255), 'time_sync_type': (0, 255),
              'time_offset': (-(2**63), 2**63 - 1), 'core_temp': (-(2**31), 2**31 - 1),
              'powerup_cnt': (0, 2**32 - 1)}
    lower, upper = bounds.get(key, (0, 2**64 - 1))
    return type(value) is int and lower <= value <= upper


def device_history(run, role, budget=4 * MIB):
    lines, files, issues = history_lines(run, role, budget)
    sessions, barriers, accepted = {}, 0, 0
    missing = Counter()
    query_results = Counter()
    for raw, source in lines:
        if raw is None:
            barriers += 1
            continue
        if type(raw.get('schema')) is not int or raw['schema'] != 1 or raw.get('kind') != 'livox_raw_state':
            issues.append(dict(kind='unsupported_schema', **source,
                               message='未知原始状态 schema/kind，未解释该行。'))
            barriers += 1
            continue
        integer_keys = ('pid', 'seq', 'wall_ns', 'steady_ns')
        if (raw.get('role') != role or not isinstance(raw.get('session'), str) or not raw['session'] or
                any(type(raw.get(k)) is not int or raw[k] < 0 for k in integer_keys) or
                not isinstance(raw.get('fields'), dict) or not isinstance(raw.get('field_status'), dict)):
            issues.append(dict(kind='invalid_record', **source, message='角色、会话、时间或字段结构无效，未跨该行推断转换。'))
            barriers += 1
            continue
        values, statuses = {}, {}
        for key in HISTORY_FIELDS:
            status, value = raw['field_status'].get(key, 'missing'), raw['fields'].get(key)
            if status == 'valid' and not history_field_valid(key, value):
                status = 'invalid'
            elif status not in ('valid', 'missing', 'invalid', 'not_available', 'no_response_length'):
                status = 'unknown_status'
            statuses[key] = status
            values[key] = value if status == 'valid' else None
            if status != 'valid':
                missing[key + ':' + status] += 1
        counters = {key: raw.get(key) if type(raw.get(key)) is int and raw[key] >= 0 else None
                    for key in ('dropped_total', 'write_errors_total', 'suppressed_push_total')}
        evidence = dict(source, time=history_time(raw['wall_ns']), wall_ns=raw['wall_ns'],
                        steady_ns=raw['steady_ns'], seq=raw['seq'], session=raw['session'])
        def scalar(key, kind):
            return raw.get(key) if type(raw.get(key)) is kind else None
        field_sources = raw.get('field_sources')
        field_sources = field_sources if isinstance(field_sources, dict) else {}
        record = dict(evidence=evidence, pid=raw['pid'], source=scalar('source', str),
                      query_result=scalar('query_result', str), fields=values, field_status=statuses,
                      sdk_status=scalar('sdk_status', int), ret_code=scalar('ret_code', int),
                      handle=scalar('handle', int), ip=scalar('ip', str), detail=scalar('detail', str),
                      writer_scheduling=scalar('writer_scheduling', str),
                      field_sources={k: field_sources[k] if isinstance(field_sources.get(k), str)
                                     else 'unrecorded' for k in HISTORY_FIELDS},
                      core_temp_unit='sdk_raw_unit_unspecified', counters=counters, barrier=barriers)
        query_results[str(raw.get('query_result', 'missing'))] += 1
        sessions.setdefault(raw['session'], []).append(record)
        accepted += 1
    summaries, transitions, gaps = [], [], []
    for session, records in sessions.items():
        records.sort(key=lambda row: (row['evidence']['seq'], row['evidence']['steady_ns']))
        first, previous, last_values, last_evidence = records[0], None, {}, {}
        session_transitions, first_error, last_valid_fields, seen_fields = [], False, {}, set()
        for row in records:
            evidence, values = row['evidence'], row['fields']
            for key, value in values.items():
                if value is not None:
                    last_valid_fields[key] = dict(value=value, evidence=evidence,
                        source=row['field_sources'][key], unit=row['core_temp_unit'] if key == 'core_temp' else None)
            discontinuity = previous is None
            reasons = []
            if previous:
                before = previous['evidence']
                if evidence['seq'] != before['seq'] + 1:
                    reasons.append('sequence_gap_or_duplicate')
                if evidence['steady_ns'] < before['steady_ns'] or row['pid'] != previous['pid']:
                    reasons.append('session_clock_or_pid_changed')
                if row['barrier'] != previous['barrier']:
                    reasons.append('unreadable_record_between')
                wall_delta = evidence['wall_ns'] - before['wall_ns']
                steady_delta = evidence['steady_ns'] - before['steady_ns']
                if abs(wall_delta - steady_delta) > 1_000_000_000:
                    gaps.append(dict(kind='wall_clock_jump', session=session, before=before, after=evidence,
                                     message='墙钟与单调时间增量相差超过 1 秒；转换排序只使用会话内 seq/steady。'))
            for key in ('dropped_total', 'write_errors_total'):
                current = row['counters'][key]
                prior = previous['counters'][key] if previous else 0
                if current is None:
                    reasons.append(key + '_missing')
                elif prior is None or current > prior or (previous and current < prior):
                    reasons.append(key)
            if reasons:
                gaps.append(dict(kind='evidence_gap', session=session, before=previous['evidence'] if previous else None,
                                 after=evidence, reasons=reasons, counters=row['counters']))
                discontinuity = True
                last_values = {}
                last_evidence = {}
            changed = {}
            for key in ('cur_work_state', 'hms_code', 'time_sync_type', 'powerup_cnt'):
                value = values[key]
                if value is None:
                    status = row['field_status'][key]
                    # Push packets are partial observations. Missing keys are not
                    # invalid measurements; retain the previous observation's own
                    # timestamp, never fill this row with that historical value.
                    partial = ((row['source'] in ('state_push', 'startup', 'shutdown') and status == 'missing' and
                                row['query_result'] in ('success', 'not_applicable')) or
                               (row['source'] == 'query' and row['query_result'] == 'success' and
                                status in ('no_response_length', 'not_available')))
                    if not partial:
                        last_values.pop(key, None)
                        last_evidence.pop(key, None)
                    continue
                compare = tuple(sorted(set(value) - {0})) if key == 'hms_code' else value
                if key not in last_values:
                    changed[key] = dict(before=None, after=list(compare) if key == 'hms_code' else compare,
                                        kind='first_observed' if key not in seen_fields else 'observed_after_gap',
                                        before_evidence=None, after_evidence=evidence)
                    if key == 'cur_work_state' and compare == 4:
                        first_error = True
                    if key == 'hms_code' and any((code & 0xFF) in (3, 4) for code in compare):
                        first_error = True
                elif compare != last_values[key]:
                    old = last_values[key]
                    changed[key] = dict(before=list(old) if key == 'hms_code' else old,
                                        after=list(compare) if key == 'hms_code' else compare, kind='observed_change',
                                        before_evidence=last_evidence[key], after_evidence=evidence)
                last_values[key] = compare
                last_evidence[key] = evidence
                seen_fields.add(key)
            if changed:
                transition = dict(session=session, evidence=evidence, changes=changed,
                                  continuous_records=not discontinuity,
                                  note='两次可见状态的变化；HMS 数组顺序不代表发生顺序，采样间可能有未记录变化。')
                session_transitions.append(transition)
                transitions.append(transition)
            previous = row
        latest = records[-1]
        summaries.append(dict(session=session, pid=latest['pid'], records=len(records),
                              first=first['evidence'], last=latest['evidence'], latest=latest,
                              last_valid_fields=last_valid_fields,
                              error_at_first_visible_or_after_gap=first_error,
                              transition_count=len(session_transitions)))
    issue_counts = Counter(item['kind'] for item in issues)
    return dict(role=role, status='available' if accepted else 'invalid' if files else 'missing',
                evidence_scope='stored_history', budget_bytes=budget, read_bytes=sum(f['read_bytes'] for f in files),
                files=files, records=accepted, sessions=summaries,
                transitions=transitions[-128:], transition_count=len(transitions),
                transitions_truncated=len(transitions) > 128, gaps=gaps[-128:], gap_count=len(gaps),
                issues=issues[-128:], issue_counts=dict(issue_counts),
                missing_field_counts=dict(missing), query_results=dict(query_results),
                latest=summaries[-1]['latest'] if summaries else None,
                limitation='仅为已保存、已读取的原始状态历史；不证明当前状态，不确定故障的物理根因。')


def history_findings(history):
    """Expose concise historical evidence without replacing the current query."""
    if history['status'] == 'missing':
        if history.get('requested'):
            return [dict(code='device_history_unavailable', severity='warning', role=history['role'],
                stage='device_history', message='运行包要求采集原始状态，但该路时间线文件缺失；采集不可用/未落盘，'
                '请核对驱动 stderr 的 writer 禁用/写入失败信息；不能据此认定雷达故障或健康',
                evidence=[dict(path=history['expected_path'], field='device_state_history.requested', time=None)])]
        return []
    findings, evidence = [], []
    sessions = history['sessions']
    if not any(any(key in s['last_valid_fields'] for key in ('cur_work_state', 'hms_code')) for s in sessions):
        findings.append(dict(code='device_history_unavailable', severity='warning', role=history['role'],
            stage='device_history', message='原始状态文件未包含有效 work_state/HMS 观测（可能只有启动/采集错误记录）；'
            '设备状态采集不可用，不能据此认定雷达故障或健康',
            evidence=[s['last'] for s in sessions[-3:]] or history['files']))
    message = f'原始状态历史：读取 {history["records"]} 行、{len(sessions)} 个进程会话'
    if sessions:
        message += f'；可见范围 {sessions[0]["first"]["time"]} 至 {sessions[-1]["last"]["time"]}（按文件和会话 seq 排序，非墙钟排序）'
        evidence += [sessions[0]['first'], sessions[-1]['last']]
        latest = sessions[-1]
        observations = latest['last_valid_fields']
        if 'cur_work_state' in observations:
            observed = observations['cur_work_state']
            state = observed['value']
            message += f'；末会话最后可见 work_state={state} ({WORK_STATES.get(state, "未知")}) @ {observed["evidence"]["time"]}'
        if history['transition_count'] > 3:
            message += f'；可见转换共 {history["transition_count"]} 条，摘要只展示最近 3 条'
        if history['transitions_truncated']:
            message += '（JSON 转换明细保留最近 128 条）'
        for transition in history['transitions'][-3:]:
            changes = []
            for key, change in transition['changes'].items():
                def describe(value):
                    if value is None:
                        return '此前未知'
                    if key == 'hms_code':
                        return '[' + ','.join(f'0x{v:08x}' for v in value) + ']'
                    if key == 'cur_work_state':
                        return f'{value}({WORK_STATES.get(value, "未知")})'
                    return str(value)
                label = {'hms_code': 'HMS', 'cur_work_state': 'work_state',
                         'time_sync_type': '原始授时类型 time_sync_type', 'powerup_cnt': '上电次数 powerup_cnt（不是 uptime）'}[key]
                changes.append(label + ' ' +
                               describe(change['before']) + ' → ' + describe(change['after']))
                if change.get('before_evidence'):
                    evidence.append(change['before_evidence'])
            message += '；' + transition['evidence']['time'] + ' ' + ', '.join(changes)
            evidence.append(transition['evidence'])
        if any(s['error_at_first_visible_or_after_gap'] for s in sessions):
            message += '；首次可见或缺口后已是 Error/错误 HMS，起点之前缺记录，不能把本行时间当故障开始'
        if 'core_temp' in observations:
            temperature = observations['core_temp']
            message += f'；core_temp 原始值={temperature["value"]}，SDK 单位未声明 @ {temperature["evidence"]["time"]}（{temperature["source"]}）'
            evidence.append(temperature['evidence'])
        else:
            message += '；末会话无有效 core_temp 观测'
        ptp = [key for key in ('time_sync_type', 'local_time_now', 'last_sync_time', 'time_offset')
               if key not in observations]
        if ptp:
            message += '；末会话缺有效 PTP 字段: ' + ','.join(ptp)
        for key in ('time_sync_type', 'time_offset'):
            if key in observations:
                observed = observations[key]
                message += f'；原始 {key}={observed["value"]} @ {observed["evidence"]["time"]}'
                evidence.append(observed['evidence'])
        if any(0x04050002 in t['changes'].get('hms_code', {}).get('after', []) for t in history['transitions']):
            message += '；0x04050002 是历史授时警告，不能据此确定 PTP 是故障根因'
    failed_queries = {k: v for k, v in history['query_results'].items()
                      if k not in ('success', 'not_applicable', 'pending')}
    if failed_queries:
        message += '；状态查询/采集结果: ' + json.dumps(failed_queries, ensure_ascii=False)
    if history['missing_field_counts']:
        message += '；缺字段/不可用状态按字段计数保留在 JSON missing_field_counts'
    message += '；电压/设备 uptime 本 schema 不提供；历史最新值不代表当前状态，温度/授时字段不直接证明物理根因'
    findings.append(dict(code='device_history', severity='info', role=history['role'], stage='device_history',
                         message=message, evidence=evidence))
    if history['issues'] or history['gap_count']:
        issue_text = ','.join(f'{k}={v}' for k, v in history['issue_counts'].items()) or '无文件格式问题'
        gap_kinds = Counter(g['kind'] for g in history['gaps'])
        gap_text = ','.join(f'{k}={v}' for k, v in gap_kinds.items()) or '无已知会话内缺口'
        gaps = history['gaps'][-3:]
        gap_evidence = [dict(g['after'], gap_kind=g['kind'], reasons=g.get('reasons'),
                             range_start=g.get('before'), counters=g.get('counters')) for g in gaps]
        findings.append(dict(code='device_history_incomplete', severity='warning', role=history['role'],
            stage='device_history', message=f'原始状态历史覆盖不完整或时钟跳变：{issue_text}；'
            f'缺口/时钟事件共 {history["gap_count"]} 条，最近最多 128 条中 {gap_text}；'
            f'最多读取 {history["budget_bytes"]} 字节，实际 {history["read_bytes"]} 字节；缺口前后不能确定所有变化及故障起点',
            evidence=gap_evidence + history['issues'][-3:]))
        findings[-1]['message'] += '；丢弃/写入失败计数在写记录时取值，所列范围是计数变化可见范围，非精确丢失时刻'
    return findings


def lifecycle(run, meta, warnings):
    events = []
    try:
        text = read_text(run / 'events.jsonl')
        for line in text.splitlines():
            try:
                event = json.loads(line)
                if isinstance(event, dict):
                    events.append(event)
            except ValueError:
                warnings.append('events.jsonl 存在未完成/无效行。')
        if (run / 'events.jsonl').stat().st_size > MIB:
            warnings.append('生命周期事件只读取前 1 MiB，后续退出可能未覆盖。')
    except OSError:
        pass
    # Launch's stop marker has no wall-clock prefix. Use original log ordering
    # only when both the signal and matching PID exit survive the bounded scan.
    stopped, positions = False, {}
    for log in [run / f'logs/driver.log.{i}' for i in range(5, 0, -1)] + [run / 'logs/driver.log']:
        try:
            with log.open('rb') as stream:
                size = log.stat().st_size
                stream.seek(max(0, size - MIB))
                if size > MIB:
                    stream.readline()
                    warnings.append(f'退出阶段只检查 {log.name} 末尾 1 MiB。')
                while stream.tell() < size:
                    raw = stream.readline(min(65536, size - stream.tell()))
                    line = raw.decode(errors='replace')
                    if ('user interrupted with ctrl-c' in line or 'signal_handler(signum=2)' in line or
                            "sending signal 'SIGINT'" in line):
                        stopped = True
                    match = re.search(r'process has died.*?pid[ :]+(\d+).*?exit code[ :]+(-?\d+)', line)
                    if match:
                        positions[(int(match[1]), int(match[2]))] = stopped
        except OSError:
            continue
    result = []
    for event in events:
        if event.get('event') != 'node_exited' or event.get('returncode') in (None, 0):
            continue
        phase = positions.get((event.get('pid'), event.get('returncode')))
        if phase is True:
            stage, message = 'shutdown', '停止信号之后非零退出；属于停止阶段异常，不能作为此前缺数据的根因'
        elif phase is False:
            stage, message = 'lifecycle_unknown', '观察到非零退出，日志中此前未见停止信号；仍需核对人工停止时间'
        else:
            stage, message = 'lifecycle_unknown', '非零退出阶段证据不足，不能据此断言运行期崩溃'
        result.append(dict(code='node_nonzero_exit', severity='warning', role=None, stage=stage,
                           message=f'{event.get("name")} PID {event.get("pid")} exit={event.get("returncode")}: {message}',
                           evidence=[dict(path=str(run / 'events.jsonl'), field='node_exited', time=event.get('time'))]))
    if meta.get('stop_requested'):
        warnings.append('本包记录人工停止；退出码须结合停止阶段解释，不等于采样中途崩溃。')
    return result


def analyze(run, process=None, selection='指定历史路径（离线）', refresh=False, query_fn=query):
    warnings, findings, roles, histories = [], [], [], []
    meta = read_json(run / 'run_metadata.json', warnings)
    if meta.get('owner') != OWNER:
        raise ValueError('运行包缺少受支持的 owner 标记。')
    active = process_alive(process)
    if process and process.get('instance_count', 1) > 1:
        warnings.append(f'发现 {process["instance_count"]} 个本机雷达 worker，选择启动最晚的 PID {process["pid"]}: {run}；其他实例未检查，可显式指定各包路径。')
    expected = meta.get('roles', [])
    if not isinstance(expected, list) or not expected:
        expected = []
        warnings.append('元数据缺少预期 roles，不能判断缺少哪一路。')
    if any(not isinstance(role, str) for role in expected):
        warnings.append('元数据 roles 含无效字段，未对其查询设备。')
        expected = [role for role in expected if isinstance(role, str)]
    startup_path = run / 'checks/startup_check.json'
    startup = read_json(startup_path, warnings)
    result_path = run / 'checks/result.json'
    result = read_json(result_path, warnings)
    sources = [run / 'checks/device_state.json', run / 'checks/failure_state.json']
    extra = sorted(run.glob('recheck_*/checks/*state.json'), reverse=True)
    sources += extra[:32]
    if len(extra) > 32:
        warnings.append('仅检查最近命名的 32 份复查设备状态。')
    states = [(path, read_json(path, warnings)) for path in sources]

    def add(code, severity, role, stage, message, path, field, when=None):
        findings.append(dict(code=code, severity=severity, role=role, stage=stage, message=message,
                             evidence=[dict(path=str(path), field=field, time=when)]))

    for role in dict.fromkeys(expected):
        if role not in IPS:
            warnings.append('无法识别预期雷达角色: ' + str(role))
            continue
        streams = {}
        for kind, label in (('lidar', 'cloud'), ('imu', 'imu')):
            topic = '/livox/' + kind + '_' + IPS[role].replace('.', '_')
            row = startup.get('topics', {}).get(topic)
            streams[label] = dict(row, topic=topic) if isinstance(row, dict) else dict(topic=topic)
            if not isinstance(row, dict):
                add('startup_evidence_missing', 'warning', role, 'startup', f'缺少启动窗口 {label} 接收证据', startup_path, topic)
            elif row.get('count') == 0:
                add('startup_stream_missing', 'error', role, 'startup', f'启动窗口 {label} 接收 0 条', startup_path, topic)
            elif row.get('passed') is not True:
                add('startup_stream_failed', 'warning', role, 'startup', f'启动窗口 {label} 检查未通过，见时间戳/空云/新鲜度字段', startup_path, topic)
        streams['source'] = str(startup_path)
        candidates = [(timestamp(state.get(role, {}).get('collected_at')) or 0, path, state[role])
                      for path, state in states if isinstance(state.get(role), dict)]
        raw, source = {}, str(run / 'checks/device_state.json')
        if candidates:
            _, source, raw = max(candidates, key=lambda item: item[0])
        live = False
        if active and refresh and process_alive(process):
            try:
                raw = query_fn(IPS[role])
            except (OSError, ValueError) as error:
                raw = dict(parameters={}, error=str(error))
            source, live = 'live:livox_read_only_0x0101/' + IPS[role], True
        device = device_record(raw, str(source), live)
        scope = '本次只读查询' if live else '历史设备快照'
        when = device['collected_at']
        if device['errors']:
            add('device_evidence_incomplete', 'warning', role, 'device', scope + '缺少有效参数: ' +
                '; '.join(k + ': ' + str(v) for k, v in device['errors'].items()), source, 'parameters', when)
        state = device['work_state']
        if state is not None and state != 1:
            add('device_not_normal', 'error', role, 'device', f'{scope} work_state={state} ({device["work_state_name"]})', source, 'parameters.work_state', when)
        for alarm in device['hms'] or []:
            add('native_hms', 'error' if (alarm['level'] or 0) >= 3 else 'info' if alarm['historical'] else 'warning',
                role, 'device', f'{scope} {alarm["code"]}: {alarm["label"]}，等级={alarm.get("level_name", "Unknown")}',
                source, 'parameters.hms', when)
        roles.append(dict(role=role, ip=IPS[role], startup=streams, device=device))
        history = device_history(run, role)
        history['requested'] = (isinstance(meta.get('device_state_history'), dict) and
                                meta['device_state_history'].get('requested') is True)
        history['expected_path'] = str(run / 'diagnostics' / (role + '_state.jsonl'))
        histories.append(history)
        findings += history_findings(history)
    if result.get('identity_ok') is False:
        identity_path = run / 'checks/identity.json'
        identity = read_json(identity_path, warnings)
        devices = identity.get('devices', [])
        failed = [item for item in devices if isinstance(item, dict) and
                  item.get('role') in expected and item.get('identity_valid') is False] if isinstance(devices, list) else []
        when = identity.get('collected_at_utc')
        when = when if timestamp(when) is not None else None
        for item in failed or [None]:
            detail = '; status=' + str(item.get('status', '未知')) if item else '；缺少可归属到单路的身份失败明细'
            add('startup_identity_failed', 'error', item.get('role') if item else None, 'identity',
                '该次身份检查未通过' + detail + ('；检查时间未记录' if when is None else ''),
                result_path, 'identity_ok', when)
            if item:
                findings[-1]['evidence'].append(dict(path=str(identity_path),
                    field='devices.' + item['role'] + '.identity_valid', time=when))
    for key, stage, label in (('data_ok', 'startup', '数据'), ('device_state_ok', 'device', '设备状态')):
        if result.get(key) is False and not any(f['stage'] == stage and f['severity'] == 'error' for f in findings):
            add('startup_check_failed', 'error', None, stage,
                f'该次{label}总检查未通过；缺少已确认的具体失败项，不能据此猜测硬件原因；检查时间未记录',
                result_path, key)
    lifecycle_findings = lifecycle(run, meta, warnings)
    findings += lifecycle_findings
    findings.sort(key=lambda item: {'error': 0, 'warning': 1, 'info': 2}[item['severity']])
    if active and not refresh:
        warnings.append('当前驱动存在，但未刷新设备状态；已有包证据不能证明当前仍在出数。')
    if process and not process_alive(process):
        active = False
        warnings.append('选择的雷达 worker 已结束或身份变化；本次结果不得作为当前实例完整验收。')
    warnings.append('启动计数/频率仅描述启动检查窗口；topic 存在、SN 可读、工作模式设置应答成功都不证明持续出数。')
    return dict(schema=1, kind='livox_driver_diagnosis', selected=str(run), selection=selection,
                active=active, process=process, metadata={k: meta.get(k) for k in (
                    'roles', 'hostname', 'role', 'started_at', 'ended_at', 'status', 'ros_environment',
                    'driver_build_revision', 'startup_check', 'stop_requested', 'forced_kill')},
                roles=roles, device_history=histories, findings=findings, warnings=warnings,
                has_evidence=bool(startup or result or any(state for _, state in states) or lifecycle_findings or
                                  any(r['device']['live'] for r in roles) or any(h['records'] for h in histories)))


def render(report, color=False):
    def paint(value, style):
        return f'\033[{style}m{value}\033[0m' if color else value
    lines = [paint('雷达现场诊断（仅读取证据）', '1;36'), '选择: ' + report['selection'],
             '数据包: ' + report['selected'], '驱动进程: ' + ('运行中' if report['active'] else '未确认活跃实例'),
             '检查器调度: ' + str(report.get('scheduling', {})),
             '记录的 ROS 环境: ' + json.dumps(report['metadata'].get('ros_environment'), ensure_ascii=False)]
    if report['process']:
        lines.append('选择的 worker PID: ' + str(report['process']['pid']))
    for role in report['roles']:
        lines.append(paint(role['role'] + ' ' + role['ip'], '1;36'))
        for key in ('cloud', 'imu'):
            stream = role['startup'][key]
            lines.append(f'  启动窗口 {key}: count={stream.get("count", "未知")} hz={stream.get("hz", "未知")} passed={stream.get("passed", "未知")}')
        device = role['device']
        lines.append(f'  {"本次设备查询" if device["live"] else "历史设备快照"}: SN={device["serial_number"]} work_state={device["work_state"]} ({device["work_state_name"]})')
        lines.append(f'  采集时间: {device["collected_at"]}；证据年龄: {device["age_sec"]} 秒；来源: {device["source"]}')
    missing_history = [h['role'] for h in report.get('device_history', []) if h['status'] == 'missing']
    if missing_history:
        lines.append('原始状态时间线未记录（旧包兼容，不据此认定雷达故障）: ' + ','.join(missing_history))
    if not report['findings']:
        lines.append('已读取证据未列出异常；不等于当前所有雷达持续健康。')
    for finding in report['findings']:
        lines.append(paint(f'[{finding["severity"]}] {finding["role"] or "生命周期"}: {finding["message"]}',
                           '1;31' if finding['severity'] == 'error' else '33' if finding['severity'] == 'warning' else '36'))
        if finding['stage'] == 'device_history':
            seen = set()
            for item in finding['evidence']:
                location = str(item.get('path', '未知文件'))
                if item.get('line') is not None:
                    location += ':' + str(item['line'])
                elif item.get('byte_offset') is not None:
                    location += ':byte=' + str(item['byte_offset']) + '（尾部读取，绝对行号未知）'
                if item.get('time'):
                    location += ' @ ' + item['time']
                if item.get('reasons'):
                    location += '；缺口=' + ','.join(item['reasons'])
                if item.get('range_start'):
                    location += '；前条记录=' + item['range_start']['time']
                if item.get('kind'):
                    location += '；' + item['kind'] + ': ' + item.get('message', '')
                if location not in seen:
                    lines.append('  证据: ' + location)
                    seen.add(location)
        else:
            lines.append('  证据: ' + json.dumps(finding['evidence'], ensure_ascii=False))
    lines += [paint('注意: ' + warning, '33') for warning in report['warnings']]
    return '\n'.join(lines)


def main(argv=None):
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path', nargs='?', help='指定雷达运行包；始终离线，不查询设备')
    parser.add_argument('--root', help='无驱动进程时的包根目录')
    parser.add_argument('--json', action='store_true')
    parser.add_argument('--color', choices=('auto', 'always', 'never'), default='auto')
    parser.add_argument('--no-refresh', action='store_true', help='仅读包，不查询活跃实例的设备状态')
    args = parser.parse_args(argv)
    try:
        scheduling = low_impact()
        run, process, selection = select(args.path, args.root)
        report = analyze(run, process, selection, refresh=not args.no_refresh and not args.path)
        report['scheduling'] = scheduling
        color = args.color == 'always' or (args.color == 'auto' and sys.stdout.isatty() and
                    'NO_COLOR' not in os.environ and os.environ.get('TERM') != 'dumb')
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) if args.json else render(report, color))
        return 0 if report['has_evidence'] else 2
    except (OSError, ValueError, RuntimeError) as error:
        print(json.dumps(dict(schema=1, kind='livox_driver_diagnosis', error=str(error), has_evidence=False), ensure_ascii=False)
              if args.json else '无法完成雷达诊断: ' + str(error))
        return 2


if __name__ == '__main__':
    sys.exit(main())
