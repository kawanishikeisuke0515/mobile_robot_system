"""ROS-independent CSV schema, snapshots and bounded asynchronous storage."""
import csv
import json
import math
from pathlib import Path
import queue
import threading
import time
from datetime import datetime, timezone
from uuid import uuid4

import yaml

COMMON = ['sample_seq', 'recv_ros_time_ns', 'elapsed_sec', 'source_stamp_ns', 'frame_id']
TWIST = [f'{part}_{axis}' for part in ('linear', 'angular') for axis in 'xyz']
CONTROL = ['session_id', 'active', 'inputs_valid', 'target_reached', 'tracking_valid',
           'docking_complete', 'target_marker_id', 'detection_sequence']
FIELDS = {
    'cmd_vel': TWIST,
    'motor_commands': ['motor_0', 'motor_1', 'motor_2', 'motor_3', 'element_count',
                       'shape_valid', 'data_json'],
    'deadman': ['data'],
    'uwb': ['x_m', 'y_m', 'valid', 'device_time_ms'],
    'zed_heading': ['raw_x', 'raw_z', 'corrected_x', 'corrected_z',
                    'magnetic_heading_deg', 'robot_yaw_deg', 'robot_yaw_rad', 'valid'],
    'optitrack': ['position_x', 'position_y', 'position_z', 'qx', 'qy', 'qz', 'qw'],
    'vision': ['id', 'x', 'y', 'z', 'distance', 'theta', 'yaw', 'center_u', 'center_v',
               'normalized_center_error'],
    'control_state': ['data'],
    'uwb_control_output': CONTROL + ['command_' + f for f in TWIST],
    'vision_control_output': CONTROL + ['command_' + f for f in TWIST],
}
FRESHNESS = ['received', 'age_sec', 'stale', 'value_valid']
SCHEMAS = {key: COMMON + fields for key, fields in FIELDS.items()}
SCHEMAS['timeline'] = ['row_seq', 'row_ros_time_ns', 'elapsed_sec'] + [
    f'{key}_{field}' for key, fields in FIELDS.items() for field in COMMON + fields + FRESHNESS]


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def twist_values(msg, prefix=''):
    return {prefix + f: getattr(getattr(msg, f.split('_')[0]), f[-1]) for f in TWIST}


def decode(key, msg):
    """Flatten a message without modifying or substituting its source values."""
    header = getattr(msg, 'header', None)
    stamp = header.stamp if header is not None else getattr(msg, 'stamp', None)
    row = {'source_stamp_ns': stamp_ns(stamp) if stamp is not None else '',
           'frame_id': header.frame_id if header is not None else ''}
    if key == 'cmd_vel':
        row.update(twist_values(msg))
    elif key == 'motor_commands':
        values = list(msg.data)
        row.update({f'motor_{i}': values[i] if i < len(values) else '' for i in range(4)})
        # JSON strings preserve non-finite values while keeping valid JSON syntax.
        row.update(element_count=len(values), shape_valid=len(values) == 4,
                   data_json=json.dumps([v if math.isfinite(v) else str(v) for v in values]))
    elif key == 'optitrack':
        row.update({f'position_{a}': getattr(msg.pose.position, a) for a in 'xyz'})
        row.update({f'q{a}': getattr(msg.pose.orientation, a) for a in 'xyzw'})
    elif key.endswith('_control_output'):
        row.update({f: getattr(msg, f) for f in CONTROL})
        row.update(twist_values(msg.command, 'command_'))
    else:
        row.update({f: getattr(msg, f) for f in FIELDS[key]})
    return row


def valid_value(key, row):
    fields = {
        'cmd_vel': TWIST, 'motor_commands': [f'motor_{i}' for i in range(4)],
        'uwb': ['x_m', 'y_m'], 'zed_heading': ['robot_yaw_rad', 'robot_yaw_deg'],
        'optitrack': FIELDS['optitrack'], 'vision': ['x', 'y', 'z', 'distance', 'theta', 'yaw'],
    }.get(key)
    if fields is None:
        return ''
    if key == 'motor_commands' and not row['shape_valid']:
        return False
    finite = all(isinstance(row[f], (int, float)) and math.isfinite(row[f]) for f in fields)
    if key in ('uwb', 'zed_heading'):
        finite = finite and row['valid']
    if key == 'optitrack':
        finite = finite and any(row['q' + a] != 0 for a in 'xyzw')
    return bool(finite)


class Samples:
    """Lock-protected latest samples; unrelated marker IDs do not refresh the target."""

    def __init__(self, target_marker_id, thresholds):
        self.target = target_marker_id
        self.thresholds = thresholds
        self.latest = {}
        self.sequences = {k: 0 for k in FIELDS}
        self.row_seq = 0
        self.lock = threading.Lock()

    def receive(self, key, values, ros_ns, elapsed):
        with self.lock:
            self.sequences[key] += 1
            row = dict(values, sample_seq=self.sequences[key], recv_ros_time_ns=ros_ns,
                       elapsed_sec=elapsed)
            if key != 'vision' or row['id'] == self.target:
                self.latest[key] = row
            return row

    def snapshot(self, ros_ns, elapsed):
        with self.lock:
            self.row_seq += 1
            result = dict(row_seq=self.row_seq, row_ros_time_ns=ros_ns, elapsed_sec=elapsed)
            for key in FIELDS:
                row = self.latest.get(key)
                result[f'{key}_received'] = row is not None
                if row is None:
                    continue
                result.update({f'{key}_{f}': v for f, v in row.items()})
                age = max(0.0, elapsed - row['elapsed_sec'])
                result.update({f'{key}_age_sec': age,
                               f'{key}_stale': age > self.thresholds[key],
                               f'{key}_value_valid': valid_value(key, row)})
            return result


class CsvWriter:
    """One disk writer, bounded queue, explicit failure and shutdown accounting."""

    def __init__(self, log_dir, name, metadata, capacity=10000, flush_interval=1.0):
        if not name.strip() or '/' in name or '\\' in name or '..' in name:
            raise ValueError('experiment_name must be a nonempty filename component')
        self.record_id = uuid4().hex
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.path = Path(log_dir).expanduser().resolve() / f'{timestamp}_{name}_{self.record_id[:8]}'
        self.path.mkdir(parents=True, exist_ok=False)
        self.metadata = dict(metadata, schema_version=1, record_id=self.record_id,
                             started_at=utc_now(), status='recording', output_dir=str(self.path))
        self.queue = queue.Queue(maxsize=capacity)
        self.flush_interval = flush_interval
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.closed = False
        self.error = None
        self.dropped = {k: 0 for k in SCHEMAS}
        self.counts = {k: 0 for k in SCHEMAS}
        self.files = {}
        self.writers = {}
        try:
            for key, fields in SCHEMAS.items():
                file = (self.path / f'{key}.csv').open('x', encoding='utf-8', newline='')
                self.files[key] = file
                self.writers[key] = csv.DictWriter(file, fieldnames=fields)
                self.writers[key].writeheader()
                file.flush()
            self.save_metadata()
        except BaseException:
            for file in self.files.values():
                file.close()
            raise
        self.thread = threading.Thread(target=self.run, name='docking-csv-writer')
        self.thread.start()

    def save_metadata(self):
        temporary = self.path / 'metadata.yaml.tmp'
        temporary.write_text(yaml.safe_dump(self.metadata, allow_unicode=True, sort_keys=False),
                             encoding='utf-8')
        temporary.replace(self.path / 'metadata.yaml')

    def submit(self, key, row):
        with self.lock:
            if self.closed or self.error:
                return False
            try:
                self.queue.put_nowait((key, row))
                return True
            except queue.Full:
                self.dropped[key] += 1
                return False

    def run(self):
        next_flush = time.monotonic() + self.flush_interval
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    key, row = self.queue.get(timeout=min(0.1, self.flush_interval))
                except queue.Empty:
                    pass
                else:
                    self.writers[key].writerow({
                        k: str(v).lower() if isinstance(v, bool) else v for k, v in row.items()})
                    self.counts[key] += 1
                if time.monotonic() >= next_flush:
                    for file in self.files.values():
                        file.flush()
                    next_flush = time.monotonic() + self.flush_interval
        except Exception as exc:
            self.error = repr(exc)
        finally:
            for file in self.files.values():
                try:
                    file.close()
                except Exception as exc:
                    self.error = self.error or repr(exc)

    def close(self, failure=None):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self.stop.set()
        self.thread.join()
        self.error = self.error or failure
        self.metadata.update(ended_at=utc_now(), status='failed' if self.error else 'completed',
                             saved_rows=self.counts, queue_dropped=self.dropped,
                             pending_rows=self.queue.qsize(), error=self.error)
        self.save_metadata()
