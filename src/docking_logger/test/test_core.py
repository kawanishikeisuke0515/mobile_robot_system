"""Data semantics and writer failure/shutdown tests without ROS hardware."""
import csv
import json
from types import SimpleNamespace as NS
import threading

import pytest
import yaml

from docking_logger.core import CsvWriter, FIELDS, Samples, decode, valid_value


def samples():
    return Samples(7, {key: 1.0 for key in FIELDS})


def test_missing_stale_and_marker_selection():
    state = samples()
    assert state.snapshot(0, 0)['uwb_received'] is False
    row = dict(id=7, x=1., y=2., z=3., distance=4., theta=0., yaw=0.)
    state.receive('vision', row, 100, 0.1)
    state.receive('vision', dict(row, id=8, x=99.), 200, 0.9)
    result = state.snapshot(300, 1.2)
    assert result['vision_x'] == 1.
    assert result['vision_sample_seq'] == 1
    assert result['vision_stale'] is True
    assert result['vision_value_valid'] is True
    assert result['vision_age_sec'] == pytest.approx(1.1)


def test_invalid_motor_and_source_stamp():
    row = decode('motor_commands', NS(data=[1., float('nan')]))
    assert row['source_stamp_ns'] == ''
    assert row['motor_2'] == ''
    assert not valid_value('motor_commands', row)
    assert json.loads(row['data_json']) == [1., 'nan']
    row = decode('uwb', NS(header=NS(stamp=NS(sec=0, nanosec=0), frame_id='uwb'),
                           x_m=2., y_m=3., valid=False, device_time_ms=42))
    assert row['source_stamp_ns'] == 0
    assert row['frame_id'] == 'uwb'
    assert not valid_value('uwb', row)


def test_writer_drain_metadata_and_unique_directory(tmp_path):
    writer = CsvWriter(tmp_path, 'trial', {'experiment_note': '日本語'}, flush_interval=.01)
    for i in range(50):
        assert writer.submit('deadman', {'sample_seq': i, 'data': i % 2 == 0})
    writer.close()
    with (writer.path / 'deadman.csv').open() as file:
        rows = list(csv.DictReader(file))
    assert len(rows) == 50
    assert rows[0]['data'] == 'true'
    metadata = yaml.safe_load((writer.path / 'metadata.yaml').read_text())
    assert metadata['status'] == 'completed'
    assert metadata['saved_rows']['deadman'] == 50
    assert metadata['pending_rows'] == 0
    other = CsvWriter(tmp_path, 'trial', {})
    other.close()
    assert writer.path != other.path


def test_queue_overflow_is_counted(tmp_path, monkeypatch):
    gate = threading.Event()
    original = CsvWriter.run
    monkeypatch.setattr(CsvWriter, 'run', lambda self: (gate.wait(), original(self)))
    writer = CsvWriter(tmp_path, 'trial', {}, capacity=1)
    try:
        assert writer.submit('deadman', {'data': True})
        assert not writer.submit('deadman', {'data': False})
        assert writer.dropped['deadman'] == 1
    finally:
        gate.set()
        writer.close()
    assert writer.counts['deadman'] == 1


def test_disk_error_is_not_completed(tmp_path):
    writer = CsvWriter(tmp_path, 'trial', {})
    class Broken:
        def writerow(self, row):
            raise OSError('disk full')
    writer.writers['deadman'] = Broken()
    writer.submit('deadman', {'data': True})
    writer.close()
    metadata = yaml.safe_load((writer.path / 'metadata.yaml').read_text())
    assert metadata['status'] == 'failed'
    assert 'disk full' in metadata['error']


@pytest.mark.parametrize('name', ['', ' ', '..', '../trial', 'a/b', 'a\\b'])
def test_bad_names(tmp_path, name):
    with pytest.raises(ValueError):
        CsvWriter(tmp_path, name, {})


def test_control_error_csv_timeline_and_invalid_input(tmp_path):
    state = samples()
    fields = dict(session_id='trial', active=False, inputs_valid=True,
                  raw_error_world_x=3., raw_error_world_y=4., error_world_x=3.,
                  error_world_y=4., error_body_x=4., error_body_y=3.,
                  distance_error_m=5., yaw_error=0.)
    msg = NS(header=NS(stamp=NS(sec=12, nanosec=34), frame_id=''), **fields)
    row = state.receive('uwb_control_error', decode('uwb_control_error', msg), 100, .1)
    snapshot = state.snapshot(200, .2)
    assert snapshot['uwb_control_error_error_body_x'] == 4.
    assert snapshot['uwb_control_error_value_valid'] is True
    assert snapshot['uwb_control_error_active'] is False
    writer = CsvWriter(tmp_path, 'errors', {})
    writer.submit('uwb_control_error', row)
    writer.submit('timeline', snapshot)
    writer.close()
    with (writer.path / 'uwb_control_error.csv').open() as file:
        saved = next(csv.DictReader(file))
    assert saved['distance_error_m'] == '5.0'
    assert saved['source_stamp_ns'] == '12000000034'
    msg.inputs_valid = False
    for field in FIELDS['uwb_control_error'][3:]:
        setattr(msg, field, float('nan'))
    state.receive('uwb_control_error', decode('uwb_control_error', msg), 300, .3)
    assert state.snapshot(400, .4)['uwb_control_error_value_valid'] is False
    assert state.snapshot(500, 2.)['uwb_control_error_stale'] is True


@pytest.mark.parametrize('stream', ['uwb_robot_pose', 'optitrack'])
def test_pose_csv_timeline_and_validity(tmp_path, stream):
    state = samples()
    assert state.snapshot(0, 0)[stream + '_received'] is False
    msg = NS(header=NS(stamp=NS(sec=12, nanosec=34), frame_id='world'),
             pose=NS(position=NS(x=1.25, y=-2., z=0.),
                     orientation=NS(x=0., y=0., z=0.6, w=0.8)))
    row = state.receive(stream, decode(stream, msg), 100, .1)
    snapshot = state.snapshot(200, .2)
    assert snapshot[stream + '_value_valid'] is True
    assert snapshot[stream + '_stale'] is False
    assert state.snapshot(300, 1.2)[stream + '_stale'] is True
    writer = CsvWriter(tmp_path, 'poses', {})
    writer.submit(stream, row)
    writer.submit('timeline', snapshot)
    writer.close()
    with (writer.path / (stream + '.csv')).open() as file:
        saved = next(csv.DictReader(file))
    with (writer.path / 'timeline.csv').open() as file:
        timeline = next(csv.DictReader(file))
    for field in FIELDS[stream] + ['source_stamp_ns', 'frame_id']:
        assert saved[field] == str(row[field])
        assert timeline[stream + '_' + field] == saved[field]
    metadata = yaml.safe_load((writer.path / 'metadata.yaml').read_text())
    assert metadata['schema_version'] == 5
    msg.pose.position.x = float('nan')
    assert not valid_value(stream, decode(stream, msg))
    msg.pose.position.x = 1.
    msg.pose.orientation.w = float('inf')
    assert not valid_value(stream, decode(stream, msg))
    msg.pose.orientation.w = msg.pose.orientation.z = 0.
    assert not valid_value(stream, decode(stream, msg))


def test_power_csv_and_invalid_notifications(tmp_path):
    state = samples()
    assert not state.snapshot(0, 0)['jetson_power_received']
    msg = NS(header=NS(stamp=NS(sec=10, nanosec=20), frame_id=''),
             total_power_rail='VDD_IN', power_w=5., average_power_w=4.5,
             valid=True, status='ok', raw_line='VDD_IN 5000/4500')
    row = state.receive('jetson_power', decode('jetson_power', msg), 100, .1)
    writer = CsvWriter(tmp_path, 'power', {})
    writer.submit('jetson_power', row)
    writer.submit('timeline', state.snapshot(200, .2))
    writer.close()
    with (writer.path / 'jetson_power.csv').open() as f:
        saved = next(csv.DictReader(f))
    assert saved['power_w'] == '5.0'
    assert saved['source_stamp_ns'] == '10000000020'
    with (writer.path / 'timeline.csv').open() as f:
        assert next(csv.DictReader(f))['jetson_power_value_valid'] == 'true'
    msg.valid, msg.status, msg.raw_line = False, 'timeout', ''
    msg.power_w = msg.average_power_w = float('nan')
    state.receive('jetson_power', decode('jetson_power', msg), 300, .3)
    snap = state.snapshot(400, .4)
    assert not snap['jetson_power_value_valid'] and not snap['jetson_power_stale']
    assert state.snapshot(500, 2.)['jetson_power_stale']


def test_zed_odom_csv_timeline_and_validity(tmp_path):
    state = samples()
    assert not state.snapshot(0, 0)['zed_odom_received']
    msg = NS(header=NS(stamp=NS(sec=12, nanosec=34), frame_id='odom'),
             child_frame_id='camera_link',
             pose=NS(pose=NS(position=NS(x=1., y=2., z=0.),
                             orientation=NS(x=0., y=0., z=0., w=1.)),
                     covariance=[0.1] * 36),
             twist=NS(twist=NS(linear=NS(x=.3, y=-.2, z=0.),
                               angular=NS(x=0., y=0., z=.05)),
                      covariance=[0.2] * 36))
    row = state.receive('zed_odom', decode('zed_odom', msg), 100, .1)
    snap = state.snapshot(200, .2)
    assert snap['zed_odom_value_valid']
    assert state.snapshot(300, 1.2)['zed_odom_stale']
    writer = CsvWriter(tmp_path, 'odom', {})
    writer.submit('zed_odom', row)
    writer.submit('timeline', snap)
    writer.close()
    assert writer.error is None
    with (writer.path / 'zed_odom.csv').open() as f:
        saved = next(csv.DictReader(f))
    with (writer.path / 'timeline.csv').open() as f:
        timeline = next(csv.DictReader(f))
    assert saved['linear_x'] == '0.3'
    assert saved['angular_z'] == '0.05'
    assert saved['source_stamp_ns'] == '12000000034'
    assert saved['frame_id'] == 'odom'
    assert saved['child_frame_id'] == 'camera_link'
    assert json.loads(saved['twist_covariance_json']) == [0.2] * 36
    for field in FIELDS['zed_odom']:
        assert timeline['zed_odom_' + field] == saved[field]
    msg.twist.twist.linear.x = float('nan')
    assert not valid_value('zed_odom', decode('zed_odom', msg))
    msg.twist.twist.linear.x = 0.
    msg.pose.pose.orientation.w = 0.
    assert not valid_value('zed_odom', decode('zed_odom', msg))
