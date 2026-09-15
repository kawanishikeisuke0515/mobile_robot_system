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
