"""Source agreement is a loading gate, never a vehicle-safety certificate."""
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
import pytest

from f1tenth_benchmarks.research.core.geometry.build_map import json_bytes
from f1tenth_benchmarks.research.overtaking.track_data import AuditLimits, TrackAuditError, TrackData


SOURCE = Path('maps/frenet/esp/geometry')


@pytest.fixture(scope='module')
def track():
    return TrackData(SOURCE, 'maps/esp.yaml')


def test_shared_bundle_and_closed_control_path(track):
    assert track.audit['passed']
    assert track.audit['rejected_boundary_rows'] == []
    assert track.audit['interval_probe_count'] == len(track.centerline)
    assert track.audit['invalid_midpoint_intervals'] == []
    np.testing.assert_array_equal(track.control_path[-1], track.control_path[0])
    np.testing.assert_array_equal(track.control_path[:-1], track.centerline[:, :2])
    assert not track.control_path.flags.writeable
    for flag in ('planning_allowed', 'footprint_verified', 'domain_verified', 'continuous_verified'):
        assert track.audit[flag] is False
    assert track.provenance['geometry_id'] == track.geometry.geometry_id
    assert track.provenance['boundary_method'] == 'exact_grid_ray'
    json.dumps(track.audit, allow_nan=False)


def test_unrecorded_file_change_rejected_before_use(tmp_path):
    root = tmp_path/'geometry'
    shutil.copytree(SOURCE, root)
    with (root/'boundaries.csv').open('a') as stream:
        stream.write('\n')
    with pytest.raises(ValueError, match='geometry_mismatch: boundaries.csv'):
        TrackData(root, 'maps/esp.yaml')


def test_resigned_incorrect_boundary_values_rejected(tmp_path):
    # Hash agreement alone cannot establish that the exported boundary geometry
    # agrees with the original map. Re-sign a coherent but incorrect export.
    root = tmp_path/'geometry'
    shutil.copytree(SOURCE, root)
    path = root/'boundaries.csv'
    rows = path.read_text().splitlines()
    fields = rows[1].split(',')
    fields[3] = str(float(fields[3])+.1)
    rows[1] = ','.join(fields)
    path.write_text('\n'.join(rows)+'\n')
    manifest = json.loads((root/'manifest.json').read_text())
    manifest.pop('geometry_id')
    manifest['file_hashes']['boundaries.csv'] = hashlib.sha256(path.read_bytes()).hexdigest()
    manifest['geometry_id'] = hashlib.sha256(json_bytes(manifest)).hexdigest()
    (root/'manifest.json').write_bytes(json_bytes(manifest))
    with pytest.raises(TrackAuditError) as caught:
        TrackData(root, 'maps/esp.yaml')
    report = caught.value.report
    assert not report['passed']
    assert report['max_errors']['boundary_m'] == pytest.approx(.1)
    assert report['rejected_boundary_rows'][0]['reason'] == 'boundary_value_mismatch'


@pytest.mark.parametrize('value', [0., -1., float('nan'), float('inf')])
def test_invalid_audit_threshold(value):
    with pytest.raises(ValueError, match='finite and positive'):
        AuditLimits(boundary_m=value)
