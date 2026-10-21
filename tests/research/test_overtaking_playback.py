"""Recorded-only lookup, original-map alignment and loopback HTTP behavior."""
import copy
import hashlib
import json
from pathlib import Path
import threading
from urllib.error import HTTPError
from urllib.request import urlopen

import pytest

from f1tenth_benchmarks.research.overtaking.playback import PlaybackSession, make_server
from f1tenth_benchmarks.research.overtaking.track_data import TrackData


@pytest.fixture(scope='module')
def track():
    return TrackData('maps/frenet/esp/geometry', 'maps/esp.yaml')


@pytest.fixture
def session_root(tmp_path, track):
    root = tmp_path/'recorded'
    root.mkdir()
    metadata = dict(schema_version=1, geometry_id=track.geometry.geometry_id,
        frame_id='map', track=track.provenance, playback_origin_s=7.3,
        simulator_params=dict(vehicle_length=.58, vehicle_width=.31),
        config=dict(lookahead_m=40., plan_period_s=.1, control_dt_s=.1))
    (root/'metadata.json').write_text(json.dumps(metadata))
    states = []
    for i in range(6):
        ref = track.geometry.reference(float(i))
        states.append(dict(control_step=i, sim_time_s=7.3+i*.1, valid=True, reason=None,
            pose=[*ref['xy'], ref['psi']], speed_mps=1.2,
            frenet=dict(valid=True, s_wrapped=float(i), s_unwrapped=float(i), d=0., e_psi=0.)))
    plans = []
    for plan_id, step, available in [(0, 1, 1), (1, 3, 3), (2, 4, 5)]:
        rejected = plan_id == 1
        x, y, yaw = states[step]['pose']
        plans.append(dict(schema_version=1, plan_id=plan_id, source_control_step=step,
            source_sim_time_s=states[step]['sim_time_s'], available_control_step=available,
            available_sim_time_s=states[available]['sim_time_s'], source_pose=[x,y,yaw],
            geometry_id=track.geometry.geometry_id, status='rejected' if rejected else 'completed',
            reason='injected failure' if rejected else None, wall_time_s=.4,
            paths=[dict(candidate_id=i, status='rejected' if rejected else 'sampled_valid',
                sampled_valid=not rejected, reason='optimizer_constraints_not_satisfied' if rejected else None,
                points=[] if rejected else [dict(x=x, y=y, yaw=yaw), dict(x=x+.1, y=y, yaw=yaw)])
                for i in range(1, 6)]))
    for name, rows in [('states', states), ('plans', plans)]:
        (root/(name+'.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in rows))
    (root/'summary.json').write_text(json.dumps(dict(schema_version=1, state_count=6,
        plan_count=3, end_reason='requested_laps_reached', completed_full_laps=1)))
    return root


def test_reverse_seek_has_no_future_or_stale_success(session_root, track):
    reader = PlaybackSession(session_root, track)
    assert reader.frame(0)['plan'] is None
    assert reader.frame(5)['plan']['plan_id'] == 2
    assert reader.frame(4)['plan']['plan_id'] == 1  # unavailable future result stays hidden
    assert reader.frame(3)['plan']['status'] == 'rejected'
    assert reader.frame(3)['plan']['sampled_valid_count'] == 0
    assert reader.frame(2)['plan']['plan_id'] == 0  # backward lookup replaces the failure
    assert reader.frame(0)['plan'] is None
    assert len(reader.frame(2)['trail']) == 3
    assert len(reader.frame(5)['trail']) == 6
    assert reader.frame(2)['plan']['paths'][0]['pixels'] == reader.frame(1)['plan']['paths'][0]['pixels']
    assert reader.frame(2)['candidate_age_s'] == pytest.approx(.1)


def test_absolute_origin_and_floor_time_selection(session_root, track):
    reader = PlaybackSession(session_root, track)
    assert reader.origin_s == 7.3
    assert reader.seek_time(0) == 0
    assert reader.seek_time(.199) == 1
    assert reader.seek_time(.2) == 2
    assert reader.seek_time(reader.duration_s) == 5
    for value in (-1., 100., float('nan'), float('inf')):
        with pytest.raises(ValueError):
            reader.seek_time(value)
    for step in (-1, 6, True, 2.5):
        with pytest.raises(ValueError):
            reader.frame(step)


def test_map_and_body_reuse_inspector_coordinate_transform(session_root, track):
    reader = PlaybackSession(session_root, track)
    frame = reader.frame(1)
    assert frame['pixel'] == reader.inspector.pixels(frame['pose'][:2])
    assert len(frame['body']) == 4
    assert frame['plan']['source_body'] == frame['body']
    data = reader.map_data()
    assert data['center'][0] == data['center'][-1]
    assert len(data['left']) == len(track.boundary_samples)
    assert 'attempts' not in json.dumps(frame)  # dense solver diagnostics are not sent to the browser


def test_missing_or_damaged_index_rebuilt_without_source_changes(session_root, track):
    (session_root/'playback_index.json').write_text('{broken')
    before = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in session_root.iterdir()}
    reader = PlaybackSession(session_root, track)
    assert reader.index_status == 'rebuilt'
    assert reader.frame(5)['at_end']
    after = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in session_root.iterdir()}
    assert before == after


@pytest.mark.parametrize('field', ['geometry_id', 'frame_id', 'track_hash'])
def test_source_mismatch_is_refused(session_root, track, field):
    path = session_root/'metadata.json'
    data = json.loads(path.read_text())
    if field == 'track_hash':
        data['track']['file_hashes']['centerline.csv'] = 'wrong'
    else:
        data[field] = 'wrong'
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='source mismatch'):
        PlaybackSession(session_root, track)


@pytest.mark.parametrize('reason', ['requested_laps_reached', 'collision', 'timeout', 'error'])
def test_end_reason_preserved_without_looping(session_root, track, reason):
    path = session_root/'summary.json'
    data = json.loads(path.read_text()); data['end_reason'] = reason; path.write_text(json.dumps(data))
    reader = PlaybackSession(session_root, track)
    assert reader.frame(5)['at_end'] and reader.frame(5)['end_reason'] == reason
    assert not reader.frame(4)['at_end'] and reader.frame(4)['end_reason'] is None


def test_http_routes_and_strict_range_checks(session_root, track):
    reader = PlaybackSession(session_root, track)
    (session_root/'progress.json').write_text(json.dumps(dict(status='running', plan_count=2, lookahead_m=40.)))
    server = make_server(reader, port=0, progress_directory=session_root)
    assert server.server_address[0] == '127.0.0.1'
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = 'http://127.0.0.1:'+str(server.server_port)
    try:
        with urlopen(base+'/') as r:
            assert '前後單步'.encode() in r.read()
        with urlopen(base+'/playback.js') as r:
            assert b'AbortController' in r.read()
        with urlopen(base+'/api/image') as r:
            assert r.read().startswith(b'\x89PNG')
        with urlopen(base+'/api/session') as r:
            data = json.load(r)
            assert data['lookahead_m'] == 40. and data['plan_period_s'] == .1
            assert data['generation_progress_enabled']
        with urlopen(base+'/api/progress') as r:
            assert json.load(r)['plan_count'] == 2
        with urlopen(base+'/api/frame?time=.199') as r:
            assert json.load(r)['control_step'] == 1
            assert r.headers['Cache-Control'] == 'no-store'
        for query in ('step=-1', 'step=6', 'step=1.5', 'time=nan', 'step=1&step=2', 'step=1&time=0', ''):
            with pytest.raises(HTTPError) as caught:
                urlopen(base+'/api/frame?'+query)
            assert caught.value.code == 400
        with pytest.raises(HTTPError) as caught:
            urlopen(base+'/metadata.json')
        assert caught.value.code == 404
    finally:
        server.shutdown(); server.server_close(); thread.join(timeout=3)
