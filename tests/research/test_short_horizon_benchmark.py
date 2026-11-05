"""Benchmark gates distinguish timely useful results and frozen motion inputs."""
import hashlib
import json

import pytest

from f1tenth_benchmarks.research import run_short_horizon_benchmark as bench


def row(total, valid_ids):
    return dict(total_s=total, make_plan_s=total-.001, encoding_s=.001,
                valid_ids=valid_ids, valid_count=len(valid_ids), over_budget=total > .1,
                timely_valid_count=len(valid_ids) if total <= .1 else 0)


def test_summary_does_not_count_late_or_empty_results_as_useful():
    summary = bench.summarize([row(.05, []), row(.15, [1, 2, 3, 4, 5]), row(.08, [2])])
    assert summary['over_budget_count'] == 1
    assert summary['useful_within_budget_count'] == 1
    assert summary['zero_valid_count'] == 1
    assert summary['valid_id_counts'] == {'1': 1, '2': 2, '3': 1, '4': 1, '5': 1}
    assert summary['total_s']['median'] == .08
    assert summary['total_s']['max'] == .15


def test_empty_summary_is_rejected():
    with pytest.raises(ValueError):
        bench.summarize([])


def test_encoding_can_make_geometrically_valid_output_late(monkeypatch):
    clocks = iter([0., .07, .11])
    monkeypatch.setattr(bench, 'perf_counter', lambda: next(clocks))
    event = dict(paths=[dict(candidate_id=i, sampled_valid=i == 2, reason=None,
        start_position_error_m=0., start_heading_error_rad=0.,
        polyline_collision_verified=True, points=[]) for i in range(1, 6)])
    monkeypatch.setattr(bench, 'make_plan', lambda *args: event)
    result = bench.measure(None, dict(control_step=7, sim_time_s=.7), bench.SessionConfig(), 0)
    assert result['valid_ids'] == [2]
    assert result['over_budget'] and result['timely_valid_count'] == 0
    assert result['make_plan_s'] == .07
    assert result['encoding_s'] == pytest.approx(.04)
    assert result['repair_attempted_count'] is None


def inputs(tmp_path):
    session = tmp_path/'source'
    session.mkdir()
    states = [dict(control_step=i, sim_time_s=i*.1, pose=[i*.1, 0., 0.],
                   speed_mps=1., valid=True, reason=None, frenet=dict(valid=True))
              for i in range(5)]
    lines = [(json.dumps(s)+'\n').encode() for s in states]
    (session/'states.jsonl').write_bytes(b''.join(lines))
    settings = dict(lookahead_m=40., max_curvature=1.2)
    digest = hashlib.sha256(lines[-1]).hexdigest()
    manifest = dict(source_session=str(session), geometry_id='fixture',
        source_sha256={'states.jsonl': bench.sha(session/'states.jsonl')}, cases=[
            dict(name='seam_3m', control_step=4, pose=states[-1]['pose'],
                 state_sha256=digest, settings=settings),
            dict(name='seam_40m', control_step=4, pose=states[-1]['pose'],
                 state_sha256=digest, settings=settings),
            dict(name='outside_map_3m', control_step=None, pose=[1e6, 1e6, 0.],
                 state_sha256=None, settings=settings)])
    path = tmp_path/'cases.json'
    path.write_text(json.dumps(manifest))
    return path, session


def test_freeze_deduplicates_pose_and_preserves_time_and_end_segment(tmp_path):
    path, session = inputs(tmp_path)
    before = bench.sha(session/'states.jsonl')
    frozen = bench.freeze_inputs(path, steps=3)
    assert frozen['horizons_m'] == [5., 6., 7.]
    assert [c['name'] for c in frozen['fixed_cases']] == ['seam', 'outside_map']
    assert not frozen['fixed_cases'][-1]['expected_useful']
    assert len(frozen['moving_segments']) == 1
    segment = frozen['moving_segments'][0]['states']
    assert [r['state']['control_step'] for r in segment] == [2, 3, 4]
    assert segment[-1]['state']['sim_time_s'] == .4
    assert segment[-1]['source_state_sha256'] == frozen['fixed_cases'][0]['source_state_sha256']
    assert frozen['settings'] == {'max_curvature_inv_m': 1.2}
    assert bench.sha(session/'states.jsonl') == before


def test_freeze_rejects_changed_source_session(tmp_path):
    path, session = inputs(tmp_path)
    with (session/'states.jsonl').open('ab') as stream:
        stream.write(b'\n')
    with pytest.raises(AssertionError):
        bench.freeze_inputs(path, steps=3)
