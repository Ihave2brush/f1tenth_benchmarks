"""Contracts for phase 2 without importing LMPC or an optimizer for control."""
import json
import math
import subprocess
from types import SimpleNamespace

import numpy as np
import pytest

from f1tenth_benchmarks.research.core.centreline_control import CentrelinePurePursuit
from f1tenth_benchmarks.research.overtaking.session import (
    SessionConfig, SessionWriter, PlanScheduler, make_plan, latest_plan,
)
from f1tenth_benchmarks.research.overtaking.verify_session import verify_session
from f1tenth_benchmarks.research import run_centerline_candidates as runner


def state():
    return dict(control_step=5, sim_time_s=.6, pose=[1., 2., .1],
                valid=True, reason=None, frenet=dict(valid=True))


def generator(geometry, x, y, yaw, **settings):
    return dict(paths=[dict(candidate_id=i, geometry_id=geometry.geometry_id,
        sampled_valid=True, status='sampled_valid', reason=None,
        points=[dict(x=x, y=y, yaw=yaw)]) for i in range(1, 6)])


def test_yaw_required_and_actual_start_gate():
    def moved(*args, **kwargs):
        r = generator(*args, **kwargs)
        r['paths'][0]['points'][0]['x'] += .01
        r['paths'][1]['points'][0]['yaw'] += .01
        r['paths'][2]['points'] = []
        return r
    result = make_plan(SimpleNamespace(geometry_id='g'), state(), SessionConfig(), 0, generator=moved)
    assert result['sampled_valid_count'] == 2
    assert [p['reason'] for p in result['paths'][:3]] == ['start_pose_mismatch']*3
    assert result['source_pose'] == [1., 2., .1]
    assert result['available_control_step'] == 5


def test_invalid_source_never_calls_generator_and_keeps_five_ids():
    s = state()
    s.update(valid=False, reason='localization_jump')
    def unreachable(*args, **kwargs):
        pytest.fail('invalid pose must not generate')
    event = make_plan(SimpleNamespace(geometry_id='g'), s, SessionConfig(), 4, generator=unreachable)
    assert event['status'] == 'skipped'
    assert event['reason'] == 'localization_jump'
    assert [p['candidate_id'] for p in event['paths']] == [1, 2, 3, 4, 5]


def test_generator_value_error_keeps_reason_and_five_ids():
    def failed(*args, **kwargs):
        raise ValueError('Invalid or insufficient boundary at start')
    event = make_plan(SimpleNamespace(geometry_id='g'), state(), SessionConfig(), 0, generator=failed)
    assert event['status'] == 'rejected'
    assert all(p['reason'] == event['reason'] for p in event['paths'])
    assert len(event['paths']) == 5


def test_scheduler_uses_sim_time_and_skips_missed_slots():
    schedule = PlanScheduler(.2, .1)
    assert schedule.due(.1)
    assert not schedule.due(.1)
    assert not schedule.due(.2)
    assert schedule.due(.3)
    assert schedule.due(.95)
    assert not schedule.due(.96)
    assert schedule.due(1.1)


def test_backward_lookup_replaces_success_with_failure_and_hides_future():
    plans = [dict(plan_id=0, source_control_step=3, available_control_step=3, status='completed'),
             dict(plan_id=1, source_control_step=7, available_control_step=8, status='rejected')]
    assert latest_plan(plans, 2) is None
    assert latest_plan(plans, 7)['plan_id'] == 0
    assert latest_plan(plans, 8)['status'] == 'rejected'
    assert latest_plan(plans, 4)['plan_id'] == 0


def test_writer_preserves_streams_and_refuses_overwrite(tmp_path):
    root = tmp_path/'session'
    writer = SessionWriter(root, dict(schema_version=1))
    writer.append('states', dict(control_step=0, sim_time_s=.1, reason='定位失敗'))
    writer.finalize(dict(end_reason='error'))
    raw = (root/'states.jsonl').read_bytes()
    index = json.loads((root/'playback_index.json').read_text())['streams']['states'][0]
    assert json.loads(raw[index['byte_offset']:index['byte_offset']+index['byte_length']])['reason'] == '定位失敗'
    with pytest.raises(FileExistsError):
        SessionWriter(root, {})
    with pytest.raises(RuntimeError):
        writer.append('states', {})


def test_shared_pp_low_speed_and_closing_segment():
    # Explicitly close the final edge; test a pose on that edge approaching s=0.
    points = np.array([[0., 0.], [3., 0.], [3., 3.], [0., 3.]])
    controller = CentrelinePurePursuit(points, 1.2, .6, .33, .4)
    action = controller.plan(dict(pose=np.array([0., .3, -math.pi/2])))
    assert action[1] == 1.2
    assert abs(action[0]) <= .4
    assert np.isfinite(controller.diagnostic['lookahead_xy']).all()
    assert controller.diagnostic['lookahead_xy'][0] > 0


@pytest.mark.parametrize('settings', [dict(plan_period_s=0), dict(speed_mps=float('nan')),
    dict(laps=True), dict(max_refinements=6), dict(sample_step_m=-1)])
def test_configuration_rejected_before_session(settings):
    with pytest.raises(ValueError):
        SessionConfig(**settings)


def test_version_keeps_git_ref_when_image_has_no_git(monkeypatch):
    expected = runner.source_version()
    def missing_git(*args, **kwargs):
        raise FileNotFoundError('git absent from simulation image')
    monkeypatch.setattr(subprocess, 'check_output', missing_git)
    fallback = runner.source_version()
    assert fallback['commit'] == expected['commit']
    assert fallback['branch'] == expected['branch']


def test_real_timeout_records_terminal_sample_and_replays(tmp_path):
    root = tmp_path/'timeout'
    summary = runner.run_session(root, SessionConfig(timeout_s=.2))
    assert summary['end_reason'] == 'timeout'
    assert not summary['recording_complete']
    assert summary['state_count'] == 3
    assert summary['plan_count'] == 2
    plans = [json.loads(s) for s in (root/'plans.jsonl').read_text().splitlines()]
    assert plans[-1]['status'] == 'skipped'
    assert plans[-1]['source_control_step'] == 2
    assert verify_session(root, require_complete=False)['passed']
    # A result borrowing a future source pose cannot pass independent replay.
    plans[0]['source_pose'][0] += 1
    # Rebuild a coherent index so rejection must come from source validation,
    # rather than merely from a damaged byte range after reserialization.
    index = json.loads((root/'playback_index.json').read_text())
    offset = 0
    encoded = []
    for plan, item in zip(plans, index['streams']['plans']):
        raw = (json.dumps(plan)+'\n').encode()
        item.update(byte_offset=offset, byte_length=len(raw))
        offset += len(raw)
        encoded.append(raw)
    (root/'plans.jsonl').write_bytes(b''.join(encoded))
    (root/'playback_index.json').write_text(json.dumps(index))
    with pytest.raises(AssertionError):
        verify_session(root, require_complete=False)


def test_runtime_failure_finalizes_existing_records(tmp_path, monkeypatch):
    def fail(*args, **kwargs):
        raise RuntimeError('injected planner failure')
    monkeypatch.setattr(runner, 'make_plan', fail)
    root = tmp_path/'error'
    with pytest.raises(RuntimeError, match='injected planner failure'):
        runner.run_session(root, SessionConfig(timeout_s=.2))
    summary = json.loads((root/'summary.json').read_text())
    plan = json.loads((root/'plans.jsonl').read_text())
    assert summary['error'] == 'RuntimeError: injected planner failure'
    assert summary['state_count'] == 1 and summary['plan_count'] == 1
    assert len(plan['paths']) == 5 and plan['status'] == 'error'
