"""Coarse sampling is explicit; independent legacy audit keeps its original grid."""
import pytest
from test_robust_adaptive_paths import make_straight
from f1tenth_benchmarks.research.overtaking import robust_adaptive_paths as planner
from f1tenth_benchmarks.research.overtaking.session import SessionConfig, make_plan
from f1tenth_benchmarks.research.run_fixed_grid_benchmark import audit_legacy_grid


def test_override_changes_grid_but_not_legacy_audit():
    geometry = make_straight()
    # Explicit sample_step sets the default dense grid without altering geometry.
    original = planner.generate_robust_paths(geometry, 1.5, 3., lookahead_m=3.,
        generation_mode='direct_only', sample_step_m=.025)
    coarse = planner.generate_robust_paths(geometry, 1.5, 3., lookahead_m=3.,
        generation_mode='direct_only', sample_step_m=.025, validation_step_m=.05)
    assert len(original['paths'][2]['points']) == 121
    assert len(coarse['paths'][2]['points']) == 61
    assert coarse['actual_sample_step_m'] == .05
    assert coarse['validation_sampling_policy']['original_dense_grid_verified'] is False
    assert 'validation_sampling_policy' not in original
    config = SessionConfig(lookahead_m=3., sample_step_m=.025, validation_step_m=.05)
    audit = audit_legacy_grid(geometry, {'result':coarse, 'paths':coarse['paths']}, config)
    assert audit['passed'] and audit['checked_count'] == 5
    assert all(p['point_count'] == 121 for p in audit['paths'])


@pytest.mark.parametrize('step', [0., -.01, .051, float('nan'), float('inf'), True])
def test_invalid_override_rejected(step):
    with pytest.raises(ValueError):
        SessionConfig(validation_step_m=step)
    with pytest.raises(ValueError):
        planner.generate_robust_paths(make_straight(), 1.5, 3., lookahead_m=3.,
            validation_step_m=step)


def test_session_forwards_and_records_override():
    config=SessionConfig(lookahead_m=3., generation_mode='direct_only', validation_step_m=.05)
    event=make_plan(make_straight(), dict(control_step=0,sim_time_s=0.,pose=[1.5,3.,0.],
        valid=True,frenet=dict(valid=True)),config,0)
    assert event['generator_settings']['validation_step_m'] == .05
    assert len(event['paths'][2]['points']) == 61
    with pytest.raises(ValueError):
        SessionConfig(validation_step_m=.05,query_mode='fixed_grid_with_fallback')


def test_real_5cm_direct_acceptance_can_fail_original_25mm_grid():
    """Frozen step807 counterexample: coarse acceptance is not legacy acceptance."""
    from f1tenth_benchmarks.research.core.geometry import load_geometry
    geometry=load_geometry('maps/frenet/esp/geometry','maps/esp.yaml',inspect=True)
    coarse=planner.generate_robust_paths(geometry,21.83373686922052,-7.34608954190013,
        yaw=2.6100938583759383,lookahead_m=5.,generation_mode='direct_only',
        query_mode='fast_with_fallback',validation_step_m=.05)
    path=coarse['paths'][0]
    assert path['sampled_valid'] and path['generation_route']=='direct'
    audit=audit_legacy_grid(geometry,{'result':coarse,'paths':[path]},SessionConfig(lookahead_m=5.))
    assert not audit['passed']
    assert audit['paths'][0]['reason']=='insufficient_lateral_margin'
