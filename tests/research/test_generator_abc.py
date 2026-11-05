"""Experiment stage timers avoid nested double counts; quality is sampled."""
import pytest

from f1tenth_benchmarks.research import run_generator_abc as experiment


def test_nested_stage_time_is_exclusive_and_hooks_are_restored(monkeypatch):
    def inner():
        return 7
    def outer():
        return experiment.planner._reference_data()
    monkeypatch.setattr(experiment.planner, '_reference_data', inner)
    monkeypatch.setattr(experiment.planner, 'query_road_sections', outer)
    ticks = iter([0., 1., 3., 6.])
    monkeypatch.setattr(experiment, 'perf_counter', lambda: next(ticks))
    with experiment.stage_profile() as profile:
        assert experiment.planner.query_road_sections() == 7
    assert profile['timers']['road_sections'] == dict(calls=1, inclusive_s=6., exclusive_s=4.)
    assert profile['timers']['curvature_references'] == dict(calls=1, inclusive_s=2., exclusive_s=2.)
    assert experiment.planner._reference_data is inner
    assert experiment.planner.query_road_sections is outer


def test_sampled_quality_distinguishes_backtracking_and_rejected_paths():
    path = dict(sampled_valid=True, start_position_error_m=0., start_heading_error_rad=0.,
        points=[dict(d=d, d_prime=.2, curvature_inv_m=.1*i, s_progress=float(i),
                     lateral_margin_m=.25) for i, d in enumerate([0., 1., 0., 1.])])
    quality = experiment.path_quality(path)
    assert quality['excess_lateral_tv_m'] == 2.
    assert quality['increment_sign_reversals'] == 2
    assert quality['max_sample_curvature_change_per_m'] == pytest.approx(.1)
    assert quality['min_lateral_margin_m'] == .25
    path['sampled_valid'] = False
    assert experiment.path_quality(path) is None
