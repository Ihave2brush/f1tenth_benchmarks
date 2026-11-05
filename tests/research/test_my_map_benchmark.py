"""MY map source integration and a common 3 m comparison policy."""
import json
from f1tenth_benchmarks.research.run_centerline_candidates import run_session
from f1tenth_benchmarks.research.overtaking.session import SessionConfig
from f1tenth_benchmarks.research.overtaking.verify_session import verify_session
from f1tenth_benchmarks.research.run_my_map_benchmark import (
    ARMS, GEOMETRY, MAP_YAML, config_for, freeze_inputs)


def test_all_modes_share_3m_and_limits():
    assert len(ARMS)==9
    for arm in ARMS:
        config=config_for(arm)
        assert config.lookahead_m==SessionConfig().lookahead_m==3.
        assert config.speed_mps==1.2 and config.safety_margin_m==.2
        assert config.pp_lookahead_m == .3
        assert config.max_slope==1. and config.max_iterations==150
        assert config.validation_step_m==(.05 if arm.endswith('5cm') else None)


def test_my_map_short_source_replay_and_frozen_inputs(tmp_path):
    root=tmp_path/'source'
    summary=run_session(root,SessionConfig(generation_mode='direct_only',
        query_mode='fast_with_fallback',max_control_steps=2),geometry_dir=GEOMETRY,map_yaml=MAP_YAML)
    assert summary['control_steps']==2 and not summary['collision']
    assert verify_session(root,require_complete=False)['passed']
    manifest=freeze_inputs(root)
    assert manifest['map']=='my_map' and manifest['lookahead_m']==3.
    assert abs(manifest['length_m']-11.93004280322861)<1e-10
    assert len(manifest['states'])==3 and len(manifest['fixed_cases'])==7
    assert all(s['valid'] for s in manifest['states'])
    metadata = json.loads((root/'metadata.json').read_text())
    assert metadata['map']=='my_map'
    assert metadata['config']['pp_lookahead_m']==.3
