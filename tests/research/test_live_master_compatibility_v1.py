"""Isolated PR18 + Diagnostics integration regressions; no control changes."""
import inspect
from types import SimpleNamespace
import pytest
from f1tenth_benchmarks.research import run_two_car_candidates as cli
from f1tenth_benchmarks.research.overtaking.session import SessionConfig

@pytest.mark.parametrize('name', ['new','esp','my_map'])
def test_phase2_guard_precedes_any_map_io(name, tmp_path, monkeypatch):
    monkeypatch.setattr(cli,'TrackData',lambda *a,**kw:pytest.fail('loaded track before phase guard'))
    with pytest.raises(ValueError,match='requires Phase 2'):
        cli.run(tmp_path/'unused',map_name=name,phase=1,rollout_diagnostics=True)

@pytest.mark.parametrize('name,step',[('new',.005),('esp',.001),('my_map',.001)])
def test_map_validation_reaches_track_load(name,step,tmp_path,monkeypatch):
    class Reached(Exception):pass
    def track(*a,**kw):raise Reached
    monkeypatch.setattr(cli,'TrackData',track)
    with pytest.raises(Reached):cli.run(tmp_path/'unused',map_name=name,phase=2,rollout_diagnostics=True)

def test_new_domain_exact_step(tmp_path,monkeypatch):
    track=SimpleNamespace(geometry=object());seen={}
    def build(g,**kw):seen.update(kw);return SimpleNamespace(step=kw['step'])
    monkeypatch.setattr(cli.MillimetreDomain,'build',build)
    domain,_=cli._load_domain(track,'new',None,tmp_path,.185)
    assert domain.step==.005 and seen['margin']==.185
    monkeypatch.setattr(cli.MillimetreDomain,'load',lambda *a,**kw:SimpleNamespace(step=.001))
    with pytest.raises(ValueError,match='5 mm'):cli._load_domain(track,'new',tmp_path,tmp_path,.185)

def test_new_map_config_retains_native_pp_setting():
    assert SessionConfig().for_map('new').pp_lookahead_m==.3

def test_diagnostics_flag_is_opt_in():
    assert inspect.signature(cli.run).parameters['rollout_diagnostics'].default is False
