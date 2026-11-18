"""Explicitly synthetic manifest contents; no executable model authenticity claim."""
from copy import deepcopy
from pathlib import Path
import pytest
from f1tenth_benchmarks.research.overtaking import model_provenance_v1 as model


def setup_case(tmp_path,commit='707d869073b8b1afe3d5a1d671e872581c01d4bb'):
    # Artificial values validate identity rules, not vehicle or ESP behavior.
    params=dict(model='three_discs',body_reference='geometric_body_centre',
        body_length_m=.51,body_width_m=.27,body_radius_m=.16,
        pair_threshold_m=.09,wall_threshold_m=.07,sample_dt_s=.05,physics_dt_s=.01,
        sample_steps=5,steer_ticks=4,physics_parameters={'timestep':.01},
        controller_config={'lookahead_m':3.},configured_pair_clearance_m=.05,
        configured_wall_clearance_m=.05)
    for name in model.MODEL_PATHS:
        p=tmp_path/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('# synthetic source '+name+'\n')
    source=dict(repository='Ihave2brush/f1tenth_benchmarks',commit='707d869073b8b1afe3d5a1d671e872581c01d4bb',
                model='three_discs',body_reference='geometric_body_centre')
    versions={'diagnostics':'two_car_rollout_diagnostics_v1/1','evidence':'f1tenth_producer_evidence_v1/1'}
    code=model.hash_model_files(tmp_path);policy=model.build_model_policy_v1(source,code,params,versions)
    c=model.build_model_provenance_v1(producer_commit=commit,native_model_source=source,
        model_code_hashes=code,model_parameters_snapshot=params,diagnostics_version=versions)
    kwargs=dict(trusted_policy=policy,expected_policy_sha256=model.model_policy_digest(policy),
        source_root=tmp_path,observed_parameters=params,actual_producer_commit=commit,observed_diagnostics_version=versions)
    return c,kwargs


@pytest.mark.parametrize('commit',['707d869073b8b1afe3d5a1d671e872581c01d4bb','4f586c1b850b927a16c62f55171833781c960681'])
def test_commit_does_not_replace_content_parameter_verification(tmp_path,commit):
    c,k=setup_case(tmp_path,commit);before=deepcopy((c,k));result=model.verify_model_provenance_v1(c,**k)
    assert result['verification_status']=='verified_content_against_explicit_pin'
    assert not result['model_behavior_equivalence_verified'] and not result['historical_authentication_verified']
    assert not result['formal_comparison_qualified'] and (c,k)==before


@pytest.mark.parametrize('change',['actual_code','actual_parameters','code_claim','parameter_claim','producer','source','version','policy_pin','outer_digest','scope','flag'])
def test_changed_or_contradictory_content_not_verified(tmp_path,change):
    c,k=setup_case(tmp_path)
    if change=='actual_code':(tmp_path/model.MODEL_PATHS[0]).write_text('# genuinely changed')
    elif change=='actual_parameters':k['observed_parameters']=deepcopy(k['observed_parameters']);k['observed_parameters']['physics_dt_s']*=2
    elif change=='code_claim':c['model_code_hashes'][model.MODEL_PATHS[0]]='0'*64
    elif change=='parameter_claim':c['model_parameters']['pair_threshold_m']+=.1
    elif change=='producer':c['producer_commit']='a'*40
    elif change=='source':c['native_model_source']['commit']='a'*40
    elif change=='version':c['diagnostics_version']['evidence']='unsupported'
    elif change=='policy_pin':k['expected_policy_sha256']='0'*64
    elif change=='scope':c['model_code_hashes'].pop(model.MODEL_PATHS[0])
    elif change=='flag':c['model_behavior_equivalence_verified']=True
    else:c['contract_sha256']='0'*64
    if change!='outer_digest':
        body={a:b for a,b in c.items() if a!='contract_sha256'};c['contract_sha256']=model._digest('contract',body)
    result=model.verify_model_provenance_v1(c,**k)
    assert result['verification_status']=='inconsistent' and result['verification_reasons']


@pytest.mark.parametrize('missing',['trusted_policy','expected_policy_sha256','source_root','observed_parameters','actual_producer_commit','observed_diagnostics_version'])
def test_unverifiable_is_unconfirmed(tmp_path,missing):
    c,k=setup_case(tmp_path);k[missing]=None
    result=model.verify_model_provenance_v1(c,**k)
    assert result['verification_status']=='unconfirmed' and result['verification_reasons']


@pytest.mark.parametrize('missing',['native_model_source','model_code_digest','model_parameter_digest'])
def test_missing_contract_fields_not_verified(tmp_path,missing):
    c,k=setup_case(tmp_path);c.pop(missing);c['contract_sha256']=model._digest('contract',{a:b for a,b in c.items() if a!='contract_sha256'})
    assert model.verify_model_provenance_v1(c,**k)['verification_status']=='unconfirmed'


def test_file_missing_or_scope_incomplete_not_faked(tmp_path):
    c,k=setup_case(tmp_path);(tmp_path/model.MODEL_PATHS[0]).unlink()
    assert model.verify_model_provenance_v1(c,**k)['verification_status']=='unconfirmed'
    with pytest.raises(ValueError):model.build_model_policy_v1(c['native_model_source'],{},c['model_parameters'],c['diagnostics_version'])


def test_source_archive_without_git_uses_only_explicit_observations(tmp_path,monkeypatch):
    import subprocess
    c,k=setup_case(tmp_path)
    assert not (tmp_path/'.git').exists()
    def no_git_or_execution(*args,**kwargs):raise AssertionError('core must not execute Git or native models')
    monkeypatch.setattr(subprocess,'check_output',no_git_or_execution)
    monkeypatch.setattr(subprocess,'Popen',no_git_or_execution)
    assert model.verify_model_provenance_v1(c,**k)['verification_status']=='verified_content_against_explicit_pin'
    k['actual_producer_commit']=None
    assert model.verify_model_provenance_v1(c,**k)['verification_status']=='unconfirmed'


def test_dirty_checkout_does_not_hide_changed_model_bytes(tmp_path):
    import subprocess
    c,k=setup_case(tmp_path)
    def git(*args):return subprocess.check_output(['git',*args],cwd=tmp_path,text=True).strip()
    git('init','-q');git('add','.')
    git('-c','user.name=Fixture','-c','user.email=fixture@example.invalid','commit','-qm','synthetic model baseline')
    commit=git('rev-parse','HEAD')
    c=model.build_model_provenance_v1(producer_commit=commit,native_model_source=c['native_model_source'],
        model_code_hashes=c['model_code_hashes'],model_parameters_snapshot=c['model_parameters'],diagnostics_version=c['diagnostics_version'])
    k['actual_producer_commit']=commit
    assert model.verify_model_provenance_v1(c,**k)['verification_status']=='verified_content_against_explicit_pin'
    (tmp_path/'outside_scope_notes.txt').write_text('unrelated dirty file')
    assert git('status','--porcelain')
    # Scope verification is not a declaration that the entire repository is clean.
    assert model.verify_model_provenance_v1(c,**k)['verification_status']=='verified_content_against_explicit_pin'
    (tmp_path/model.MODEL_PATHS[0]).write_text('# changed dirty model')
    assert git('rev-parse','HEAD')==commit
    assert model.verify_model_provenance_v1(c,**k)['verification_status']=='inconsistent'


def test_import_and_verification_have_no_runtime_research_dependencies(tmp_path,monkeypatch):
    import ast,builtins,importlib.util
    source=Path(model.__file__)
    imports=[n.module for n in ast.walk(ast.parse(source.read_text())) if isinstance(n,ast.ImportFrom) and n.level]
    assert imports==['plan_binding_v2']
    c,k=setup_case(tmp_path);original=builtins.__import__
    denied={'producer_evidence_v1','rollout_diagnostics_v1','cost_feature_v1','cost_evaluation_v1',
            'two_car_decision','two_car_env','dynamic_models','discs','GlobalPurePursuit'}
    def guarded(name,globals=None,locals=None,fromlist=(),level=0):
        if name.split('.')[-1] in denied or any(n in denied for n in (fromlist or ())):
            raise AssertionError('unexpected native/runtime dependency')
        return original(name,globals,locals,fromlist,level)
    monkeypatch.setattr(builtins,'__import__',guarded)
    spec=importlib.util.spec_from_file_location('f1tenth_benchmarks.research.overtaking._core_import_probe',source)
    fresh=importlib.util.module_from_spec(spec);spec.loader.exec_module(fresh)
    assert fresh.verify_model_provenance_v1(c,**k)['verification_status']=='verified_content_against_explicit_pin'


def test_json_roundtrip_has_no_cost_ranking_or_control_output(tmp_path):
    import json
    c,k=setup_case(tmp_path);before=deepcopy((c,k))
    copy=json.loads(json.dumps(c,allow_nan=False));result=model.verify_model_provenance_v1(copy,**k)
    assert (c,k)==before and copy==c
    assert json.loads(json.dumps(result,allow_nan=False))==result
    assert result['inspection_only'] is True and result['planning_allowed'] is False
    assert result['formal_comparison_qualified'] is False
    assert not {'weights','cost','ranking','selected_candidate_id','steering','target_speed_mps'} & result.keys()


def test_explicit_root_is_independent_of_current_working_directory(tmp_path,monkeypatch):
    c,k=setup_case(tmp_path/'source');other=tmp_path/'other';other.mkdir()
    monkeypatch.chdir(other)
    assert model.verify_model_provenance_v1(c,**k)['verification_status']=='verified_content_against_explicit_pin'
    k['source_root']=None
    assert model.verify_model_provenance_v1(c,**k)['verification_status']=='unconfirmed'


def test_model_scope_rejects_symlink_escape_before_reading(tmp_path):
    c,k=setup_case(tmp_path/'source');external=tmp_path/'not_a_model.py';external.write_text('# outside explicit scope')
    p=Path(k['source_root'])/model.MODEL_PATHS[0];p.unlink();p.symlink_to(external)
    result=model.verify_model_provenance_v1(c,**k)
    assert result['verification_status']=='inconsistent'
    assert 'model_file_outside_root' in result['verification_reasons']


def test_malformed_policy_manifest_container_is_inconsistent_not_exception(tmp_path):
    c,k=setup_case(tmp_path);bad=deepcopy(k['trusted_policy']);bad['code_hashes']=list(bad['code_hashes'])
    k.update(trusted_policy=bad,expected_policy_sha256=model.model_policy_digest(bad))
    result=model.verify_model_provenance_v1(c,**k)
    assert result['verification_status']=='inconsistent'
    assert 'incomplete_model_code_scope' in result['verification_reasons']


def test_policy_builder_rejects_non_mapping_manifest(tmp_path):
    c,k=setup_case(tmp_path);policy=k['trusted_policy']
    with pytest.raises(ValueError,match='incomplete_model_code_scope'):
        model.build_model_policy_v1(policy['native_model_source'],list(policy['code_hashes']),policy['parameters'],policy['diagnostics_version'])
