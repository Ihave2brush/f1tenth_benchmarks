"""Pinned local model content/parameter contract; never behavioral authentication."""
from copy import deepcopy
from pathlib import Path
import hashlib
import re

from .plan_binding_v2 import content_digest

SCHEMA='f1tenth_model_provenance_v1'
FLAGS=dict(inspection_only=True,planning_allowed=False,historical_authentication_verified=False,
           model_behavior_equivalence_verified=False,formal_comparison_qualified=False)
MODEL_PATHS=(
 'f1tenth_benchmarks/research/overtaking/two_car_decision.py',
 'f1tenth_benchmarks/research/overtaking/two_car_env.py',
 'f1tenth_benchmarks/research/overtaking/raceline_cruise.py',
 'f1tenth_benchmarks/research/collision/discs.py',
 'f1tenth_benchmarks/research/collision/multi_car.py',
 'f1tenth_benchmarks/research/run_recorded_candidate_pp.py',
 'f1tenth_benchmarks/research/core/centreline_control.py',
 'f1tenth_benchmarks/classic_racing/GlobalPurePursuit.py',
 'f1tenth_benchmarks/simulator/dynamics_simulator.py',
 'f1tenth_benchmarks/simulator/dynamic_models.py',
 'f1tenth_benchmarks/utils/BasePlanner.py',
 'f1tenth_benchmarks/utils/track_utils.py',
 'f1tenth_benchmarks/research/core/geometry/tracker.py',
 'f1tenth_benchmarks/research/core/frenet_lap_tracking.py',
 'f1tenth_benchmarks/research/overtaking/mm_bspline_repair.py',
 'f1tenth_benchmarks/research/overtaking/millimetre_domain.py',
 'f1tenth_benchmarks/research/overtaking/footprint_validation.py',
 'f1tenth_benchmarks/research/overtaking/session.py',
 'f1tenth_benchmarks/research/overtaking/reference_path.py')
# Versioned data schema, independent of any runtime collector or controller.
PARAMETER_KEYS=('model','body_reference','body_length_m','body_width_m',
                'body_radius_m','pair_threshold_m','wall_threshold_m','sample_dt_s','physics_dt_s',
                'sample_steps','steer_ticks','physics_parameters','controller_config',
                'configured_pair_clearance_m','configured_wall_clearance_m')


def _digest(domain,obj):return content_digest(SCHEMA+':'+domain,obj)
def _same(a,b):return _digest('compare',a)==_digest('compare',b)
def _sha(v,n):
    if not isinstance(v,str) or re.fullmatch('[0-9a-f]{'+str(n)+'}',v) is None:raise ValueError('invalid_digest_or_commit')
def _source(v):
    if not isinstance(v,dict) or not isinstance(v.get('repository'),str) or not v['repository']:
        raise ValueError('missing_native_model_source')
    _sha(v.get('commit'),40)
    if v.get('model')!='three_discs' or v.get('body_reference')!='geometric_body_centre':raise ValueError('unsupported_model_descriptor')


def hash_model_files(root):
    """Explicit outside-hot-path IO; exact byte identity, fixed reviewed scope."""
    root=Path(root).resolve();out={}
    for name in MODEL_PATHS:
        p=(root/name).resolve()
        if root not in p.parents:raise ValueError('model_file_outside_root')
        out[name]=hashlib.sha256(p.read_bytes()).hexdigest()
    return out


def model_parameters(runtime):
    if not isinstance(runtime,dict) or any(k not in runtime or runtime[k] is None for k in PARAMETER_KEYS):
        raise ValueError('missing_model_parameters')
    result={k:deepcopy(runtime[k]) for k in PARAMETER_KEYS}
    _digest('parameter_validation',result)
    return result


def build_model_policy_v1(native_model_source, code_hashes, parameters, diagnostics_version):
    """Caller-reviewed manifest. Pin must be provided separately when verifying."""
    _source(native_model_source)
    if not isinstance(code_hashes,dict) or set(code_hashes)!=set(MODEL_PATHS):raise ValueError('incomplete_model_code_scope')
    for h in code_hashes.values():_sha(h,64)
    parameters=model_parameters(parameters)
    if not isinstance(diagnostics_version,dict) or not diagnostics_version or any(not isinstance(v,str) or not v for v in diagnostics_version.values()):
        raise ValueError('missing_diagnostics_version')
    return dict(schema='f1tenth_model_policy_v1',schema_version=1,native_model_source=deepcopy(native_model_source),
        code_hashes=deepcopy(code_hashes),parameters=parameters,diagnostics_version=deepcopy(diagnostics_version),
        trust_basis='explicit_caller_selected_local_manifest_not_independent_certification',**FLAGS)


def model_policy_digest(policy):return _digest('trusted_policy',policy)


def build_model_provenance_v1(*, producer_commit,native_model_source,model_code_hashes,
                              model_parameters_snapshot,diagnostics_version):
    _sha(producer_commit,40)
    policy=build_model_policy_v1(native_model_source,model_code_hashes,model_parameters_snapshot,diagnostics_version)
    result=dict(schema=SCHEMA,schema_version=1,producer_commit=producer_commit,
        native_model_source=policy['native_model_source'],model_code_hashes=policy['code_hashes'],
        model_code_digest=_digest('model_code',policy['code_hashes']),
        model_parameters=policy['parameters'],model_parameter_digest=_digest('model_parameters',policy['parameters']),
        diagnostics_version=policy['diagnostics_version'],verification_status='unconfirmed',
        verification_reasons=['verification_not_performed'],native_model_source_relationship='declared_source_not_behavior_proof',**FLAGS)
    result['contract_sha256']=_digest('contract',result)
    return result


def verify_model_provenance_v1(contract, *, trusted_policy=None,expected_policy_sha256=None,
                                source_root=None,observed_parameters=None,actual_producer_commit=None,
                                observed_diagnostics_version=None):
    """Recompute actual content and compare to a separately pinned explicit policy."""
    reasons=[];status='unconfirmed';code=None;params=None
    try:
        c=deepcopy(contract)
        if c is None:raise ValueError('missing_model_contract')
        if not isinstance(c,dict) or c.get('schema')!=SCHEMA or type(c.get('schema_version')) is not int or c['schema_version']!=1:
            raise ValueError('unsupported_model_provenance_schema')
        for k,v in FLAGS.items():
            if c.get(k) is not v:raise ValueError('invalid_model_provenance_flags')
        digest=c.pop('contract_sha256',None)
        if digest!=_digest('contract',c):raise ValueError('model_contract_digest_mismatch')
        required=('native_model_source','producer_commit','model_code_hashes','model_code_digest',
                  'model_parameters','model_parameter_digest','diagnostics_version')
        if any(c.get(k) is None for k in required):raise ValueError('missing_model_fields')
        _source(c.get('native_model_source'));_sha(c.get('producer_commit'),40)
        if c.get('verification_status')!='unconfirmed':raise ValueError('contract_cannot_self_verify')
        if trusted_policy is None or expected_policy_sha256 is None:reasons.append('missing_explicit_policy_pin')
        if source_root is None:reasons.append('missing_actual_model_files')
        if observed_parameters is None:reasons.append('missing_actual_model_parameters')
        if actual_producer_commit is None:reasons.append('missing_actual_producer_commit')
        if observed_diagnostics_version is None:reasons.append('missing_actual_diagnostics_version')
        if not reasons:
            if model_policy_digest(trusted_policy)!=expected_policy_sha256:raise ValueError('model_policy_pin_mismatch')
            policy=build_model_policy_v1(trusted_policy['native_model_source'],trusted_policy['code_hashes'],
                trusted_policy['parameters'],trusted_policy['diagnostics_version'])
            if not _same(policy,trusted_policy):raise ValueError('invalid_model_policy')
            code=hash_model_files(source_root);params=model_parameters(observed_parameters)
            if not _same(c['native_model_source'],policy['native_model_source']):raise ValueError('model_source_claim_mismatch')
            if c['producer_commit']!=actual_producer_commit:raise ValueError('producer_commit_mismatch')
            if not _same(c['model_code_hashes'],code) or not _same(code,policy['code_hashes']):raise ValueError('actual_model_code_mismatch')
            if c['model_code_digest']!=_digest('model_code',code):raise ValueError('model_code_digest_mismatch')
            if not _same(c['model_parameters'],params) or not _same(params,policy['parameters']):raise ValueError('actual_model_parameter_mismatch')
            if c['model_parameter_digest']!=_digest('model_parameters',params):raise ValueError('model_parameter_digest_mismatch')
            if not _same(c['diagnostics_version'],observed_diagnostics_version) or not _same(observed_diagnostics_version,policy['diagnostics_version']):
                raise ValueError('diagnostics_version_mismatch')
            if params['model']!=policy['native_model_source']['model'] or params['body_reference']!=policy['native_model_source']['body_reference']:
                raise ValueError('runtime_model_descriptor_mismatch')
            status='verified_content_against_explicit_pin'
    except (ValueError,KeyError,TypeError,OSError) as exc:
        status='unconfirmed' if isinstance(exc,OSError) or str(exc).startswith('missing_') else 'inconsistent'
        reasons.append(str(exc))
    return dict(schema='f1tenth_model_verification_v1',schema_version=1,verification_status=status,
        verification_reasons=reasons,contract_sha256=contract.get('contract_sha256') if isinstance(contract,dict) else None,
        policy_sha256=expected_policy_sha256,model_code_digest=_digest('model_code',code) if code is not None else None,
        model_parameter_digest=_digest('model_parameters',params) if params is not None else None,
        commit_identity_scope='caller_observed_checkout_not_historical_authentication',**FLAGS)
