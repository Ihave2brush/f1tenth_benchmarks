"""V1 immutable pre-evaluation capture, post-step binding; no authentication/control."""
from collections.abc import Mapping
from copy import deepcopy
from pathlib import Path
from types import MappingProxyType
import json
import re

import numpy as np

from .plan_binding_v2 import build_plan_binding, verify_plan_binding, content_digest, geometry_evidence
from .rollout_diagnostics_v1 import validate_record, snapshot_sha256

SCHEMA = 'f1tenth_producer_evidence_v1'
FLAGS = dict(inspection_only=True, planning_allowed=False,
             historical_authentication_verified=False, formal_comparison_qualified=False)
CONTEXT_KEYS = ('geometry_id','frame_id','producer_commit','source_file_sha256',
                'physics_parameters','vehicle_parameters')
RUNTIME_KEYS = ('geometry_id','frame_id','model','body_reference','body_length_m','body_width_m',
                'body_radius_m','pair_threshold_m','wall_threshold_m','sample_dt_s','physics_dt_s',
                'sample_steps','steer_ticks','physics_parameters','controller_config',
                'configured_pair_clearance_m','configured_wall_clearance_m')


def _freeze(value):
    # Capture only. No canonicalization, hash, validation, or filesystem access.
    if isinstance(value, Mapping):
        return MappingProxyType({k:_freeze(v) for k,v in value.items()})
    if isinstance(value, (list,tuple)):return tuple(_freeze(v) for v in value)
    if isinstance(value,np.ndarray):
        result=value.copy();result.setflags(write=False);return result
    return deepcopy(value)


def _thaw(value):
    if isinstance(value,Mapping):return {k:_thaw(v) for k,v in value.items()}
    if isinstance(value,tuple):return [_thaw(v) for v in value]
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    return deepcopy(value)


def _same(a,b,reason):
    if content_digest('producer_evidence_compare',a)!=content_digest('producer_evidence_compare',b):
        raise ValueError(reason)


def _context(meta, runtime, record, *, require_runtime=True):
    if not isinstance(meta,dict):raise ValueError('missing_session_metadata')
    for key in CONTEXT_KEYS:
        if key not in meta or meta[key] is None:raise ValueError('missing_context:'+key)
    if not isinstance(meta['producer_commit'],str) or not re.fullmatch('[0-9a-f]{40}',meta['producer_commit']):
        raise ValueError('invalid_producer_commit')
    hashes=meta['source_file_sha256']
    if not isinstance(hashes,dict) or not hashes:raise ValueError('missing_code_hashes')
    if not all(isinstance(k,str) and isinstance(v,str) and re.fullmatch('[0-9a-f]{64}',v) for k,v in hashes.items()):
        raise ValueError('invalid_code_hash')
    for key in ('physics_parameters','vehicle_parameters'):
        if not isinstance(meta[key],dict) or not meta[key]:raise ValueError('missing_context:'+key)
    for key in ('geometry_id','frame_id'):_same(meta[key],record['metadata'][key],'context_mismatch:'+key)
    if not require_runtime:return
    if not isinstance(runtime,dict):raise ValueError('missing_runtime_metadata')
    for key in RUNTIME_KEYS:
        if key not in runtime:raise ValueError('missing_runtime:'+key)
        _same(runtime[key],record['metadata'][key],'runtime_mismatch:'+key)



class ProducerCaptureV1:
    """One source frame. Private frozen snapshots cannot be mutated by callers."""
    def __init__(self, source, session_metadata, runtime_metadata, *, length_m=None):
        self._source=_freeze(source);self._metadata=_freeze(session_metadata)
        self._runtime=_freeze(runtime_metadata);self._length=length_m
        self._plan=None;self._plan_sequence=None;self._groups={};self._options=[];self._sequence=0
        self._rollouts={};self._closed=False

    def _next(self):
        if self._closed:raise ValueError('capture_already_finalized')
        self._sequence+=1;return self._sequence

    def capture_plan(self, plan):
        sequence=self._next()
        if self._plan_sequence is not None:raise ValueError('duplicate_plan_capture')
        if any(o['candidate_id'] is not None for o in self._options):raise ValueError('plan_capture_after_candidate')
        self._plan=_freeze(plan);self._plan_sequence=sequence

    def capture_group(self, group):
        sequence=self._next()
        try:gid=group['prediction_group_id']
        except (KeyError,TypeError) as exc:raise ValueError('capture_missing_prediction_group_id') from exc
        if gid in self._groups:raise ValueError('duplicate_prediction_group')
        self._groups[gid]=_freeze(dict(capture_sequence=sequence,group=group))

    def capture_option(self, option):
        sequence=self._next()
        try:eid=option['evaluation_id'];gid=option['prediction_group_id'];candidate_id=option['candidate_id']
        except (KeyError,TypeError) as exc:raise ValueError('capture_missing_option_identity') from exc
        if gid not in self._groups:raise ValueError('missing_pre_evaluation_group')
        if any(o['evaluation_id']==eid for o in self._options):raise ValueError('duplicate_evaluation_id')
        if option['candidate_id'] is not None and self._plan_sequence is None:
            raise ValueError('missing_pre_evaluation_plan')
        keys=('evaluation_id','prediction_group_id','candidate_id','reference_id','target_speed_mps',
              'candidate_snapshot','reference_xy')
        row={k:option.get(k) for k in keys}
        row.update(capture_sequence=sequence,plan_present_at_capture=self._plan_sequence is not None)
        self._options.append(_freeze(row))

    def capture_rollout_input(self, evaluation_id, pp):
        sequence=self._next()
        if not any(o['evaluation_id']==evaluation_id for o in self._options):raise ValueError('unknown_evaluation_id')
        if evaluation_id in self._rollouts:raise ValueError('duplicate_rollout_input')
        try:path=pp.path
        except AttributeError as exc:raise ValueError('capture_missing_pp_path') from exc
        self._rollouts[evaluation_id]=_freeze(dict(capture_sequence=sequence,xy=path,
            pp_settings={k:getattr(pp,k,None) for k in ('speed','lookahead','wheelbase','max_steer')}))

    def finalize(self, record, *, geometry_manifest_path=None, expected_manifest_sha256=None):
        """Caller must invoke only after native step. All expensive work is here."""
        self._next();self._closed=True
        r=validate_record(record);source=_thaw(self._source);meta=_thaw(self._metadata)
        runtime=_thaw(self._runtime);plan=_thaw(self._plan)
        _context(meta,runtime,r)
        _same(source,r['source'],'captured_source_mismatch');_same(plan,r['plan_snapshot'],'captured_plan_mismatch')
        binding=build_plan_binding(plan,session_metadata=meta,length_m=self._length,
            geometry_manifest_path=geometry_manifest_path,expected_manifest_sha256=expected_manifest_sha256) if plan is not None else None
        result=dict(schema=SCHEMA,schema_version=1,capture_kind='in_process_pre_evaluation_snapshot',
            capture_phase='source_before_evaluation_candidate_plan_before_candidate_option',digest_phase='after_native_step',
            source=source,session_metadata=meta,runtime_metadata=runtime,plan_snapshot=plan,
            plan_capture_sequence=self._plan_sequence,plan_binding_v2=binding,
            groups=[_thaw(v) for v in self._groups.values()],options=[_thaw(v) for v in self._options],
            rollout_inputs={k:_thaw(v) for k,v in self._rollouts.items()},
            native_option_order=[o['evaluation_id'] for o in r['options']],
            sidecar_record_sha256=snapshot_sha256(r),source_snapshot_sha256=snapshot_sha256(source),
            plan_snapshot_sha256=snapshot_sha256(plan) if plan is not None else None,**FLAGS)
        result['evidence_sha256']=content_digest(SCHEMA,result)
        verify_producer_evidence_v1(result,r,session_metadata=meta,
            geometry_manifest_path=geometry_manifest_path,expected_manifest_sha256=expected_manifest_sha256)
        return result


def _verify_producer_evidence_v1(evidence, record, *, session_metadata,
                                geometry_manifest_path=None, expected_manifest_sha256=None):
    """Local content verification; producer timing remains a capture claim."""
    r=validate_record(record);e=deepcopy(evidence)
    if e.get('schema')!=SCHEMA or type(e.get('schema_version')) is not int or e['schema_version']!=1:
        raise ValueError('unsupported_producer_evidence_schema')
    for key,value in FLAGS.items():
        if e.get(key) is not value:raise ValueError('invalid_evidence_flag:'+key)
    if (e.get('capture_kind')!='in_process_pre_evaluation_snapshot' or
        e.get('capture_phase')!='source_before_evaluation_candidate_plan_before_candidate_option' or e.get('digest_phase')!='after_native_step'):
        raise ValueError('invalid_capture_phase')
    digest=e.pop('evidence_sha256',None)
    if digest!=content_digest(SCHEMA,e):raise ValueError('evidence_digest_mismatch')
    _same(e['session_metadata'],session_metadata,'producer_metadata_mismatch')
    _context(session_metadata,e['runtime_metadata'],r)
    _same(e['source'],r['source'],'producer_source_mismatch')
    if e['source_snapshot_sha256']!=snapshot_sha256(r['source']):raise ValueError('source_digest_mismatch')
    if e['sidecar_record_sha256']!=snapshot_sha256(r):raise ValueError('sidecar_digest_mismatch')
    _same(e['plan_snapshot'],r['plan_snapshot'],'producer_plan_mismatch')
    plan=r['plan_snapshot'];binding=e['plan_binding_v2'];sequence=e['plan_capture_sequence']
    if plan is not None:
        if type(sequence) is not int or sequence<=0 or binding is None:raise ValueError('missing_plan_capture')
        if e['plan_snapshot_sha256']!=snapshot_sha256(plan):raise ValueError('plan_digest_mismatch')
        verify_plan_binding(plan,binding,session_metadata=session_metadata,
            length_m=binding['lookahead_length_m'],geometry_manifest_path=geometry_manifest_path,
            expected_manifest_sha256=expected_manifest_sha256)
    elif binding is not None or sequence is not None or e['plan_snapshot_sha256'] is not None:
        raise ValueError('fabricated_plan_binding')
    groups={};sequences=[]
    for row in e['groups']:
        group=row['group'];gid=group['prediction_group_id']
        if gid in groups:raise ValueError('duplicate_prediction_group')
        groups[gid]=row;sequences.append(row['capture_sequence'])
    _same([g['group'] for g in e['groups']],r['lead_predictions'],'producer_group_mismatch')
    native={o['evaluation_id']:o for o in r['options']};seen=set()
    _same(e['native_option_order'],[o['evaluation_id'] for o in r['options']],'native_order_mismatch')
    tracked={p['tracking_path_id']:p for p in r['tracking_paths']};rollout_ids=set()
    for row in e['options']:
        eid=row['evaluation_id']
        if eid in seen or eid not in native:raise ValueError('duplicate_or_unknown_evaluation_id')
        seen.add(eid);option=native[eid];gid=row['prediction_group_id'];seq=row['capture_sequence'];sequences.append(seq)
        if gid not in groups or type(seq) is not int or seq<=groups[gid]['capture_sequence']:
            raise ValueError('invalid_capture_order')
        for key in ('prediction_group_id','candidate_id','reference_id','target_speed_mps','candidate_snapshot','reference_xy'):
            _same(row[key],option[key],'producer_option_mismatch:'+key)
        if row['plan_present_at_capture'] is not bool(sequence is not None and sequence<seq):raise ValueError('invalid_plan_capture_order')
        if row['candidate_id'] is not None:
            if not row['plan_present_at_capture']:raise ValueError('candidate_before_plan')
            candidates=[p for p in plan['paths'] if p['candidate_id']==row['candidate_id']]
            if len(candidates)!=1:raise ValueError('missing_plan_candidate')
            _same(row['candidate_snapshot'],candidates[0],'producer_candidate_plan_mismatch')
        ri=e['rollout_inputs'].get(eid)
        if option.get('rollout_attempted'):
            if ri is None or type(ri['capture_sequence']) is not int or ri['capture_sequence']<=seq:
                raise ValueError('missing_pre_rollout_input')
            sequences.append(ri['capture_sequence']);rollout_ids.add(eid)
            _same(ri['xy'],tracked[option['tracking_path_id']]['xy'],'tracking_path_mismatch')
            _same(ri['pp_settings'],option['pp_settings'],'tracking_settings_mismatch')
        elif ri is not None:raise ValueError('fabricated_rollout_input')
    if seen!=set(native) or set(e['rollout_inputs'])!=rollout_ids:raise ValueError('producer_option_count_mismatch')
    if sequence is not None:sequences.append(sequence)
    if not all(type(v) is int and v>0 for v in sequences) or len(sequences)!=len(set(sequences)):
        raise ValueError('invalid_capture_sequence')
    return dict(status='locally_verified_content',evidence_schema=SCHEMA,
        evidence_sha256=digest,producer_time_capture_claim_present=True,
        capture_timing_independently_authenticated=False,new_capture_contract_compliant=True,
        geometry_evidence=geometry_evidence(geometry_manifest_path,expected_manifest_sha256),
        producer_binding_verified=False,code_artifacts_status='recorded_hash_claims_not_reverified',**FLAGS)


def verify_producer_evidence_v1(evidence, record, *, session_metadata,
                                geometry_manifest_path=None, expected_manifest_sha256=None):
    """Strict public validator; malformed evidence cannot fall back to legacy."""
    try:
        return _verify_producer_evidence_v1(evidence,record,session_metadata=session_metadata,
            geometry_manifest_path=geometry_manifest_path,expected_manifest_sha256=expected_manifest_sha256)
    except (KeyError,TypeError,IndexError,StopIteration,AttributeError) as exc:
        raise ValueError('malformed_producer_evidence') from exc


def write_producer_evidence(path, records):
    """Explicit post-step IO. Never overwrites a sidecar or existing evidence."""
    lines=[]
    for r in records:
        if not isinstance(r,dict) or r.get('schema')!=SCHEMA or type(r.get('schema_version')) is not int or r['schema_version']!=1:
            raise ValueError('unsupported_producer_evidence_schema')
        if any(r.get(k) is not v for k,v in FLAGS.items()):raise ValueError('invalid_evidence_flags')
        body={k:v for k,v in r.items() if k!='evidence_sha256'}
        if r.get('evidence_sha256')!=content_digest(SCHEMA,body):raise ValueError('evidence_digest_mismatch')
        lines.append(json.dumps(r,sort_keys=True,allow_nan=False))
    with Path(path).open('x') as stream:
        for line in lines:stream.write(line+'\n')


def verify_legacy_esp_evidence(evidence, record, *, session_metadata,
                               geometry_manifest_path=None, expected_manifest_sha256=None):
    """Read old ESP evidence as legacy claims; never synthesize new capture timing."""
    r=validate_record(record);e=deepcopy(evidence)
    try:
        if (not isinstance(e,dict) or e.get('schema')!='esp_producer_evidence_v1' or type(e.get('schema_version')) is not int or
            e['schema_version']!=1 or e.get('errors')!=[] or e.get('inspection_only') is not True or
            e.get('planning_allowed') is not False or e.get('historical_authentication_verified') is not False):
            raise ValueError('invalid_legacy_evidence')
        _context(session_metadata,None,r,require_runtime=False)  # Supplied claims, not a pre-captured runtime.
        _same(e['source'],r['source'],'legacy_source_mismatch')
        source_hash=snapshot_sha256(r['source']);p=e['plan']
        if p is None or r['plan_snapshot'] is None:raise ValueError('legacy_plan_capture_required')
        if p['capture_phase']!='after_native_generation_before_selector':raise ValueError('legacy_capture_phase')
        if p['source_snapshot_sha256']!=source_hash or p['plan_snapshot_sha256']!=snapshot_sha256(r['plan_snapshot']):
            raise ValueError('legacy_source_plan_digest_mismatch')
        bound=p['plan_binding_v2']
        verify_plan_binding(r['plan_snapshot'],bound,session_metadata=session_metadata,
            length_m=bound['lookahead_length_m'],geometry_manifest_path=geometry_manifest_path,
            expected_manifest_sha256=expected_manifest_sha256)
        if len(e['options'])!=len(r['options']):raise ValueError('legacy_option_count_mismatch')
        seen=set();native={o['evaluation_id']:o for o in r['options']}
        for saved in e['options']:
            event=saved['event'];eid=event['evaluation_id']
            if eid in seen or eid not in native:raise ValueError('duplicate_or_unknown_evaluation_id')
            seen.add(eid);option=native[eid]
            if saved['event_sha256']!=snapshot_sha256(event):raise ValueError('legacy_event_digest_mismatch')
            if event['capture_phase']!='native_start_option_before_rollout':raise ValueError('legacy_capture_phase')
            for key in ('prediction_group_id','candidate_id','reference_id','target_speed_mps'):
                _same(event[key],option[key],'legacy_option_mismatch:'+key)
            if event['source_snapshot_sha256']!=source_hash or event['plan_v2_sha256']!=bound['plan_sha256']:
                raise ValueError('legacy_option_binding_mismatch')
            if event['candidate_snapshot_sha256']!=option['candidate_snapshot_sha256']:
                raise ValueError('legacy_candidate_digest_mismatch')
            group=next(g for g in r['lead_predictions'] if g['prediction_group_id']==option['prediction_group_id'])
            if event['lead_group_sha256']!=snapshot_sha256(group):raise ValueError('legacy_group_digest_mismatch')
        if seen!=set(native):raise ValueError('legacy_option_count_mismatch')
    except (KeyError,TypeError,IndexError,StopIteration) as exc:
        raise ValueError('malformed_legacy_evidence') from exc
    return dict(status='legacy_content_verified',evidence_schema='esp_producer_evidence_v1',
        producer_time_capture_claim_present=True,capture_timing_independently_authenticated=False,
        new_capture_contract_compliant=False,runtime_captured_before_evaluation_verified=False,
        legacy_limitation='Legacy producer hashes before evaluation; cannot satisfy new deferred-digest contract.',
        geometry_evidence=geometry_evidence(geometry_manifest_path,expected_manifest_sha256),
        producer_binding_verified=False,**FLAGS)


def verify_optional_evidence(evidence, record, *, session_metadata, allow_legacy_esp=False,
                              geometry_manifest_path=None, expected_manifest_sha256=None):
    if evidence is None:
        return dict(status='missing',producer_binding_verified=False,**FLAGS)
    kwargs=dict(session_metadata=session_metadata,geometry_manifest_path=geometry_manifest_path,
                expected_manifest_sha256=expected_manifest_sha256)
    if not isinstance(evidence,dict):raise ValueError('invalid_evidence_object')
    if evidence.get('schema')==SCHEMA:
        try:return verify_producer_evidence_v1(evidence,record,**kwargs)
        except (KeyError,TypeError,IndexError,StopIteration) as exc:
            raise ValueError('malformed_producer_evidence') from exc
    if allow_legacy_esp and evidence.get('schema')=='esp_producer_evidence_v1':
        return verify_legacy_esp_evidence(evidence,record,**kwargs)
    raise ValueError('unsupported_evidence_schema_no_legacy_downgrade')
