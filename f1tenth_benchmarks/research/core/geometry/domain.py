"""Stage C sampled domain sidecar and exact per-query source-domain checks."""
import hashlib
import json
from pathlib import Path

import numpy as np

def json_bytes(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),
                      ensure_ascii=False,allow_nan=False).encode('utf-8')


class DomainAtlas:
    """Sampled runs are inspection guides; membership is always recomputed."""
    def __init__(self,geometry,data):
        if data.get('schema_version') != 1 or data.get('geometry_id') != geometry.geometry_id:
            raise ValueError('domain geometry_mismatch')
        if data.get('domain_verified') is not False or data.get('planning_allowed') is not False:
            raise ValueError('Stage C sidecar cannot approve planning')
        self.geometry,self.data = geometry,data

    def query(self,s,d):
        result = self.geometry.to_cartesian(s,d)
        return dict(result,domain_model_available=True,
                    domain_membership_method='exact_source_query',domain_verified=False,
                    planning_allowed=False)

    def section(self,s):
        """Exact road limits and necessary J bounds, not unique-projection proof."""
        ref = self.geometry.reference(s)
        b = self.geometry.boundaries(s)
        if not b['valid']:
            return dict(b,valid=False)
        lower,upper = b['d_right'],b['d_left']
        if ref['kappa']>0:
            upper = min(upper,.8/ref['kappa'])
        elif ref['kappa']<0:
            lower = max(lower,.8/ref['kappa'])
        return dict(b,necessary_d_min=lower,necessary_d_max=upper,
                    unique_projection_verified=False)


def build_domain(geometry,sections=512,d_step_m=.1,extra_s=(),progress=None):
    if not geometry.curve.closed:raise ValueError('open reference: use section_ranges instead of closed Stage C atlas')
    if not isinstance(sections,int) or sections<4 or not np.isfinite(d_step_m) or d_step_m<=0:
        raise ValueError('invalid domain sampling options')
    positions = list(np.linspace(0,geometry.curve.L,sections,endpoint=False))
    positions += [1e-6,geometry.curve.L-1e-6]+list(extra_s)
    if not np.isfinite(positions).all():
        raise ValueError('nonfinite domain section')
    positions = sorted(set(float(s%geometry.curve.L) for s in positions))
    data = dict(schema_version=1,stage='C',geometry_id=geometry.geometry_id,
                frame_id='map',domain_verified=False,planning_allowed=False,
                membership_method='exact_source_query',s_interpolation_allowed=False,
                d_interpolation_allowed=False,sampling=dict(sections=sections,d_step_m=d_step_m,
                extra_sections=len(extra_s)),sections=[],
                limitations=['Sampled runs do not certify points between samples',
                             'Stage D dense/adaptive validation remains pending',
                             'Physical Cartesian track boundaries are preserved'])
    atlas = DomainAtlas(geometry,data)
    for index,s in enumerate(positions):
        boundary = atlas.section(s)
        if not boundary['valid']:
            data['sections'].append(dict(s_m=s,reason=boundary['reason'],samples=[],sampled_runs=[]))
            continue
        epsilon = max(1e-6,geometry.raster.resolution*1e-5)
        lower,upper = boundary['d_right']+epsilon,boundary['d_left']-epsilon
        values = np.linspace(lower,upper,max(2,int(np.ceil((upper-lower)/d_step_m))+1)).tolist()
        values += [0.,boundary['necessary_d_min']+epsilon,boundary['necessary_d_max']-epsilon]
        values = sorted(set(float(d) for d in values if lower<=d<=upper))
        samples = []
        for d in values:
            result = atlas.query(s,d)
            samples.append(dict(d_m=d,valid=result['valid'],reason=result['reason']))
        runs = [];current=[]
        for sample in samples:
            if sample['valid']:
                current.append(sample['d_m'])
            elif current:
                runs.append(dict(d_min_m=current[0],d_max_m=current[-1],continuous_verified=False));current=[]
        if current:
            runs.append(dict(d_min_m=current[0],d_max_m=current[-1],continuous_verified=False))
        data['sections'].append(dict(s_m=s,road_d_min=boundary['d_right'],road_d_max=boundary['d_left'],
            necessary_d_min=boundary['necessary_d_min'],necessary_d_max=boundary['necessary_d_max'],
            samples=samples,sampled_runs=runs))
        if progress is not None and (index%64==0 or index==len(positions)-1):
            progress(dict(completed=index+1,total=len(positions)))
    return atlas


def save_domain(atlas,directory):
    """Exclusive sidecar; never edit the hashed Stage B validity.json."""
    root = Path(directory);root.mkdir(parents=True,exist_ok=False)
    payload = json_bytes(atlas.data)
    identifier = hashlib.sha256(payload).hexdigest()
    (root/'validity.json').write_bytes(payload)
    (root/'manifest.json').write_bytes(json_bytes(dict(schema_version=1,stage='C',
        geometry_id=atlas.geometry.geometry_id,domain_id=identifier,
        file_hashes={'validity.json':identifier},planning_allowed=False)))
    return identifier


def load_domain(directory,geometry):
    root=Path(directory);manifest=json.loads((root/'manifest.json').read_text())
    payload=(root/'validity.json').read_bytes();digest=hashlib.sha256(payload).hexdigest()
    if (manifest.get('schema_version')!=1 or manifest.get('stage')!='C'
        or manifest.get('geometry_id')!=geometry.geometry_id or manifest.get('planning_allowed') is not False
        or manifest.get('file_hashes')!={'validity.json':digest} or manifest.get('domain_id')!=digest):
        raise ValueError('domain geometry_mismatch or hash mismatch')
    return DomainAtlas(geometry,json.loads(payload))
