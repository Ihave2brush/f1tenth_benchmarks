"""Small synthetic saved-format fixtures; no prediction, optimizer or simulation."""
from copy import deepcopy
import hashlib
import json


def inputs():
    paths = [dict(candidate_id=i, sampled_valid=True, reason=None, geometry_id='g', frame_id='map',
        points=[dict(x=0., y=0., yaw=0., valid=True, t=0.),
                dict(x=10., y=0., yaw=0., valid=True, t=1.)]) for i in range(1, 6)]
    plan = dict(plan_id=7, status='completed', paths=paths, source_pose=[0., 0., 0.],
                source_sim_time_s=10., available_sim_time_s=99., source_control_step=4, geometry_id='g')
    return plan, dict(session_metadata=dict(geometry_id='g',frame_id='map'))


def analytic_input():
    p,o=inputs();p['generator_settings']=dict(lookahead_m=3.)
    for c in p['paths']:
        c.update(points=[deepcopy(c['points'][0])],output_representation='analytic_spline_and_single_start_point',
                 reference_s_origin=0.,spline=dict(knots=[0.]*4+[3.]*4,coefficients=[0.,0.,.1,.1],degree=3))
    return p,o['session_metadata']


def artifact(tmp_path):
    root=tmp_path/'geometry';root.mkdir()
    (root/'reference_curve.json').write_text('{"synthetic":true}')
    sha=hashlib.sha256((root/'reference_curve.json').read_bytes()).hexdigest()
    m=dict(schema_version=1,map_id='synthetic',geometry_id='g',frame_id='map',
           file_hashes={'reference_curve.json':sha},source_hashes={'image':'unverified_claim'})
    path=root/'manifest.json';path.write_text(json.dumps(m))
    return path,hashlib.sha256(path.read_bytes()).hexdigest()
