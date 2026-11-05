"""Export a selected recorded lap for the read-only dashboard.

No simulation or planning is rerun. Smooth wall lines are an explicitly
display-only Gaussian rendering; the original inward-envelope safety band
is exported separately and remains the planner's actual constraint.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml
from scipy.interpolate import BSpline
from scipy.ndimage import gaussian_filter1d
from PIL import Image, ImageDraw, ImageFont

from .core.geometry import load_geometry
from .overtaking.millimetre_domain import MillimetreDomain


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_rows(path):
    return [json.loads(s) for s in Path(path).read_text().splitlines()]


def run(source, output, arm_name=None, baseline_source=None, skip_gif=False):
    source, out = Path(source), Path(output)
    out.mkdir(parents=True, exist_ok=False)
    report = json.loads((source/'report.json').read_text())
    arm = next(a for a in report['arms'] if a['name'] == arm_name) if arm_name else next(a for a in report['arms'] if a['budget_ms'] == 20)
    map_id = report.get('map', 'my_map')
    map_yaml = f'maps/{map_id}.yaml'
    session = source/arm['name']
    metadata = json.loads((session/'metadata.json').read_text())
    summary = json.loads((session/'summary.json').read_text())
    states, actions, plans = [read_rows(session/(name+'.jsonl')) for name in ('states','actions','plans')]
    geometry = load_geometry(f'maps/frenet/{map_id}/geometry', map_yaml, inspect=True)
    domain_dir = Path(metadata['experimental_planner']['domain_dir'])
    domain = MillimetreDomain.load(domain_dir, geometry,
        expected_margin=report['profile']['centre_prefilter_margin_m'])
    arrays, dm = domain.arrays, domain.metadata
    select = np.arange(0, len(arrays['sites']), 20 if map_id == 'esp' else 5)
    def points(values):
        return np.round(values, 6).tolist()
    def band(name):
        return points((arrays['xy']+arrays[name][:,None]*arrays['normal'])[select])
    # Visual wall smoothing does not alter a single stored domain bound.
    sigma_m = .05
    wall_lines = {}
    for name in ('raw_lower','raw_upper'):
        raw = arrays['xy']+arrays[name][:,None]*arrays['normal']
        wall_lines[name] = points(gaussian_filter1d(raw, sigma_m/dm['step_m'], axis=0, mode='wrap')[select])
    raster = geometry.raster
    Image.fromarray(raster.gray).save(out/'map.png')
    action_map = {a['source_control_step']:a for a in actions}
    cycle_map = {c['source_control_step']:c for c in summary['cycle_timings']}
    frames = []
    for state, plan in zip(states, plans):
        step = state['control_step']
        assert plan['source_control_step'] == step
        start = plan['result']['start']
        distance = np.linspace(0., metadata['config']['lookahead_m'], 101 if map_id == 'esp' else 301)
        ref, tangent, _, _ = geometry.curve.sample(start['s']+distance)
        normal = np.column_stack((-tangent[:,1],tangent[:,0]))
        paths = []
        for path in plan['paths']:
            spec = path['spline']
            curve = BSpline(spec['knots'], spec['coefficients'], spec['degree'])
            d = curve(distance)
            route = ('reference' if path['path_type']=='reference_centerline' else
                     'bspline' if path.get('generation_route')=='budgeted_bspline_repair' else 'direct')
            paths.append(dict(id=path['candidate_id'], valid=path['sampled_valid'],
                reason=path['reason'], prior_reason=path.get('prior_direct_reason'),
                route=route, target=path['requested_target_d'], boundary=path['boundary_valid'],
                boundary_check=path['boundary_check'],
                max_curvature=path.get('max_abs_curvature_inv_m'),
                min_jacobian=path.get('min_jacobian'), spec=spec,
                xy=points(ref+d[:,None]*normal), d=points(d),
                repair_validation=path.get('repair_validation')))
        row = int(domain.indices(np.array([start['s']]))[0])
        reference = geometry.reference(start['s'])
        action = action_map.get(step)
        pp = None
        if action:
            target = np.array(action['controller_diagnostic']['lookahead_xy'])
            rel = target-np.array(state['pose'][:2])
            lateral = float(np.dot([-math.sin(state['pose'][2]),math.cos(state['pose'][2])],rel))
            true_distance = float(np.linalg.norm(rel))
            steer = (0. if abs(lateral)<1e-6 else math.atan(2*metadata['vehicle_params']['wheelbase']*lateral/true_distance**2))
            assert abs(steer-action['controller_diagnostic']['unconstrained_steering'])<1e-9
            pp = dict(action=action['action'], **action['controller_diagnostic'],
                      true_distance=true_distance, lateral=lateral, calculated_steering=steer)
        frames.append(dict(step=step, time=state['elapsed_time_s'], pose=state['pose'],
            speed=state['speed_mps'], start=start, e_psi=state['frenet']['e_psi'],
            kappa=reference['kappa'], row=row, lower=float(arrays['lower'][row]),
            upper=float(arrays['upper'][row]), paths=paths, pp=pp,
            valid=[p['id'] for p in paths if p['valid']],
            repair=plan['result']['bspline_repair'],
            checks={k:plan['result'][k] for k in ('explicit_boundary_checks','inherited_boundary_checks',
                    'interval_count','curvature_sample_count','online_rays','online_candidate_inverse_projections',
                    'online_candidate_xy_conversions')},
            plan_ms=plan['wall_time_s']*1000, cycle_ms=cycle_map[step]['wall_time_s']*1000,
            terminal=plan['trigger']=='terminal'))
    assert len(frames)==len(actions)+1 and frames[-1]['terminal']
    src_files = [source/'report.json', session/'metadata.json', session/'summary.json',
                 *[session/(n+'.jsonl') for n in ('states','actions','plans')],
                 domain_dir/'domain.npz', domain_dir/'metadata.json', Path(map_yaml),
                 Path(map_yaml).parent/yaml.safe_load(Path(map_yaml).read_text())['image']]
    comparisons = list(report['arms'])
    if baseline_source:
        baseline_path = Path(baseline_source)/'report.json'
        prior = json.loads(baseline_path.read_text())
        assert prior.get('map', 'my_map') == map_id
        baseline = next(a for a in prior['arms'] if a['budget_ms'] == 20)
        for name in ('states', 'actions'):
            assert digest(Path(baseline_source)/baseline['name']/(name+'.jsonl')) == digest(session/(name+'.jsonl'))
        comparisons.insert(0, baseline)
        src_files.append(baseline_path)
    data = dict(version=3, map_id=map_id, title=f'{map_id} · {arm["budget_ms"]:g} ms 路線規劃實驗', selected_budget_ms=arm['budget_ms'],
        geometry=dict(width=raster.width,height=raster.height,resolution=raster.resolution,
            origin=raster.origin.tolist(),length=domain.length,center=points(arrays['xy'][select]),
            raw_right=band('raw_lower'),raw_left=band('raw_upper'),
            smooth_right=wall_lines['raw_lower'],smooth_left=wall_lines['raw_upper'],
            safe_right=band('lower'),safe_left=band('upper'),
            smooth_wall_policy=dict(sigma_m=sigma_m,mode='periodic Gaussian XY, display only; never used for planning'),
            safety_band_policy='stored inward maximum/minimum envelope, half-window 0.025 m; no outward expansion'),
        config=metadata['config'], vehicle=metadata['vehicle_params'], profile=report['profile'],
        domain=dm, offline_file_bytes=(domain_dir/'domain.npz').stat().st_size,
        chosen={k:v for k,v in arm.items() if k!='session_summary'}, comparisons=[{k:v for k,v in a.items() if k!='session_summary'} for a in comparisons],
        summary={k:v for k,v in summary.items() if k!='cycle_timings'}, frames=frames,
        audits=dict(all_passed=report['all_audits_passed'],
            baseline=report['arms'][0]['original_density_audit'],chosen=arm['original_density_audit'],
            one_mm=[r for r in report['one_mm_repair_audits'] if r['arm']==arm['name']]),
        sources=[dict(path=str(p),sha256=digest(p)) for p in src_files],
        display_sampling_note=f'候選顯示間距 {3 if map_id == "esp" else 1} cm；僅降低繪圖取樣，保留全部記錄步數。原驗證仍使用 1 mm 格與不超過 0.5 mm 曲率點。',
        candidate_control_enabled=False,footprint_verified=False)
    (out/'data.json').write_text(json.dumps(data,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n')
    (out/'report.md').write_text(Path('docs/project/CHANGELOG.md').read_text())
    if not skip_gif:
        make_gif(data,out,raster.gray)
    print(json.dumps(dict(frames=len(frames),output=str(out),data_bytes=(out/'data.json').stat().st_size,
                         gif_bytes=(out/f'{map_id}_{arm["budget_ms"]:g}ms.gif').stat().st_size if not skip_gif else None)))


def make_gif(data, out, gray):
    """Every recorded frame, 100 ms/frame, with a one-second terminal pause."""
    import matplotlib
    font_dir = Path(matplotlib.get_data_path())/'fonts'/'ttf'
    font = ImageFont.truetype(str(font_dir/'DejaVuSans.ttf'),16)
    small = ImageFont.truetype(str(font_dir/'DejaVuSans.ttf'),13)
    title = ImageFont.truetype(str(font_dir/'DejaVuSans-Bold.ttf'),23)
    W,H=1120,710
    gif_name = f'{data["map_id"]}_{data["selected_budget_ms"]:g}ms.gif'
    g=data['geometry'];scale=min(710/(g['width']*g['resolution']),540/(g['height']*g['resolution']))
    mw,mh=g['width']*g['resolution']*scale,g['height']*g['resolution']*scale
    ox,oy=24+(720-mw)/2,88+(540-mh)/2
    def point(p):return (ox+(p[0]-g['origin'][0])*scale,oy+mh-(p[1]-g['origin'][1])*scale)
    base=Image.new('RGB',(W,H),'#f3f5ef');draw=ImageDraw.Draw(base)
    draw.text((25,22),f'{data["map_id"].upper()} / {data["selected_budget_ms"]:g} ms B-SPLINE REPAIR',font=title,fill='#182f32')
    draw.text((25,55),'Recorded lap | 27 x 51 cm | wheelbase 33 cm | steering 29.5 deg',font=small,fill='#536968')
    image=Image.fromarray(gray).convert('RGB').resize((round(mw),round(mh)),Image.Resampling.NEAREST)
    base.paste(image,(round(ox),round(oy)))
    draw=ImageDraw.Draw(base)
    for key,color in [('smooth_left','#bd710f'),('smooth_right','#bd710f'),('safe_left','#008c77'),('safe_right','#008c77'),('center','#6880a1')]:
        line=[point(p) for p in g[key]];draw.line(line+[line[0]],fill=color,width=2)
    draw.text((25,645),'Orange: smooth walls (display only)   Green: actual safe-band envelope',font=small,fill='#244443')
    draw.text((25,670),'PP tracks C3 only. Body-edge 5 cm clearance / candidate tracking not verified.',font=small,fill='#9f493d')
    colors=['#3478bc','#bd8300','#008b72','#a44bc7','#d45848']
    rendered=[]
    for f in data['frames']:
        im=base.copy();d=ImageDraw.Draw(im)
        for p in f['paths']:
            if p['valid']:d.line([point(v) for v in p['xy']],fill=colors[p['id']-1],width=3)
        x,y,yaw=f['pose'];length=data['profile']['vehicle_length_m'];width=data['profile']['vehicle_width_m']
        corners=[]
        for a,b in [(1,1),(1,-1),(-1,-1),(-1,1)]:
            corners.append(point([x+a*length/2*math.cos(yaw)-b*width/2*math.sin(yaw),y+a*length/2*math.sin(yaw)+b*width/2*math.cos(yaw)]))
        d.polygon(corners,fill='#203c4b',outline='#ffffff')
        if f['pp']:
            target=point(f['pp']['lookahead_xy']);car=point([x,y]);d.line([car,target],fill='#2a7eaf',width=2)
            d.ellipse((target[0]-4,target[1]-4,target[0]+4,target[1]+4),fill='#157bcb')
        d.text((775,103),f"STEP {f['step']:03d} / {len(data['frames'])-1}",font=title,fill='#163d37')
        lines=[f"Elapsed {f['time']:.1f} s",f"s = {f['start']['s']:.3f} m",f"d = {f['start']['d']*100:.2f} cm",f"speed = {f['speed']:.2f} m/s",'',
            f"Plan: {f['plan_ms']:.2f} ms",f"Full cycle: {f['cycle_ms']:.2f} ms",f"Valid IDs: {', '.join(map(str,f['valid']))}",
            f"Repair: {f['repair']['repaired_ids']}",f"Timed out: {f['repair']['timed_out_ids']}",
            f"Steer: {math.degrees(f['pp']['action'][0]):+.2f} deg" if f['pp'] else 'Terminal: no next command','']
        yy=145
        for line in lines:d.text((775,yy),line,font=font,fill='#294c4a');yy+=26
        for p in f['paths']:
            label=f"C{p['id']} {'PASS' if p['valid'] else 'REJECT'} / {p['route']}"
            d.text((775,yy),label,font=small,fill=colors[p['id']-1]);yy+=24
        rendered.append(im)
    durations=[100]*len(rendered);durations[-1]=1000
    # Shared palette keeps the raster and annotation colors stable across frames.
    palette=rendered[34].quantize(colors=192,method=Image.MEDIANCUT)
    quantized=[im.quantize(palette=palette,dither=Image.NONE) for im in rendered]
    quantized[0].save(out/gif_name,save_all=True,append_images=quantized[1:],
                      duration=durations,loop=0,optimize=False,disposal=2)
    rendered[34].save(out/'gif_preview.png')
    with Image.open(out/gif_name) as check:
        assert check.n_frames==len(data['frames'])
    (out/'gif_manifest.json').write_text(json.dumps(dict(frames=len(rendered),size=[W,H],
        frame_duration_ms=100,terminal_duration_ms=1000,loop=True,
        gif_sha256=digest(out/gif_name),source=f'same {len(data["frames"])} recorded states as data.json'),indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',required=True)
    parser.add_argument('--output-dir',required=True)
    parser.add_argument('--arm')
    parser.add_argument('--baseline-source')
    parser.add_argument('--skip-gif',action='store_true')
    args=parser.parse_args()
    run(args.source,args.output_dir,args.arm,args.baseline_source,args.skip_gif)
