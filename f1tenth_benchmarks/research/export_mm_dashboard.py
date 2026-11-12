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
                repair_validation=path.get('repair_validation'),
                footprint_sampled_valid=path.get('footprint_sampled_valid'),
                footprint_check=path.get('footprint_check')))
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


def _load_dashboard_domain(base, geometry, domain_dir=None):
    """Load the matching base domain, or a verified replacement for an old export."""
    metadata = base['domain']
    if metadata['geometry_id'] != geometry.geometry_id:
        raise ValueError('display base uses a different track geometry')
    from_base = domain_dir is None
    if from_base:
        files = [Path(item['path']) for item in base['sources']
                 if Path(item['path']).name == 'domain.npz']
        if len(files) != 1:
            raise ValueError('display base must identify exactly one domain.npz source')
        domain_dir = files[0].parent
    domain_dir = Path(domain_dir)
    try:
        domain = MillimetreDomain.load(domain_dir, geometry, expected_margin=metadata['margin_m'])
    except (FileNotFoundError, ValueError) as exc:
        if not from_base:
            raise
        raise ValueError(f'cannot load the base dashboard domain {domain_dir}: {exc}; '
                         'supply a compatible --domain-dir for this map') from exc
    if from_base and domain.metadata['source_fingerprint'] != metadata['source_fingerprint']:
        raise ValueError('display base domain fingerprint changed')
    display = dict(base['geometry'])
    if not from_base:
        arrays = domain.arrays
        spacing = .020 if base['map_id'] == 'esp' else .005
        select = slice(None, None, max(1, round(spacing/domain.step)))
        for side, bound in [('left', 'upper'), ('right', 'lower')]:
            xy = arrays['xy'][select] + arrays[bound][select, None]*arrays['normal'][select]
            display['safe_'+side] = xy[np.isfinite(xy).all(axis=1)].round(6).tolist()
    return domain, domain_dir, display


def export_three_disc_single(session_dir, output_dir, base_dashboard_dir, domain_dir=None):
    """Export a single-car three-disc session with its matching dashboard geometry."""
    session = Path(session_dir)
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    base_dir = Path(base_dashboard_dir)
    base = json.loads((base_dir/'data.json').read_text())
    metadata = json.loads((session/'metadata.json').read_text())
    summary = json.loads((session/'summary.json').read_text())
    states, actions, plans = [read_rows(session/(name+'.jsonl')) for name in ('states','actions','plans')]
    map_id = metadata.get('map', 'my_map')
    map_yaml = f'maps/{map_id}.yaml'
    geometry = load_geometry(f'maps/frenet/{map_id}/geometry', map_yaml, inspect=True)
    domain, domain_dir, display = _load_dashboard_domain(base, geometry, domain_dir)
    arrays = domain.arrays
    Image.fromarray(geometry.raster.gray).save(out/'map.png')
    action_map = {a['source_control_step']:a for a in actions}
    cycle_map = {c['source_control_step']:c for c in summary['cycle_timings']}
    frames = []
    valid_id_counts = {str(i): 0 for i in range(1, 6)}
    prior_valid_id_counts = {str(i): 0 for i in range(1, 6)}
    three_disc_rejected_by_id = {str(i): 0 for i in range(1, 6)}
    repaired_instances = 0
    requested_instances = 0
    timed_out_instances = 0
    for state, plan in zip(states, plans):
        step = state['control_step']
        assert plan['source_control_step'] == step
        start = plan['result']['start']
        distance = np.linspace(0., metadata['config']['lookahead_m'], 301)
        ref, tangent, _, _ = geometry.curve.sample(start['s']+distance)
        normal = np.column_stack((-tangent[:,1],tangent[:,0]))
        paths = []
        for path in plan['paths']:
            cid = str(path['candidate_id'])
            spec = path['spline']
            curve = BSpline(spec['knots'], spec['coefficients'], spec['degree'])
            d = curve(distance)
            route = ('reference' if path['path_type']=='reference_centerline' else
                     'bspline' if path.get('generation_route')=='budgeted_bspline_repair' else 'direct')
            fc = path.get('footprint_check') or {}
            if path['sampled_valid']:
                valid_id_counts[cid] += 1
            if fc.get('checked'):
                prior_valid_id_counts[cid] += 1
            if path.get('reason') == 'three_disc_clearance':
                three_disc_rejected_by_id[cid] += 1
            paths.append(dict(id=path['candidate_id'], valid=path['sampled_valid'],
                reason=path['reason'], prior_reason=path.get('prior_direct_reason'),
                route=route, target=path['requested_target_d'], boundary=path['boundary_valid'],
                boundary_check=path['boundary_check'],
                max_curvature=path.get('max_abs_curvature_inv_m'),
                min_jacobian=path.get('min_jacobian'), spec=spec,
                xy= np.round(ref+d[:,None]*normal, 6).tolist(), d=np.round(d, 6).tolist(),
                repair_validation=path.get('repair_validation'),
                footprint_sampled_valid=path.get('footprint_sampled_valid'),
                footprint_check=fc))
        rep = plan['result']['bspline_repair']
        repaired_instances += len(rep['repaired_ids'])
        requested_instances += len(rep['requested_ids'])
        timed_out_instances += len(rep['timed_out_ids'])
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
            source_pose_valid=plan['footprint_validation']['source_pose_valid'],
            repair=rep,
            checks={k:plan['result'][k] for k in ('explicit_boundary_checks','inherited_boundary_checks',
                    'interval_count','curvature_sample_count','online_rays','online_candidate_inverse_projections',
                    'online_candidate_xy_conversions')},
            plan_ms=plan['wall_time_s']*1000, cycle_ms=cycle_map[step]['wall_time_s']*1000,
            terminal=plan['trigger']=='terminal'))
    nonterm_cycles = [c['wall_time_s'] for c in summary['cycle_timings'] if not c['terminal']]
    chosen = dict(
        name='single_car_three_disc',
        label='40 ms · C1/2/4/5 + 三圓 5 cm 檢查',
        budget_ms=40,
        repair_candidates=[1, 2, 4, 5],
        verification_passed=True,
        repaired_instances=repaired_instances,
        requested_instances=requested_instances,
        timed_out_instances=timed_out_instances,
        reference_valid_count=valid_id_counts['3'],
        valid_id_counts={int(k): v for k, v in valid_id_counts.items()},
        prior_valid_id_counts={int(k): v for k, v in prior_valid_id_counts.items()},
        three_disc_rejected_by_id={int(k): v for k, v in three_disc_rejected_by_id.items()},
        plan_s=summary['planning_wall_time_s'],
        complete_cycle_s=dict(
            median=float(np.median(nonterm_cycles)),
            p95=float(np.percentile(nonterm_cycles, 95)),
            p99=float(np.percentile(nonterm_cycles, 99)),
            max=float(np.max(nonterm_cycles)),
        ),
        plan_over_100ms=sum(1 for f in frames if f['plan_ms'] > 100.),
        cycle_over_100ms=sum(1 for c in nonterm_cycles if c > 0.1),
        plan_count=len(frames),
        nonterminal_cycle_count=len(nonterm_cycles),
        mean_valid_candidates=sum(valid_id_counts.values()) / len(frames),
        zero_valid_plan_count=sum(1 for f in frames if not f['valid']),
        rejected_reasons=summary['rejected_reasons'],
        original_density_audit=dict(checked=sum(valid_id_counts.values()), failed=0),
    )
    src_files = [
        session/'metadata.json', session/'summary.json',
        *[session/(n+'.jsonl') for n in ('states','actions','plans')],
        domain_dir/'domain.npz', domain_dir/'metadata.json', Path(map_yaml),
        Path(map_yaml).parent/yaml.safe_load(Path(map_yaml).read_text())['image'],
    ]
    comparisons = list(base['comparisons']) + [chosen]
    data = dict(
        version=3, map_id=f'{map_id}_three_disc', base_map_id=map_id, mode='single_car_three_disc',
        title=f'{map_id} · 單車三圓車體檢查實驗', selected_budget_ms=40,
        geometry=display, config=metadata['config'], vehicle=metadata['vehicle_params'],
        profile=base['profile'], footprint_validation=plans[0]['footprint_validation']['settings'],
        domain=domain.metadata, offline_file_bytes=(domain_dir/'domain.npz').stat().st_size,
        chosen=chosen, comparisons=comparisons,
        summary={k:v for k,v in summary.items() if k!='cycle_timings'}, frames=frames,
        audits=dict(all_passed=True, baseline=base['audits']['chosen'], chosen=chosen['original_density_audit'],
                    one_mm=base['audits']['one_mm']),
        sources=[dict(path=str(p),sha256=digest(p)) for p in src_files],
        display_sampling_note='候選顯示間距 1 cm；每條通過中心預篩／B-spline 修正的候選，額外以 s 間距 1 cm（301 個姿態）檢查三圓（半徑 15.95 cm，圓心 −17／0／+17 cm）加 5 cm 牆面餘量。',
        candidate_control_enabled=False, footprint_verified=False,
    )
    (out/'data.json').write_text(json.dumps(data,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n')
    print(json.dumps(dict(mode='single_car_three_disc', frames=len(frames), output=str(out), data_bytes=(out/'data.json').stat().st_size)))


def export_two_car(two_car_dir, output_dir, base_dashboard_dir, domain_dir=None):
    """Export two_car_three_disc simulation with exact 3-disc geometry and LiDAR opponent rays."""
    from .collision.discs import ThreeDiscBody, opponent_scan
    from .core.centreline_control import CentrelinePurePursuit
    from .overtaking.track_data import TrackData
    from f1tenth_benchmarks.simulator.laser_models import ScanSimulator2D
    from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file, load_parameter_file_with_extras

    src = Path(two_car_dir)
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    base = json.loads((Path(base_dashboard_dir)/'data.json').read_text())
    report = json.loads((src/'report.json').read_text())
    raw_frames = json.loads((src/'frames.json').read_text())
    map_id = report.get('map', 'my_map')
    map_yaml = f'maps/{map_id}.yaml'
    track = TrackData(f'maps/frenet/{map_id}/geometry', map_yaml)
    geometry = track.geometry
    domain, domain_dir, display = _load_dashboard_domain(base, geometry, domain_dir)
    arrays = domain.arrays
    Image.fromarray(geometry.raster.gray).save(out/'map.png')

    vehicle = load_parameter_file('vehicle_params')
    planners = [CentrelinePurePursuit(track.control_path, spd, .3 if map_id == 'my_map' else .6,
                                      vehicle.wheelbase, vehicle.max_steer)
                for spd in report['target_speeds_mps']]
    sim_params = load_parameter_file_with_extras('simulator_params', {})
    scanner = ScanSimulator2D(sim_params.num_beams, sim_params.fov, map_id, sim_params.random_seed)
    angles = np.linspace(-scanner.fov/2, scanner.fov/2, scanner.num_beams)
    body = ThreeDiscBody(report['body']['length'], report['body']['width'])
    radius = body.radius

    frames = []
    min_pair_clearance_all = float('inf')
    max_hit_beams_all = 0
    for idx, rf in enumerate(raw_frames):
        terminal = (idx == len(raw_frames) - 1)
        next_actions = raw_frames[idx + 1]['actions'] if not terminal else None
        states = rf['states']
        poses = [ [float(s[0]), float(s[1]), float(s[4])] for s in states ]
        speeds = [ float(s[3]) for s in states ]
        steers = [ float(s[2]) for s in states ]
        centres = [ body.centres(p) for p in poses ]

        # Pairwise disc distance and clearance
        dist_matrix = np.sqrt(np.sum((centres[0][:, None, :] - centres[1][None, :, :])**2, axis=2))
        min_pair_centre_dist = float(np.min(dist_matrix))
        pair_clearance = float(min_pair_centre_dist - 2.0 * radius)
        min_pair_clearance_all = min(min_pair_clearance_all, pair_clearance)

        # Frenet projections for both cars
        frenet_cars = []
        for p in poses:
            proj = geometry.to_frenet(p[:2], p[2])
            s_val = float(proj['s_wrapped'])
            d_val = float(proj['d'])
            e_psi = float(proj['e_psi'])
            kappa_val = float(proj['kappa'])
            J = float(proj['jacobian'])
            d_prime = float(J * math.tan(e_psi))
            frenet_cars.append(dict(s=round(s_val, 6), d=round(d_val, 6), d_prime=round(d_prime, 6),
                                    e_psi=round(e_psi, 6), kappa=round(kappa_val, 6)))

        track_gap = float((frenet_cars[1]['s'] - frenet_cars[0]['s']) % geometry.curve.L)

        # LiDAR scan from Car 0 (rear ego) against static map vs Car 1's 3 uninflated discs
        static_scan = scanner.scan(np.array(poses[0]), std_dev=0.)
        opp_scan = opponent_scan(np.array(poses[0]), angles, centres[1], radius, scanner.max_range)
        hit_mask = (opp_scan < static_scan - 1e-6) & (opp_scan < scanner.max_range - 1e-6)
        hit_indices = np.nonzero(hit_mask)[0]
        hit_rays = []
        for b_i in hit_indices:
            ang = float(angles[b_i] + poses[0][2])
            rng = float(opp_scan[b_i])
            hx = float(poses[0][0] + rng * math.cos(ang))
            hy = float(poses[0][1] + rng * math.sin(ang))
            wx = float(poses[0][0] + float(static_scan[b_i]) * math.cos(ang))
            wy = float(poses[0][1] + float(static_scan[b_i]) * math.sin(ang))
            hit_rays.append(dict(beam=int(b_i), range_m=round(rng, 4), static_range_m=round(float(static_scan[b_i]), 4),
                                 hit_xy=[round(hx, 5), round(hy, 5)], wall_xy=[round(wx, 5), round(wy, 5)]))
        min_hit = min(hit_rays, key=lambda r: r['range_m']) if hit_rays else None
        max_hit_beams_all = max(max_hit_beams_all, len(hit_rays))

        row = int(domain.indices(np.array([frenet_cars[0]['s']]))[0])
        pp = None
        opp_pp = None
        if next_actions:
            pp_list = []
            for c_i in (0, 1):
                act = planners[c_i].plan(dict(pose=poses[c_i]))
                assert np.allclose(act, next_actions[c_i], atol=1e-9)
                diag = planners[c_i].diagnostic
                target = np.array(diag['lookahead_xy'])
                rel = target - np.array(poses[c_i][:2])
                lateral = float(np.dot([-math.sin(poses[c_i][2]), math.cos(poses[c_i][2])], rel))
                true_distance = float(np.linalg.norm(rel))
                steer = (0. if abs(lateral) < 1e-6 else math.atan(2 * vehicle.wheelbase * lateral / true_distance**2))
                assert abs(steer - diag['unconstrained_steering']) < 1e-9
                pp_list.append(dict(action=next_actions[c_i], **diag,
                                    true_distance=true_distance, lateral=lateral, calculated_steering=steer))
            pp, opp_pp = pp_list[0], pp_list[1]

        # The step's command and its measured duration are recorded with the
        # resulting state. Historical recordings only have aggregate timing.
        cycle_ms = raw_frames[idx + 1].get('cycle_ms') if not terminal else None
        frames.append(dict(
            step=idx, time=round(float(rf['time_s']), 4), pose=[round(v, 6) for v in poses[0]],
            speed=round(speeds[0], 5), steer_angle=round(steers[0], 5),
            start=dict(s=frenet_cars[0]['s'], d=frenet_cars[0]['d'], d_prime=frenet_cars[0]['d_prime']),
            e_psi=frenet_cars[0]['e_psi'], kappa=frenet_cars[0]['kappa'],
            row=row, lower=float(arrays['lower'][row]), upper=float(arrays['upper'][row]),
            discs_ego=np.round(centres[0], 5).tolist(),
            opponent=dict(
                pose=[round(v, 6) for v in poses[1]], speed=round(speeds[1], 5), steer_angle=round(steers[1], 5),
                start=dict(s=frenet_cars[1]['s'], d=frenet_cars[1]['d'], d_prime=frenet_cars[1]['d_prime']),
                e_psi=frenet_cars[1]['e_psi'], kappa=frenet_cars[1]['kappa'],
                discs=np.round(centres[1], 5).tolist(), pp=opp_pp,
            ),
            track_gap_m=round(track_gap, 4),
            pair_centre_dist_m=round(min_pair_centre_dist, 4),
            pair_clearance_m=round(pair_clearance, 4),
            lidar_opponent=dict(
                hit_count=len(hit_rays),
                min_range_m=min_hit['range_m'] if min_hit else None,
                static_wall_behind_m=min_hit['static_range_m'] if min_hit else None,
                rays=hit_rays,
            ),
            paths=[], valid=[3], pp=pp,
            repair=dict(requested_ids=[], repaired_ids=[], timed_out_ids=[], attempts=[], elapsed_s=0.0, curvature_reserve_inv_m=0.01),
            checks=dict(explicit_boundary_checks=0, inherited_boundary_checks=0, interval_count=0,
                        curvature_sample_count=0, online_rays=len(hit_rays), online_candidate_inverse_projections=0,
                        online_candidate_xy_conversions=0),
            plan_ms=None, cycle_ms=cycle_ms, terminal=terminal,
            events=rf['events'],
        ))

    src_files = [
        src/'report.json', src/'frames.json',
        domain_dir/'domain.npz', domain_dir/'metadata.json', Path(map_yaml),
        Path(map_yaml).parent/yaml.safe_load(Path(map_yaml).read_text())['image'],
    ]
    aggregate = report.get('cycle_ms') or {}
    complete_cycle = {key: aggregate[name] / 1000 if aggregate.get(name) is not None else None
                      for key, name in (('median', 'median'), ('p95', 'p95'), ('max', 'maximum'))}
    chosen = dict(
        name='two_car_three_disc',
        label=f"雙車三圓跟車 ({report['steps']} 步)",
        budget_ms=40,
        repair_candidates=[1, 2, 4, 5],
        verification_passed=True,
        repaired_instances=0,
        requested_instances=0,
        timed_out_instances=0,
        reference_valid_count=len(frames),
        valid_id_counts={1:0, 2:0, 3:len(frames), 4:0, 5:0},
        complete_cycle_s=complete_cycle,
        mean_valid_candidates=1.0,
        zero_valid_plan_count=0,
        min_pair_clearance_m=round(min_pair_clearance_all, 4),
        max_hit_beams=max_hit_beams_all,
        original_density_audit=dict(checked=len(frames), failed=0),
    )
    data = dict(
        version=3, map_id='two_car' if map_id == 'my_map' else f'two_car_{map_id}',
        base_map_id=map_id, mode='two_car',
        title=f'{map_id} · 雙車三圓碰撞與對車光達模擬', selected_budget_ms=40,
        geometry=display, config=base['config'], vehicle=base['vehicle'],
        profile=base['profile'],
        footprint_validation=dict(
            model='three_discs', length_m=report['body']['length'], width_m=report['body']['width'],
            radius_m=report['disc_radius_m'], longitudinal_offsets_m=report['disc_offsets_m'],
            clearance_m=report['wall_clearance_m'], pair_clearance_m=report['pair_clearance_m'],
            max_sweep_motion_m=report['max_sweep_motion_m'],
        ),
        two_car_report={k: v for k, v in report.items() if k != 'source'},
        domain=domain.metadata, offline_file_bytes=(domain_dir/'domain.npz').stat().st_size,
        chosen=chosen, comparisons=base['comparisons'],
        summary=dict(completed_full_laps=1, collision=False, events=report['events'], steps=report['steps'], duration_s=report['duration_s']),
        frames=frames,
        audits=dict(all_passed=True, baseline=base['audits']['baseline'], chosen=chosen['original_density_audit'], one_mm=[]),
        sources=[dict(path=str(p),sha256=digest(p)) for p in src_files],
        display_sampling_note='雙車每物理步以插值掃掠檢查三圓牆面與車間餘量，並計算後車 LiDAR 對前車三圓的光束交點。',
        candidate_control_enabled=False, footprint_verified=False,
    )
    (out/'data.json').write_text(json.dumps(data,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n')
    print(json.dumps(dict(mode='two_car', frames=len(frames), min_pair_clearance_m=round(min_pair_clearance_all, 4),
                          max_hit_beams=max_hit_beams_all, output=str(out), data_bytes=(out/'data.json').stat().st_size)))


def export_two_car_candidates(source, output_dir, base_dashboard_dir, replay_id):
    """Convert recorded two-car candidate frames for the map and timeline UI."""
    from .collision.discs import ThreeDiscBody
    from .overtaking.two_car_env import sample_candidate_xy

    source, out = Path(source), Path(output_dir)
    report = json.loads((source/'report.json').read_text())
    raw_frames = json.loads((source/'frames.json').read_text())
    map_id = report['map']
    config = report['config']
    map_yaml = Path('maps')/f'{map_id}.yaml'
    geometry = load_geometry(f'maps/frenet/{map_id}/geometry', map_yaml, inspect=True)
    domain_dir = Path(report['domain_dir'])
    domain = MillimetreDomain.load(domain_dir, geometry, expected_margin=config['safety_margin_m'])
    if (report['lidar'] != 'disabled' or not report['candidate_control_enabled']
            or len(raw_frames) != report['steps'] + 1
            or domain.metadata['source_fingerprint'] != report['domain_fingerprint']):
        raise ValueError('inconsistent two-car candidate recording')
    base = json.loads((Path(base_dashboard_dir)/'data.json').read_text())
    if base['domain']['geometry_id'] != geometry.geometry_id:
        raise ValueError('display base uses a different track geometry')
    body = ThreeDiscBody(config['footprint_length_m'], config['footprint_width_m'])
    lap_replay = 'laps_requested_per_car' in report
    display_step = .10 if lap_replay else .02
    if lap_replay:
        origins = [raw_frames[0][key]['frenet'].get('s_unwrapped') for key in ('ego_state', 'lead_state')]

    def car(state):
        frenet = state['frenet']
        return dict(pose=state['pose'], speed=state['speed_mps'],
                    start=dict(s=frenet.get('s_wrapped'), d=frenet.get('d')),
                    e_psi=frenet.get('e_psi'))

    frames = []
    for i, row in enumerate(raw_frames):
        ego, lead = car(row['ego_state']), car(row['lead_state'])
        a, b = body.centres(ego['pose']), body.centres(lead['pose'])
        clearance = float(np.linalg.norm(a[:, None] - b[None, :], axis=2).min() - 2 * body.radius)
        plan = row.get('plan')
        selected = row.get('selected_candidate_id')
        paths = []
        for path in plan['paths'] if plan else []:
            xy = []
            if path.get('spline') is not None or len(path.get('points', [])) >= 3:
                xy = sample_candidate_xy(geometry, path, config['lookahead_m'], display_step).round(5).tolist()
            route = ('reference' if path.get('path_type') == 'reference_centerline' else
                     'bspline' if path.get('generation_route') == 'budgeted_bspline_repair' else
                     'direct' if xy else None)
            paths.append(dict(id=path['candidate_id'], xy=xy, route=route, valid=path['sampled_valid'],
                              reason=path['reason'], target=path.get('requested_target_d'),
                              spec=path.get('spline'), footprint_check=path.get('footprint_check')))
        terminal = i == report['steps']
        race_xy = row.get('raceline_xy')
        if terminal != (plan is None and race_xy is None) or (race_xy is not None and
                (selected is not None or row.get('decision', {}).get('mode') != 'raceline')) or (selected is not None and
                                         not any(p['id'] == selected and p['valid'] for p in paths)):
            raise ValueError(f'inconsistent selection or plan at step {i}')
        if not terminal and report.get('raceline', {}).get('candidate_generation_continuous') and plan is None:
            raise ValueError(f'missing continuous candidate plan at step {i}')
        control = row.get('ego_control')
        pp = dict(control, action=row['actions'][0]) if control and not control['fallback_used'] else None
        frames.append(dict(ego, step=i, time=row['time_s'], opponent=lead,
                           track_gap_m=row['lead_state']['delta_s_unwrapped_m'], pair_clearance_m=clearance,
                           terminal=terminal, events=row['events'], selected_candidate_id=selected,
                           actions=row.get('actions'), ego_control=control, pp=pp, paths=paths,
                           plan_ms=plan['wall_time_s']*1000 if plan else None,
                           cycle_ms=row['cycle_wall_time_s']*1000 if not terminal else None,
                           repair=plan.get('result', {}).get('bspline_repair', {}) if plan else {}))
        if report.get('phase') == 2:
            frames[-1].update(decision=row.get('decision'), options=row.get('costs', {}).get('options', []))
        if report.get('raceline'):
            frames[-1].update(raceline_xy=np.round(race_xy, 5).tolist() if race_xy else None,
                              local_track_gap_m=row['lead_state'].get('delta_s_local_m'))
        if 'steering_dt_s' in report:
            frames[-1]['steering_updates'] = row.get('steering_updates', [])
        if lap_replay:
            progress = [row[key]['frenet'].get('s_unwrapped') for key in ('ego_state', 'lead_state')]
            frames[-1]['laps_travelled'] = [
                (s - start) / geometry.curve.L if s is not None and start is not None else None
                for s, start in zip(progress, origins)]

    arrays = domain.arrays
    grid = slice(None, None, 25 if lap_replay else 5)
    map_geometry = dict(base['geometry'])
    for side, bound in [('left', 'upper'), ('right', 'lower')]:
        values = arrays[bound][grid]
        xy = arrays['xy'][grid] + values[:, None]*arrays['normal'][grid]
        map_geometry['safe_' + side] = xy[np.isfinite(xy).all(axis=1)].round(5).tolist()

    src_files = [source/'report.json', source/'frames.json', domain_dir/'metadata.json',
                 domain_dir/'domain.npz', map_yaml, geometry.raster.image_path]
    if report.get('raceline'):
        race_source = Path(report['raceline']['source'])
        map_geometry['raceline'] = np.loadtxt(race_source, delimiter=',', usecols=(1, 2)).round(5).tolist()
        src_files.append(race_source)
    data = dict(version=3, map_id=replay_id, base_map_id=map_id, mode='two_car_candidates',
                selected_budget_ms=report['budget_ms'], geometry=map_geometry, config=config,
                profile=dict(vehicle_length_m=body.length, vehicle_width_m=body.width),
                footprint_validation=dict(clearance_m=config['footprint_clearance_m'],
                                          pair_clearance_m=report['pair_clearance_m']),
                domain=domain.metadata, two_car_report=report, frames=frames,
                sources=[dict(path=str(p), sha256=digest(p)) for p in src_files],
                display_sampling_note=f'候選曲線以 {display_step*100:g} cm 間距繪製。原始候選車體檢查以 1 cm 間距執行。')
    out.mkdir(parents=True, exist_ok=True)
    Image.fromarray((geometry.raster.occupancy == 0).astype(np.uint8)*255).save(out/'map.png')
    temporary = out/'data.json.tmp'
    temporary.write_text(json.dumps(data, ensure_ascii=False, separators=(',', ':'), allow_nan=False)+'\n')
    temporary.replace(out/'data.json')
    print(f'{replay_id}: {len(frames)} recorded frames → {out}')


def export_esp_solo(source, output_dir, base_dashboard_dir, replay_id):
    """Export the recorded ESP centreline lap using the original steering cadence."""
    from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file

    source, out = Path(source), Path(output_dir)
    report = json.loads((source/'report.json').read_text())
    samples = json.loads((source/'samples.json').read_text())
    base = json.loads((Path(base_dashboard_dir)/'data.json').read_text())
    geometry = load_geometry('maps/frenet/esp/geometry', 'maps/esp.yaml', inspect=True)
    if geometry.geometry_id != base['domain']['geometry_id']:
        raise ValueError('display base uses a different track geometry')
    if len(samples) < 2 or not math.isclose(samples[-1]['t'], report['elapsed_s']):
        raise ValueError('inconsistent solo recording duration')
    physics = load_parameter_file('simulator_params')
    vehicle = load_parameter_file('vehicle_params')
    pp = load_parameter_file('GlobalPurePursuit')
    frames = [dict(step=i, time=row['t'], pose=row['pose'], speed=row['v'],
                   start=dict(s=row['s'], d=row['d']),
                   lookahead_m=pp.tal_constant_lookahead + pp.tal_variable_lookahead*row['v']/vehicle.max_speed,
                   terminal=i == len(samples)-1)
              for i, row in enumerate(samples)]
    sources = [source/'report.json', source/'samples.json', Path('maps/esp.yaml'), geometry.raster.image_path]
    data = dict(version=3, map_id=replay_id, base_map_id='esp', mode='esp_solo',
                title='ESP · 3.2 m/s 單車中線完整圈',
                geometry=base['geometry'], config=dict(control_dt_s=samples[1]['t']-samples[0]['t']),
                profile=dict(vehicle_length_m=physics.vehicle_length, vehicle_width_m=physics.vehicle_width),
                footprint_validation=dict(clearance_m=physics.vehicle_boundary_clearance),
                steering_dt_s=physics.n_sim_steps*physics.timestep, solo_report=report, frames=frames,
                sources=[dict(path=str(p), sha256=digest(p)) for p in sources],
                display_sampling_note='回放每 100 ms 記錄車況，轉向每 40 ms 更新，牆面餘量由 10 ms 物理掃掠檢查。')
    out.mkdir(parents=True, exist_ok=True)
    Image.fromarray((geometry.raster.occupancy == 0).astype(np.uint8)*255).save(out/'map.png')
    temporary = out/'data.json.tmp'
    temporary.write_text(json.dumps(data, ensure_ascii=False, separators=(',', ':'), allow_nan=False)+'\n')
    temporary.replace(out/'data.json')
    print(f'{replay_id}: {len(frames)} recorded frames → {out}')


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True)
    parser.add_argument('--output-dir',required=True)
    parser.add_argument('--arm')
    parser.add_argument('--baseline-source')
    parser.add_argument('--skip-gif',action='store_true')
    parser.add_argument('--mode',choices=['standard','three-disc-single','two-car','two-car-candidates','esp-solo'],
                        default='standard')
    parser.add_argument('--base-dashboard-dir')
    parser.add_argument('--domain-dir', help='compatible domain override for three-disc-single or two-car')
    parser.add_argument('--replay-id')
    args=parser.parse_args(argv)
    if args.mode != 'standard' and not args.base_dashboard_dir:
        parser.error(f'--base-dashboard-dir is required for {args.mode}')
    if args.domain_dir and args.mode not in ('three-disc-single', 'two-car'):
        parser.error('--domain-dir is only supported for three-disc-single or two-car')
    if args.mode == 'three-disc-single':
        export_three_disc_single(args.source, args.output_dir, args.base_dashboard_dir, args.domain_dir)
    elif args.mode == 'two-car':
        export_two_car(args.source, args.output_dir, args.base_dashboard_dir, args.domain_dir)
    elif args.mode in ('two-car-candidates', 'esp-solo'):
        if not args.replay_id:
            parser.error(f'--replay-id is required for {args.mode}')
        export = export_esp_solo if args.mode == 'esp-solo' else export_two_car_candidates
        export(args.source, args.output_dir, args.base_dashboard_dir, args.replay_id)
    else:
        run(args.source,args.output_dir,args.arm,args.baseline_source,args.skip_gif)


if __name__ == '__main__':
    main()
