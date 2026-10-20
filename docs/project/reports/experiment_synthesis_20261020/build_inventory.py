"""Read-only research inventory; writes only beside this script, no optimizer/plant."""
import csv
import hashlib
import json
import re
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
REPORT = OUT.with_suffix('.md')
DATA = ROOT / 'Data/research'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def relative(path):
    return str(path.relative_to(ROOT))


def last_jsonl(path):
    if not path.exists() or not path.stat().st_size:
        return {}
    with path.open('rb') as stream:
        stream.seek(max(0, path.stat().st_size - 1024 * 1024))
        return json.loads(stream.read().splitlines()[-1])


def write_csv(name, rows, columns):
    with (OUT / name).open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


sessions = []
session_paths = sorted(DATA.rglob('session.json'))
for path in session_paths:
    d = json.loads(path.read_text())
    last = last_jsonl(path.with_name('samples.jsonl'))
    handoff = d.get('handoff') or {}
    steps = None
    if handoff and d.get('sample_count') is not None:
        steps = d['sample_count'] - 1 - handoff['raw_sample_id']
    elif d.get('stage') in ('B3_local_lmpc', 'B2.5_short_closed_loop'):
        steps = d.get('control_steps')
    paper = last.get('paper_state') or []
    last_s = paper[4] if len(paper) >= 5 else None
    local_s = last_s - handoff['s_origin'] if last_s is not None and 's_origin' in handoff else None
    initial_s = (handoff.get('local_state') or [None] * 6)[4]
    distance = local_s - initial_s if local_s is not None and initial_s is not None else None
    rmse = d.get('prediction_error_rmse') or []
    role = 'initialization' if 'initialization' in path.parts else 'session'
    if 'run_segment_001_logging_stop' in path.parts:
        role = 'archived_logging_segment_do_not_add_to_resumed_run'
    sessions.append(dict(
        path=relative(path), role=role, stage=d.get('stage'),
        experiment_id=d.get('experiment_id'), passed_raw=d.get('passed'),
        sample_count_raw=d.get('sample_count'), control_steps_raw=d.get('control_steps'),
        initialization_completed_laps_raw=d.get('initialization_completed_laps'),
        completed_full_laps_raw=d.get('completed_full_laps'),
        completed_lmpc_laps_raw=d.get('completed_lmpc_laps'),
        applied_lmpc_commands_derived=steps, final_core_s=last_s,
        final_local_s=local_s, handoff_distance_m_derived=distance,
        yaw_one_step_rmse_raw=rmse[2] if len(rmse) > 2 else None,
        error_raw=d.get('error'), sha256=digest(path)))

write_csv('session_inventory.csv', sessions, list(sessions[0]))

batches = []
for group in sorted(p for p in DATA.iterdir() if p.is_dir()):
    for path in sorted(p for p in group.iterdir() if p.is_dir()):
        # experiments and lifecycle stages are containers, not independent runs.
        children = sorted(p for p in path.iterdir() if p.is_dir()) if (
            path.name == 'experiments' or re.fullmatch(r'stage\d+', path.name)) else [path]
        for batch in children:
            files = [p for p in batch.rglob('*') if p.is_file() and '__pycache__' not in p.parts]
            reports = [relative(p) for p in files if p.name == 'report.md']
            records = [s for s in sessions if s['path'].startswith(relative(batch) + '/')]
            batches.append(dict(group=group.name, path=relative(batch), file_count=len(files),
                                session_count=len(records), report_paths='; '.join(reports)))
write_csv('batch_inventory.csv', batches, list(batches[0]))

source_paths = set(session_paths)
source_paths.update((ROOT / 'docs/local/lmpc').rglob('*.md'))
source_paths.update((ROOT / 'docs/local/lmpc/sources').rglob('*.*'))
source_paths.update(DATA.rglob('report.md'))
source_paths.update((ROOT / 'params/research').glob('*.yaml'))
source_paths.update(ROOT / p for p in [
    'docs/project/status.md', 'docs/project/features.md', 'docs/project/steering_actuator_modes.md',
    'docs/project/reports/manual_run_001_report.md', 'Data/research/lmpc/README.md',
    'Data/research/lmpc/experiments/A5_COMPARE_20261020_001/summary.csv',
    'Data/research/lmpc/experiments/A5_COMPARE_20261020_001/comparison.json',
    'Data/research/lmpc/experiments/QP_D1_20261020_001/summary.json',
    'f1tenth_benchmarks/research/lmpc/paper_model.py',
    'f1tenth_benchmarks/research/lmpc/input_kernel_model.py',
    'f1tenth_benchmarks/research/lmpc/controller.py',
    'f1tenth_benchmarks/research/lmpc/paper_controller.py',
    'f1tenth_benchmarks/research/lmpc/safe_set.py',
    'f1tenth_benchmarks/research/lmpc/run_lmpc.py',
    'f1tenth_benchmarks/simulator/dynamic_models.py',
])
sources = [dict(path=relative(p), bytes=p.stat().st_size, sha256=digest(p))
           for p in sorted(source_paths) if p.is_file()]

fixed_manifest = ROOT / 'docs/local/lmpc/sources/r0_20261012/manifest.json'
fixed = json.loads(fixed_manifest.read_text())
fixed_checks = [dict(file=x['saved_reference'], commit=x['commit'],
                     matches=digest(fixed_manifest.parent / x['saved_reference']) == x['sha256'])
                for x in fixed]
assert all(x['matches'] for x in fixed_checks)

c_path = DATA / 'lmpc/experiments/A5_COMPARE_20261020_001/summary.csv'
with c_path.open() as stream:
    cells = list(csv.DictReader(stream))
assert [int(c['applied_commands']) for c in cells] == [23, 6, 6, 6]
assert all(int(c['laps']) == 0 for c in cells)
assert all(s['completed_lmpc_laps_raw'] in (None, 0) for s in sessions)

broken = []
local_links = 0
for target in re.findall(r'\]\(([^)]+)\)', REPORT.read_text()):
    if '://' in target or target.startswith('#'):
        continue
    local_links += 1
    resolved = REPORT.parent / target.split('#')[0]
    if resolved.resolve() == (OUT / 'evidence_manifest.json').resolve():
        continue  # This artifact is written after checks below.
    if not resolved.exists():
        broken.append(target)
assert not broken, broken

manifest = dict(
    created_at=datetime.now(ZoneInfo('Asia/Taipei')).isoformat(),
    scope='All saved research session.json files and research batch directories; metadata inventory only',
    rules=[
        'Empty CSV cells mean unavailable, never inferred zero.',
        'completed_full_laps may include initialization PP laps; never equated to LMPC laps.',
        'Derived LMPC actions = sample_count - 1 - handoff.raw_sample_id when handoff exists.',
        'For B3/B2.5 without handoff, control_steps is retained as the recorded applied count.',
        'Repeated history, duplicate A7 attempt and archived checkpoint segment are not independent training data.',
        'No raw transition validity, model accuracy, replay or optimizer result is revalidated by this inventory.',
        'Top-level research directories and one extra level for experiments/stage containers define batch rows.',
    ],
    checks=dict(session_json_parsed=len(sessions), batches=len(batches),
                author_snapshots=fixed_checks, c_factorial_applied_commands=[23, 6, 6, 6],
                explicit_lmpc_lap_counts_all_zero=True, local_links=local_links,
                broken_local_links=broken),
    sources=sources,
    generated_artifacts=[dict(path=relative(p), sha256=digest(p)) for p in
                         [REPORT, OUT / 'build_inventory.py', OUT / 'batch_inventory.csv',
                          OUT / 'session_inventory.csv']],
)
(OUT / 'evidence_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(manifest['checks'], ensure_ascii=False, indent=2))
