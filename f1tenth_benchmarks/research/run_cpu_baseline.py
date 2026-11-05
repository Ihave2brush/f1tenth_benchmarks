"""CPU-only, fixed-input generator probe; no simulation or parameter writes.

Use the original single-thread BLAS environment. A generous repair deadline
keeps CPU comparisons from doing less work merely because of a 40 ms timeout.
Only this probe process's affinity changes, and it is restored in finally.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import resource
import time

import numpy as np
import scipy

from .core.geometry import load_geometry
from .overtaking.millimetre_domain import MillimetreDomain
from .overtaking.mm_bspline_repair import BudgetedDomainGenerator


def task_ticks():
    result = {}
    for path in Path('/proc/self/task').glob('*/stat'):
        fields = path.read_text().rsplit(')', 1)[1].split()
        result[path.parent.name] = int(fields[11]) + int(fields[12])
    return result


def run(output):
    initial = set(os.sched_getaffinity(0))
    report = dict(scope='generator_only_fixed_recorded_inputs_not_full_cycle',
        deadline_s=60., deadline_reason='prevent hardware speed changing attempted work',
        python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__,
        architecture=platform.machine(), original_affinity=sorted(initial),
        environment={k: os.environ.get(k) for k in
            ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS')},
        cgroup={name: (Path('/sys/fs/cgroup')/name).read_text().strip()
            for name in ('cpu.max', 'cpuset.cpus.effective', 'memory.max')}, arms=[])
    cpu_info = Path('/proc/cpuinfo').read_text().splitlines()
    report['cpu_model'] = next((s.split(':', 1)[1].strip() for s in cpu_info
                               if s.startswith('model name')), platform.processor())
    try:
        for map_id, dirname in [('my_map', 'my_map_mm40_outer_20261030_001'),
                                ('esp', 'esp_mm40_outer_20261030_001')]:
            root = Path('Data/research/overtaking')/dirname/'validation'
            original = json.loads((root/'report.json').read_text())
            plans_file = root/'budget_40ms_c1245'/'plans.jsonl'
            all_plans = [json.loads(s) for s in plans_file.read_text().splitlines()]
            indices = (range(len(all_plans)) if map_id == 'my_map' else
                       sorted(set(np.linspace(0, len(all_plans)-1, 32, dtype=int).tolist()
                                  + [21, 1309, 1318, 1588])))
            plans = [all_plans[i] for i in indices]
            geometry = load_geometry(f'maps/frenet/{map_id}/geometry',
                                     f'maps/{map_id}.yaml', inspect=True)
            domain = MillimetreDomain.load(original['domain_dir'], geometry,
                expected_margin=original['profile']['centre_prefilter_margin_m'])
            cfg = original['config']
            settings = dict(lookahead_m=cfg['lookahead_m'], max_slope=cfg['max_slope'],
                            max_curvature=cfg['max_curvature_inv_m'])
            def invoke(generator, plan):
                x, y, yaw = plan['source_pose']
                return generator(geometry, x, y, yaw, **settings)
            for affinity_name, affinity in [('unbound', initial),
                                             ('one_logical_cpu', {min(initial)})]:
                os.sched_setaffinity(0, affinity)
                warm = BudgetedDomainGenerator(domain, budget_s=60., repair_candidates=(1, 2, 4, 5))
                for plan in plans[:5]:
                    invoke(warm, plan)
                del warm
                durations, signatures = [], []
                before_tasks = task_ticks()
                cpu_start, wall_start = time.process_time(), time.perf_counter()
                repaired, timed_out = 0, 0
                for repeat in range(2):
                    generator = BudgetedDomainGenerator(domain, budget_s=60.,
                                                        repair_candidates=(1, 2, 4, 5))
                    for plan in plans:
                        started = time.perf_counter()
                        result = invoke(generator, plan)
                        durations.append(time.perf_counter()-started)
                        repair = result['bspline_repair']
                        repaired += len(repair['repaired_ids'])
                        timed_out += len(repair['timed_out_ids'])
                        signatures.append([(p['sampled_valid'], p['spline'])
                                           for p in result['paths']])
                wall_s, cpu_s = time.perf_counter()-wall_start, time.process_time()-cpu_start
                after_tasks = task_ticks()
                delta = {k: v-before_tasks.get(k, v) for k, v in after_tasks.items()}
                ticks = sum(delta.values())
                arm = dict(map=map_id, affinity_mode=affinity_name, affinity=sorted(affinity),
                    selected_steps=[p['source_control_step'] for p in plans], repeats=2,
                    calls=len(durations), wall_s=wall_s, cpu_s=cpu_s,
                    cpu_core_equivalents=cpu_s/wall_s,
                    measured_ms={name: float(np.percentile(durations, q))*1000
                                 for name, q in [('median', 50), ('p95', 95), ('max', 100)]},
                    active_threads=sum(v > 0 for v in delta.values()),
                    main_thread_tick_share=delta.get(str(os.getpid()), 0)/ticks if ticks else None,
                    native_thread_count=len(after_tasks), repaired=repaired, timed_out=timed_out,
                    process_peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
                    plans_sha256=hashlib.sha256(plans_file.read_bytes()).hexdigest(),
                    output_sha256=hashlib.sha256(json.dumps(signatures, sort_keys=True).encode()).hexdigest())
                report['arms'].append(arm)
                print(json.dumps(arm), flush=True)
            del domain, geometry
    finally:
        os.sched_setaffinity(0, initial)
    report['same_outputs_across_affinities'] = all(
        report['arms'][i]['output_sha256'] == report['arms'][i+1]['output_sha256']
        for i in (0, 2))
    report['completed'] = True
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    run(parser.parse_args().output)
