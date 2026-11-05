"""Measure direct-first/legacy-repair under unchanged dense validation.

100 ms is an observed qualification gate, not an enforced deadline. Late
geometrically valid candidates are never counted as timely useful output.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from .core.geometry import load_geometry
from .overtaking.road_table import get_road_table
from .overtaking.session import SessionConfig, make_plan


def run(cases_path, output, repeats=3, generation_mode='direct_then_repair'):
    root = Path(output)
    root.mkdir(parents=True, exist_ok=False)
    manifest = json.loads(Path(cases_path).read_text())
    report = dict(schema_version=1, target_s=.1, deadline_enforced=False,
                  generation_mode=generation_mode,
                  scope='make_plan_and_json_encoding_excludes_control_and_disk',
                  case_manifest_sha256=hashlib.sha256(Path(cases_path).read_bytes()).hexdigest(),
                  repeats=repeats, cases=[], passed_functional_checks=True)
    for case in manifest['cases']:
        geometry = load_geometry('maps/frenet/esp/geometry', 'maps/esp.yaml', inspect=True)
        assert geometry.geometry_id == manifest['geometry_id']
        started = perf_counter()
        get_road_table(geometry, 'maps/frenet/esp/geometry')
        initialization_s = perf_counter()-started
        settings = dict(case['settings'])
        settings['max_curvature_inv_m'] = settings.pop('max_curvature')
        config = SessionConfig(**settings, query_mode='fast_with_fallback',
                               generation_mode=generation_mode)
        state = dict(control_step=case['control_step'] or 0, sim_time_s=0.,
                     pose=case['pose'], valid=True, frenet=dict(valid=True))
        row = dict(name=case['name'], config=asdict(config), initialization_s=initialization_s, runs=[])
        for index in range(repeats+1):
            started = perf_counter()
            event = make_plan(geometry, state, config, index)
            generation_s = perf_counter()-started
            encode_start = perf_counter()
            raw = json.dumps(event, separators=(',', ':'), allow_nan=False).encode()
            encoding_s = perf_counter()-encode_start
            total_s = perf_counter()-started
            result = event.get('result', {})
            paths = event['paths']
            assert [p['candidate_id'] for p in paths] == [1, 2, 3, 4, 5]
            for path in paths:
                if path['sampled_valid']:
                    assert path['start_position_error_m'] <= config.max_start_position_error_m
                    assert abs(path['start_heading_error_rad']) <= config.max_start_heading_error_rad
                    assert path['polyline_collision_verified']
            measure = dict(cache='first_generation' if index == 0 else 'identical_input_warm',
                make_plan_s=generation_s, encoding_s=encoding_s, total_s=total_s,
                output_bytes=len(raw), completed_within_target=total_s <= .1,
                geometric_valid_count=event['sampled_valid_count'],
                timely_valid_count=event['sampled_valid_count'] if total_s <= .1 else 0,
                direct_valid_count=result.get('direct_valid_count', 0),
                repaired_valid_count=result.get('repaired_valid_count', 0),
                phases=result.get('timing'), query_reuse=result.get('query_reuse'),
                paths=[{k: p.get(k) for k in ('candidate_id', 'sampled_valid', 'reason',
                        'generation_route', 'repair_attempted', 'direct_attempt', 'timing',
                        'start_position_error_m', 'start_heading_error_rad')}
                       for p in paths])
            row['runs'].append(measure)
            print(json.dumps(dict(case=case['name'], repeat=index, total_s=total_s,
                valid=event['sampled_valid_count'], direct=measure['direct_valid_count'],
                repaired=measure['repaired_valid_count'], within_100ms=total_s <= .1)), flush=True)
        report['cases'].append(row)
        (root/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    moving_or_first = [c['runs'][0] for c in report['cases'] if c['name'] != 'outside_map_3m']
    report['all_first_generations_within_target'] = all(r['completed_within_target'] for r in moving_or_first)
    report['all_first_generations_timely_useful'] = all(r['timely_valid_count'] > 0 for r in moving_or_first)
    report['code_sha256'] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in [Path(__file__), Path(__file__).with_name('overtaking')/'robust_adaptive_paths.py',
                     Path(__file__).with_name('overtaking')/'geometry_queries.py',
                     Path(__file__).with_name('overtaking')/'road_table.py',
                     Path(__file__).with_name('overtaking')/'session.py']}
    (root/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', default='docs/project/reports/algorithm_p01_cases_20261027.json')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--generation-mode', choices=('optimized', 'direct_only', 'direct_then_repair', 'width_fraction_only', 'width_fraction_then_repair'),
                        default='direct_then_repair')
    args = parser.parse_args()
    if not 1 <= args.repeats <= 100:
        parser.error('repeats must be in [1, 100]')
    report = run(args.cases, args.output_dir, args.repeats, args.generation_mode)
    print(json.dumps({k: report[k] for k in ('passed_functional_checks',
        'all_first_generations_within_target', 'all_first_generations_timely_useful')}))


if __name__ == '__main__':
    main()
