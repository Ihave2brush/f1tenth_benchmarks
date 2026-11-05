"""Profile frozen moving inputs separately from uninstrumented timing runs."""
import argparse
import cProfile
import json
from pathlib import Path
import pstats

from .overtaking.session import SessionConfig
from .run_short_horizon_benchmark import fresh_geometry, measure, sha


def run(inputs, output, horizon=6.):
    manifest = json.loads(Path(inputs).read_text())
    root = Path(output)
    root.mkdir(parents=True, exist_ok=False)
    config = SessionConfig(**manifest['settings'], lookahead_m=horizon,
        query_mode='fast_with_fallback', generation_mode='direct_then_repair')
    report = dict(input_sha256=sha(inputs), driver_sha256=sha(__file__),
        horizon_m=horizon, generation_mode=config.generation_mode,
        scope='cProfile_instrumented_measure_calls_excludes_separate_initialization',
        warning='Profiling overhead included; use uninstrumented benchmark for latency claims.',
        segments=[])
    for segment in manifest['moving_segments']:
        geometry, initialization = fresh_geometry(manifest['geometry_id'])
        profiler = cProfile.Profile()
        profiler.enable()
        rows = [measure(geometry, item['state'], config, i)
                for i, item in enumerate(segment['states'])]
        profiler.disable()
        profiler.dump_stats(str(root/(segment['name']+'.prof')))
        stats = pstats.Stats(profiler)
        entries = [dict(file=file, line=line, function=function, primitive_calls=value[0],
                        total_calls=value[1], self_s=value[2], cumulative_s=value[3])
                   for (file, line, function), value in stats.stats.items()]
        row = dict(name=segment['name'], initialization_s=initialization,
            instrumented_total_s=stats.total_tt, measurements=rows,
            top_cumulative=sorted(entries, key=lambda e: e['cumulative_s'], reverse=True)[:80],
            top_self=sorted(entries, key=lambda e: e['self_s'], reverse=True)[:40])
        report['segments'].append(row)
        (root/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps(dict(segment=segment['name'], instrumented_s=stats.total_tt,
                              top=row['top_cumulative'][:8])), flush=True)
        del geometry
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--horizon', type=float, choices=(5., 6., 7.), default=6.)
    args = parser.parse_args()
    run(args.inputs, args.output_dir, args.horizon)


if __name__ == '__main__':
    main()
