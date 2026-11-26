"""Portable synthetic prepared-core benchmark; no ESP, generation or controls."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import platform
import sys
from time import perf_counter
import tracemalloc
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'/'research')]
from test_realtime_cost_core_v1 import ready
from f1tenth_benchmarks.research.overtaking.realtime_cost_core_v1 import prepare_runtime_inputs,evaluate_cost_batch,evaluate_runtime_cost_core


def stats(values):
    t=np.asarray(values)*1000
    return dict(n=len(t),median_ms=float(np.median(t)),p95_ms=float(np.percentile(t,95)),p99_ms=float(np.percentile(t,99)),max_ms=float(t.max()))


def main():
    p=argparse.ArgumentParser();p.add_argument('--repeats',type=int,default=200);p.add_argument('--warmup',type=int,default=20)
    p.add_argument('--output',type=Path);a=p.parse_args()
    if a.repeats<2 or a.warmup<0:raise ValueError('invalid_benchmark_counts')
    if a.output and a.output.exists():raise FileExistsError(a.output)
    record,context,length,projection=ready();batch=prepare_runtime_inputs(record,context,length_cache=length,projection_bundle=projection)
    rows={}
    for name,fn in [('prepared_core',lambda:evaluate_cost_batch(batch)),('adapter_plus_core',lambda:evaluate_runtime_cost_core(record,context,length_cache=length,projection_bundle=projection))]:
        for _ in range(a.warmup):fn()
        values=[]
        for _ in range(a.repeats):
            t=perf_counter();fn();values.append(perf_counter()-t)
        tracemalloc.start();fn();retained,peak=tracemalloc.get_traced_memory();tracemalloc.stop()
        rows[name]=dict(stats(values),python_peak_bytes=peak,python_retained_bytes=retained)
    out=dict(schema='runtime_cost_synthetic_benchmark_v1',python=sys.version,platform=platform.platform(),warmup=a.warmup,
        options=1,groups=1,samples_per_option=3,results=rows,
        scope='Synthetic fixture; excludes geometry setup, projection, native rollout, IO and ROS. No real-time guarantee.',
        inspection_only=True,planning_allowed=False,formal_comparison_qualified=False)
    if a.output:
        with a.output.open('x') as f:json.dump(out,f,indent=2,allow_nan=False)
    print(json.dumps(out,indent=2,allow_nan=False))

if __name__=='__main__':main()
