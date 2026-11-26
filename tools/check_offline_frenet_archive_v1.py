"""Read-only optional saved-sidecar replay; explicit paths and exclusive output."""
import argparse
import json
import math
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from f1tenth_benchmarks.research.core.geometry import load_geometry
from f1tenth_benchmarks.research.core.geometry.tracker import TrackingOptions
from f1tenth_benchmarks.research.overtaking.offline_rollout_frenet_v1 import project_rollout_record


def compare(a,b,path='root'):
    if isinstance(a,dict):
        if not isinstance(b,dict) or set(a)!=set(b):raise ValueError('fields:'+path)
        for key in a:compare(a[key],b[key],path+'/'+key)
    elif isinstance(a,list):
        if not isinstance(b,list) or len(a)!=len(b):raise ValueError('length:'+path)
        for i,(x,y) in enumerate(zip(a,b)):compare(x,y,path+'/'+str(i))
    elif type(a) in (int,float) and type(b) in (int,float):
        if not math.isfinite(a) or not math.isfinite(b) or abs(a-b)>1e-9:raise ValueError('value:'+path)
    elif a!=b:raise ValueError('value:'+path)


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True)
    p.add_argument('--geometry-dir',type=Path,required=True);p.add_argument('--map-yaml',type=Path,required=True)
    p.add_argument('--expected',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    records=[json.loads(line) for line in args.source.read_text().splitlines() if line.strip()]
    expected=json.loads(args.expected.read_text())
    if not isinstance(expected,list) or not expected or len(expected)!=len(records):raise ValueError('source_expected_count_mismatch')
    geometry=load_geometry(str(args.geometry_dir),str(args.map_yaml),inspect=True)
    output=[]
    for record,old in zip(records,expected):
        opts=TrackingOptions(**old['tracking_options'])
        result=project_rollout_record(record,geometry,tracking_options=opts,anchor_tolerance_m=old['anchor_tolerance_m'])
        compare(old,result);output.append(result)
    with args.output.open('x') as f:json.dump(output,f,indent=2,allow_nan=False)
    print(json.dumps(dict(records=len(output),options=sum(len(r['options']) for r in output),parity=True,tolerance=1e-9)))

if __name__=='__main__':main()
