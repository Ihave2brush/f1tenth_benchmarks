"""Read-only HTTP acceptance against an existing recorded session.

Run from the repo root in the simulation Python environment. Prints a compact
JSON report; redirect it to a new report file when preserving evidence.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
from urllib.parse import urlsplit
from urllib.request import urlopen


def check(directory, base):
    if urlsplit(base).hostname not in ('127.0.0.1', 'localhost', '::1'):
        raise ValueError('use a local playback server')
    root = Path(directory)
    states = [json.loads(line) for line in (root/'states.jsonl').read_text().splitlines()]
    plans = [json.loads(line) for line in (root/'plans.jsonl').read_text().splitlines()]
    metadata = json.loads((root/'metadata.json').read_text())
    def api(route):
        with urlopen(base+route, timeout=10) as response:
            return json.load(response)
    overview = api('/api/session')
    assert overview['state_count'] == len(states)
    assert overview['geometry_id'] == metadata['geometry_id']
    assert overview['origin_sim_time_s'] == states[0]['sim_time_s']
    for steps in (range(len(states)), reversed(range(len(states)))):
        for step in steps:
            frame = api('/api/frame?step='+str(step))
            state = states[step]
            assert frame['pose'] == state['pose']
            assert frame['sim_time_s'] == state['sim_time_s']
            assert frame['control_step'] == step and len(frame['trail']) == step+1
            eligible = [p for p in plans if max(p['source_control_step'], p['available_control_step']) <= step
                        and max(p['source_sim_time_s'], p['available_sim_time_s']) <= state['sim_time_s']]
            expected = max(eligible, key=lambda p: (p['source_control_step'], p['plan_id'])) if eligible else None
            if expected is None:
                assert frame['plan'] is None
            else:
                shown = frame['plan']
                assert shown['plan_id'] == expected['plan_id']
                assert shown['source_pose'] == expected['source_pose']
                assert shown['source_control_step'] == expected['source_control_step']
                assert shown['source_sim_time_s'] == expected['source_sim_time_s']
                assert math.isclose(frame['candidate_age_s'], state['sim_time_s']-expected['source_sim_time_s'], abs_tol=1e-12)
                assert [p['candidate_id'] for p in shown['paths']] == [1,2,3,4,5]
                assert [p['reason'] for p in shown['paths']] == [p['reason'] for p in expected['paths']]
                assert [p['sampled_valid'] for p in shown['paths']] == [p['sampled_valid'] for p in expected['paths']]
                assert 'attempts' not in shown
            assert frame['at_end'] == (step == len(states)-1)
    return dict(schema_version=1, passed=True, session=str(root), geometry_id=metadata['geometry_id'],
        forward_frames=len(states), backward_frames=len(states), plan_count=len(plans),
        checks=dict(recorded_pose_and_time=True, five_candidate_ids=True, candidate_age=True,
                    failure_reasons=True, no_future_candidates=True, past_trail_only=True, end_state=True),
        source_sha256={name:hashlib.sha256((root/name).read_bytes()).hexdigest()
                       for name in ('metadata.json','states.jsonl','plans.jsonl','summary.json')},
        code_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            [Path('f1tenth_benchmarks/research/overtaking')/name for name in ('playback.py','playback.html','playback.js')]},
        planning_allowed=False, footprint_verified=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--url', default='http://127.0.0.1:8766')
    args = parser.parse_args()
    print(json.dumps(check(args.directory, args.url), indent=2, allow_nan=False))
