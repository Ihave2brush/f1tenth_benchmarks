"""Independent B3 replay and terminal/model/source audit, including failed probes."""
import argparse
import json
from pathlib import Path

import numpy as np

from .database import LapDatabase, sha256
from .run_smoke import replay
from .safe_set import select_local_safe_set


def verify(directory):
    root = Path(directory)
    session = json.loads((root/'session.json').read_text())
    rows = [json.loads(s) for s in (root/'samples.jsonl').read_text().splitlines()]
    controls = [json.loads(s) for s in (root/'controls.jsonl').read_text().splitlines()]
    database = LapDatabase.load(session['source_database'])
    measured = set(np.unique(np.concatenate([lap.sample_ids[:-1] for lap in database.laps])).tolist())
    result = dict(stage='B3_independent_audit', b3_full_lap_passed=session['passed'],
                  original_physical_replay=replay(rows, session['simulator_params'], session['start_pose']),
                  code_hashes_match=all(Path(p).exists() and sha256(p)==h for p, h in session['code_hashes'].items()),
                  input_hashes_match=all(Path(p).exists() and sha256(p)==h for p, h in session['input_hashes'].items()),
                  historical_npz_unchanged=all(sha256(Path(session['source_database'])/p)==h for p,h in session['historical_npz_hashes'].items()),
                  successful_control_count=0, maximum_terminal_error=0., maximum_dynamics_error=0.)
    np.testing.assert_allclose(np.diff([r['time'] for r in rows]), .1, atol=1e-10, rtol=0)
    assert all(r['reset_count']==1 for r in rows)
    for control in controls:
        if 'failure' in control:
            continue
        states, inputs = np.array(control['states']), np.array(control['controls'])
        terminal = select_local_safe_set(database, control['terminal_candidate'], cycle=0)
        assert control['terminal_sources']==list(terminal.provenance)
        lam = np.array(control['lambda_weights'])
        assert lam.min() >= 0 and abs(lam.sum()-1) < 1e-9
        error = float(np.max(abs(states[-1, :6]-terminal.states@lam)))
        assert error <= session['config']['settings']['equality_gate']
        np.testing.assert_allclose(control['next_terminal_candidate'], terminal.successors@lam, atol=1e-9, rtol=0)
        for identification in control['identification']:
            ids = identification['source_ids']
            assert identification['measured_only'] and len(ids)==len(set(ids))==80
            assert set(ids) <= measured
        assert control['planning_gate']['valid'] and not control['planning_gate']['global_planning_allowed']
        assert all(qp['status']=='solved' for qp in control['qp'])
        dynamics_error = 0.
        for k, model in enumerate(control['prediction_models']):
            residual = states[k+1]-np.array(model['A'])@states[k]-np.array(model['B'])@inputs[k]-model['C']
            dynamics_error = max(dynamics_error, float(np.max(abs(residual))))
        assert dynamics_error <= session['config']['settings']['equality_gate']
        result['maximum_terminal_error'] = max(result['maximum_terminal_error'], error)
        result['maximum_dynamics_error'] = max(result['maximum_dynamics_error'], dynamics_error)
        result['successful_control_count'] += 1
    result['audit_passed'] = all(result[k] for k in ['code_hashes_match', 'input_hashes_match', 'historical_npz_unchanged']) and result['original_physical_replay']['passed']
    result['scope'] = 'data integrity and executed MPC steps; a stopped probe is not a completed LMPC lap'
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = verify(args.directory)
    with Path(args.output).open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps(result, indent=2))
    if not result['audit_passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
