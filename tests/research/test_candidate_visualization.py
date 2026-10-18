"""V4 rendering contracts against real V3 output using a headless backend."""
import copy
import os
import subprocess
import sys

import matplotlib
matplotlib.use('Agg', force=True)
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import pytest

from f1tenth_benchmarks.research.core.geometry import load_geometry
from f1tenth_benchmarks.research.overtaking.candidate_validation import validate_candidate_paths
from f1tenth_benchmarks.research.overtaking import candidate_visualization as v4


@pytest.fixture(autouse=True)
def close_figures():
    yield
    plt.close('all')


@pytest.fixture(scope='module')
def esp():
    return load_geometry('maps/frenet/esp/geometry', 'maps/esp.yaml', inspect=True)


@pytest.fixture(scope='module')
def mixed_result(esp):
    # A real V3 result with both sampled-valid and rejected paths.
    result = validate_candidate_paths(esp, 0, 0, lookahead_m=40, sample_step_m=.25)
    assert result['sampled_valid_count'] == 3
    return result


def candidate_lines(ax):
    return [line for line in ax.lines if line.get_label().startswith('Candidate ')]


def collection(ax, label):
    return next(item for item in ax.collections if item.get_label() == label)


@pytest.mark.parametrize('name', ['esp', 'aut', 'gbr', 'mco', 'CornerHall', 'my_map'])
def test_six_maps_centerline_boundaries_and_paths(name, tmp_path):
    geometry = load_geometry(f'maps/frenet/{name}/geometry', f'maps/{name}.yaml', inspect=True)
    result = validate_candidate_paths(geometry, *geometry.reference(1)['xy'],
                                      lookahead_m=5, sample_step_m=.5)
    fig, ax = v4.plot_candidate_paths(geometry, result)
    reference = next(line for line in ax.lines if line.get_label() == 'Reference centerline')
    expected, _, _, _ = geometry.curve.sample(np.linspace(0, geometry.curve.L, 350))
    np.testing.assert_allclose(reference.get_xydata(), expected)
    # Scatter positions must be actual exact-grid-ray hits, not connected CSV walls.
    expected_left, expected_right = [], []
    for s in np.linspace(0, geometry.curve.L, 240, endpoint=not geometry.curve.closed):
        boundary = geometry.boundaries(float(s))
        if boundary['valid']:
            if boundary['left_xy'] is not None:
                expected_left.append(boundary['left_xy'])
            if boundary['right_xy'] is not None:
                expected_right.append(boundary['right_xy'])
    np.testing.assert_allclose(collection(ax, 'Left boundary samples').get_offsets(), expected_left)
    np.testing.assert_allclose(collection(ax, 'Right boundary samples').get_offsets(), expected_right)
    assert not any('boundary' in line.get_label().lower() for line in ax.lines)
    lines = candidate_lines(ax)
    assert len(lines) == 5
    for path, line in zip(result['paths'], lines):
        assert line.get_linestyle() == ('-' if path['sampled_valid'] else '--')
        if path['points']:
            expected_xy = [[p['x'], p['y']] for p in path['points']]
            np.testing.assert_allclose(line.get_xydata(), expected_xy)
    assert result['inspection_only'] and not result['continuous_verified']
    assert not result['footprint_verified']
    output = tmp_path / f'{name}.png'
    fig.savefig(output)
    with Image.open(output) as image:
        image.load()
        assert image.format == 'PNG' and min(image.size) > 100


def test_pass_reject_styles_and_first_invalid_markers(esp, mixed_result):
    fig, ax = v4.plot_candidate_paths(esp, mixed_result)
    rejected = [p for p in mixed_result['paths'] if not p['sampled_valid']]
    assert len(rejected) == 2
    lines = candidate_lines(ax)
    assert sum(line.get_linestyle() == '-' for line in lines) == 3
    assert sum(line.get_linestyle() == '--' for line in lines) == 2
    markers = [c for c in ax.collections if c.get_zorder() == 5]
    assert len(markers) == 2
    for path, marker in zip(rejected, markers):
        point = path['points'][path['first_invalid_sample']]
        np.testing.assert_allclose(marker.get_offsets(), [[point['x'], point['y']]])
        annotation = next(t for t in ax.texts if t.get_text().startswith(f"C{path['candidate_id']}:"))
        assert point['reason'] in annotation.get_text()
        np.testing.assert_allclose(annotation.xy, [point['x'], point['y']])
    assert '3/5' in ax.get_title()


def test_real_invalid_targets_without_points_render():
    geometry = load_geometry('maps/frenet/aut/geometry', 'maps/aut.yaml', inspect=True)
    result = validate_candidate_paths(geometry, *geometry.reference(1)['xy'],
                                      lookahead_m=30, sample_step_m=1)
    invalid = [p for p in result['paths'] if not p['target_valid']]
    assert invalid and all(p['points'] == [] for p in invalid)
    fig, ax = v4.plot_candidate_paths(geometry, result)
    lines = candidate_lines(ax)
    assert len(lines) == 5
    for path in invalid:
        line = next(line for line in lines if line.get_label().startswith(f"Candidate {path['candidate_id']}:"))
        assert len(line.get_xdata()) == 0 and path['reason'] in line.get_label()
    fig.canvas.draw()


@pytest.mark.parametrize('missing', ['x', 'y', 'both', 'nan', 'absent'])
def test_missing_invalid_coordinates_break_line_without_crashing(esp, mixed_result, missing):
    result = copy.deepcopy(mixed_result)
    path = result['paths'][0]
    index = path['first_invalid_sample']
    point = path['points'][index]
    if missing == 'both':
        point.update(x=None, y=None)
    elif missing == 'nan':
        point['x'] = float('nan')
    elif missing == 'absent':
        point.pop('x')
        point.pop('y')
    else:
        point[missing] = None
    fig, ax = v4.plot_candidate_paths(esp, result)
    xy = candidate_lines(ax)[0].get_xydata()
    assert np.isnan(xy[index]).all()
    assert np.isfinite(xy[index-1]).all() and np.isfinite(xy[index+1]).all()
    # The unknown failed location must not be replaced by another coordinate.
    assert not any(t.get_text().startswith('C1:') for t in ax.texts)
    assert len([c for c in ax.collections if c.get_zorder() == 5]) == 1
    fig.canvas.draw()


@pytest.mark.parametrize('index', [0, None, -1, 100000])
def test_first_invalid_index_zero_and_unavailable(esp, mixed_result, index):
    result = copy.deepcopy(mixed_result)
    result['paths'][0]['first_invalid_sample'] = index
    fig, ax = v4.plot_candidate_paths(esp, result)
    annotations = [t for t in ax.texts if t.get_text().startswith('C1:')]
    if index == 0:
        assert len(annotations) == 1
        point = result['paths'][0]['points'][0]
        np.testing.assert_allclose(annotations[0].xy, [point['x'], point['y']])
    else:
        assert annotations == []
    fig.canvas.draw()


@pytest.mark.parametrize('key,value', [('geometry_id', 'other'), ('frame_id', 'odom')])
def test_geometry_and_frame_mismatch_rejected(esp, mixed_result, key, value):
    result = copy.deepcopy(mixed_result)
    result[key] = value
    with pytest.raises(ValueError, match='mismatch'):
        v4.plot_candidate_paths(esp, result)


def test_cli_main_reuses_real_v3_and_never_shows(esp, mixed_result, tmp_path, monkeypatch):
    calls = []
    original = v4.validate_candidate_paths
    def validate(**kwargs):
        calls.append(kwargs)
        return original(**kwargs)
    monkeypatch.setattr(v4, 'validate_candidate_paths', validate)
    def forbidden_show():
        pytest.fail('--no-show must not invoke plt.show')
    monkeypatch.setattr(v4.plt, 'show', forbidden_show)
    output = tmp_path / 'v4.png'
    monkeypatch.setattr(sys, 'argv', ['candidate_visualization',
        '--geometry-dir', 'maps/frenet/esp/geometry', '--map-yaml', 'maps/esp.yaml',
        '--x', '0', '--y', '0', '--lookahead', '5', '--margin', '.2',
        '--sample-step', '.5', '--save', str(output), '--no-show'])
    v4.main()
    assert len(calls) == 1
    assert calls[0]['lookahead_m'] == 5 and calls[0]['sample_step_m'] == .5
    assert calls[0]['safety_margin_m'] == .2
    assert plt.get_fignums() == []
    with Image.open(output) as image:
        image.verify()


@pytest.mark.parametrize('save', [True, False])
def test_cli_subprocess_save_and_no_show(tmp_path, save):
    cmd = [sys.executable, '-B', '-m',
        'f1tenth_benchmarks.research.overtaking.candidate_visualization',
        '--geometry-dir', 'maps/frenet/esp/geometry', '--map-yaml', 'maps/esp.yaml',
        '--x', '0', '--y', '0', '--lookahead', '5', '--sample-step', '1', '--no-show']
    output = tmp_path / 'cli.png'
    if save:
        cmd += ['--save', str(output)]
    environment = dict(os.environ, MPLBACKEND='Agg', MPLCONFIGDIR=str(tmp_path / 'mpl'))
    completed = subprocess.run(cmd, capture_output=True, text=True, env=environment, timeout=60)
    assert completed.returncode == 0, completed.stderr
    assert 'Sampled-valid paths:' in completed.stdout
    assert output.exists() == save
    if save:
        with Image.open(output) as image:
            image.verify()
