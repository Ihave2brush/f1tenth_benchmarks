"""Published tracks must load using Git files only, with unchanged source checks."""
import hashlib
import json
from numbers import Integral, Real
from pathlib import Path

import pytest

from f1tenth_benchmarks.research.core.geometry.frenet import load_geometry
from f1tenth_benchmarks.research.core.geometry.section_ranges import load_ranges,query_section

ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'maps/frenet'


def assert_same_section(actual, expected, path='section'):
    """Keep structure/discrete values exact; allow only floating-point roundoff.

    Published JSON was generated on another CPU/BLAS build. A 1e-12 absolute
    tolerance is far below the map and query tolerances; no relative tolerance
    scales it up for large coordinates. Artifact hashes remain checked exactly.
    """
    if isinstance(expected, dict):
        assert isinstance(actual, dict) and actual.keys() == expected.keys(), path
        for key in expected:
            assert_same_section(actual[key], expected[key], f'{path}.{key}')
    elif isinstance(expected, list):
        assert isinstance(actual, list) and len(actual) == len(expected), path
        for index, (a, e) in enumerate(zip(actual, expected)):
            assert_same_section(a, e, f'{path}[{index}]')
    elif isinstance(expected, float):
        assert isinstance(actual, Real) and not isinstance(actual, Integral), path
        assert actual == pytest.approx(expected, rel=0, abs=1e-12), path
    else:
        assert type(actual) is type(expected) and actual == expected, path


def test_section_comparison_accepts_ci_coordinate_roundoff():
    actual = {'boundary': {'left_xy': [16.762927652689932, -2.149999999999999]},
              'valid': False, 'sample_count': 92, 'kind': 'occupied', 'reason': None}
    expected = {**actual, 'boundary': {'left_xy': [16.762927652689932, -2.1499999999999995]}}
    assert_same_section(actual, expected)


@pytest.mark.parametrize('actual,expected', [
    ({'xy': [0., .001]}, {'xy': [0., 0.]}),
    ({'xy': [100., 0.]}, {'xy': [100.+1e-8, 0.]}),
    ({'valid': True}, {'valid': False}),
    ({'valid': 0}, {'valid': False}),
    ({'samples': 92.}, {'samples': 92}),
    ({'kind': 'free'}, {'kind': 'occupied'}),
    ({'xy': [0.]}, {'xy': [0., 0.]}),
    ({'x': 0., 'extra': False}, {'x': 0.}),
    ({'x': float('nan')}, {'x': 0.}),
])
def test_section_comparison_rejects_meaningful_or_discrete_changes(actual, expected):
    with pytest.raises(AssertionError):
        assert_same_section(actual, expected)


def test_publication_assets_and_portable_catalog():
    publication=json.loads((BASE/'publication.json').read_text())
    for name,digest in publication['file_hashes'].items():
        assert hashlib.sha256((BASE/name).read_bytes()).hexdigest()==digest
    entries=json.loads((BASE/'inspector_catalog.json').read_text())
    assert len(entries)==publication['track_count']==6
    for entry in entries:
        for key in ('geometry','ranges','map_yaml'):
            path=(BASE/entry[key]).resolve()
            assert path.is_relative_to(ROOT/'maps') and path.exists()
        assert 'report' not in entry and 'diagnostics' not in entry


@pytest.mark.parametrize('map_id',['esp','aut','gbr','mco','CornerHall','my_map'])
def test_published_track_loads_gzip_and_keeps_section_results(map_id):
    geometry_dir=BASE/map_id/'geometry';range_dir=BASE/map_id/'ranges'
    g=load_geometry(geometry_dir,ROOT/'maps'/f'{map_id}.yaml',inspect=True)
    saved=load_ranges(range_dir,g,geometry_dir)
    assert not (range_dir/'section_ranges.json').exists()
    assert saved['summary']['range_interface_passed'] and saved['geometry_audit']['passed']
    assert g.curve.closed==(map_id!='CornerHall')
    for name in ('centerline.csv','boundaries.csv'):
        assert (geometry_dir/name).exists() and not (range_dir/name).exists()
    limited=next((p for p in saved['sections'] if p['limited_ranges']),saved['sections'][0])
    actual=query_section(g,limited['s_m'])
    assert_same_section(actual, limited)
    assert g.to_cartesian(0,0)['valid']
    endpoint=query_section(g,g.curve.L)
    assert endpoint['s_m']==pytest.approx(0 if g.curve.closed else g.curve.L)
