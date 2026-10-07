"""Published tracks must load using Git files only, with unchanged source checks."""
import hashlib
import json
from pathlib import Path

import pytest

from f1tenth_benchmarks.research.core.geometry.frenet import load_geometry
from f1tenth_benchmarks.research.core.geometry.section_ranges import load_ranges,query_section

ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'maps/frenet'


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
    assert actual==limited
    assert g.to_cartesian(0,0)['valid']
    endpoint=query_section(g,g.curve.L)
    assert endpoint['s_m']==pytest.approx(0 if g.curve.closed else g.curve.L)
