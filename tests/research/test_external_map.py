"""Actual PGM semantics plus independent externally anchored closed references."""
from pathlib import Path

import numpy as np
import pytest

from f1tenth_benchmarks.research.core.geometry.map_raster import load_map,MapRaster,FREE,UNKNOWN
from f1tenth_benchmarks.research.core.geometry.extraction import signed_area
from f1tenth_benchmarks.research.core.geometry.build_external_map import build_external_geometry

ROOT=Path(__file__).resolve().parents[2]


def test_actual_pgm_keeps_user_unknown_semantics_and_metadata():
    original=load_map(ROOT/'maps/my_map.original.yaml')
    working=load_map(ROOT/'maps/my_map.yaml')
    assert (working.width,working.height)==(174,128)
    assert working.resolution==.05
    np.testing.assert_array_equal(working.origin,[-4.8,-4.1,0])
    np.testing.assert_array_equal(original.gray,working.gray)
    assert np.all(original.occupancy[original.gray==205]==FREE)
    assert np.all(working.occupancy[working.gray==205]==UNKNOWN)
    assert np.count_nonzero(working.occupancy==FREE)==5211


@pytest.mark.parametrize('direction,positive',[('counterclockwise',True),('clockwise',False)])
def test_generic_external_reference_uses_requested_direction(direction,positive):
    rows,cols=np.indices((120,120));radius=np.hypot(cols-60,rows-60)
    gray=np.where((radius>25)&(radius<45),254,0).astype(np.uint8)
    raster=MapRaster(gray,dict(resolution=.05,origin=[-3,-3,0],negate=0,free_thresh=.196,occupied_thresh=.65))
    built=build_external_geometry(raster,direction)
    assert (signed_area(built['xy'])>0)==positive
    assert built['options']['requested_direction']==direction
    assert built['anchor']['displacement_m']<1e-6
    assert all(p['valid'] for p in built['boundaries'])


def test_open_space_does_not_silently_become_external_closed_track():
    raster=MapRaster(np.full((20,20),254,np.uint8),dict(resolution=.05,
        origin=[0,0,0],negate=0,free_thresh=.196,occupied_thresh=.65))
    with pytest.raises(ValueError,match='exactly one supported annular corridor'):
        build_external_geometry(raster)
