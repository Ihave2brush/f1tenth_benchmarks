"""Closed map import without a legacy centreline; explicit direction and anchor."""
import argparse
from pathlib import Path

import numpy as np
from scipy import ndimage,interpolate

from .map_raster import load_map,FREE,sha256
from .extraction import (ExtractionOptions,ReferenceCurve,corridor_loops,
    distance_balanced_seed,refine_centerline,signed_area,build_geometry)
from .build_map import save_geometry,validate_candidate,verify_artifacts,json_bytes


def build_external_geometry(raster,direction='counterclockwise',start_hint=None):
    if direction not in ('counterclockwise','clockwise'):raise ValueError('invalid direction')
    labels,count=ndimage.label(raster.occupancy==FREE)
    eligible=[];excluded=[]
    for component in range(1,count+1):
        cells=np.argwhere(labels==component)
        anchor=raster.pixel_to_map(*cells[0][::-1])
        try:mask,loops=corridor_loops(raster,anchor)
        except ValueError as error:
            excluded.append(dict(component=component,cells=len(cells),reason=str(error)))
            continue
        eligible.append((component,mask,loops))
    if start_hint is None:
        if len(eligible)!=1:raise ValueError('need start hint: expected exactly one supported annular corridor')
        component,mask,loops=eligible[0]
    else:
        hint=np.asarray(start_hint,float)
        pixel=raster.map_to_pixel(hint)
        if not pixel.inside_map or raster.classify(hint)!='free':raise ValueError('start hint must be free')
        selected=int(labels[pixel.cell[1],pixel.cell[0]])
        choices=[p for p in eligible if p[0]==selected]
        if len(choices)!=1:raise ValueError('start hint is not in a supported annular corridor')
        component,mask,loops=choices[0]
    options=ExtractionOptions(method='equidistant_periodic_v1')
    middle,_=refine_centerline(raster,mask,distance_balanced_seed(raster,mask),options)
    if (signed_area(middle)>0)!=(direction=='counterclockwise'):middle=middle[::-1]
    data=np.vstack((middle,middle[0]))
    tck,_=interpolate.splprep(data.T,s=len(middle)*options.smoothing_rms_m**2,per=True,k=3)
    curve=ReferenceCurve(tck)
    positions=np.linspace(0,curve.L,int(np.ceil(curve.L/.05)),endpoint=False)
    xy,tangent,psi,kappa=curve.sample(positions)
    window_m=min(1.,curve.L/4)
    if start_hint is None:
        width=max(1,int(np.ceil(window_m/(positions[1]-positions[0]))))
        score=ndimage.maximum_filter1d(np.abs(kappa),size=width,mode='wrap')
        index=int(np.argmin(score))
        method='minimum maximum absolute curvature in a physical arclength window'
    else:
        index=int(np.argmin(np.linalg.norm(xy-hint,axis=1)))
        method='nearest preliminary reference sample to explicit start hint'
    build=build_geometry(raster,options,xy[index],psi[index])
    if (signed_area(build['xy'])>0)!=(direction=='counterclockwise'):
        raise ValueError('generated reference direction disagrees with requested direction')
    build['options'].update(anchor_selection=method,anchor_window_m=window_m,
        requested_direction=direction,user_start_hint=None if start_hint is None else hint.tolist(),
        eligible_corridors=len(eligible),selected_component=component,selected_free_cells=int(mask.sum()),
        excluded_components=excluded)
    return build


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-yaml',required=True,type=Path)
    parser.add_argument('--original-yaml',type=Path)
    parser.add_argument('--direction',choices=['counterclockwise','clockwise'],required=True)
    parser.add_argument('--start-hint',type=float,nargs=2)
    parser.add_argument('--output-root',required=True,type=Path)
    args=parser.parse_args();raster=load_map(args.map_yaml)
    if args.original_yaml:raster.source_hashes['original_yaml']=sha256(args.original_yaml)
    build=build_external_geometry(raster,args.direction,args.start_hint)
    validation=validate_candidate(raster,build)
    target=save_geometry(raster,build,args.output_root,args.map_yaml.stem,validation)
    verify_artifacts(target)
    print(json_bytes(dict(output=str(target),validation=validation)).decode(),flush=True)
    if not validation['stage_b_passed']:raise SystemExit(1)


if __name__=='__main__':main()
