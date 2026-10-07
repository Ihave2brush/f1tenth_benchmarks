"""Explicitly oriented open-corridor reference; never invent a closed lap."""
import argparse
import heapq
from pathlib import Path

import numpy as np
from scipy import ndimage, interpolate

from .map_raster import load_map,FREE
from .extraction import ReferenceCurve,ray_boundary
from .build_map import save_geometry,verify_artifacts,render_overlay,json_bytes,intersections


def thin(mask):
    """Deterministic Zhang-Suen thinning; no additional imaging dependency."""
    image=np.pad(mask.astype(bool),1)
    while True:
        changed=False
        for phase in (0,1):
            p=[image[:-2,1:-1],image[:-2,2:],image[1:-1,2:],image[2:,2:],
               image[2:,1:-1],image[2:,:-2],image[1:-1,:-2],image[:-2,:-2]]
            count=sum(q.astype(int) for q in p)
            transitions=sum((~p[i]&p[(i+1)%8]).astype(int) for i in range(8))
            if phase==0:extra=~(p[0]&p[2]&p[4])&~(p[2]&p[4]&p[6])
            else:extra=~(p[0]&p[2]&p[6])&~(p[0]&p[4]&p[6])
            remove=image[1:-1,1:-1]&(count>=2)&(count<=6)&(transitions==1)&extra
            if remove.any():image[1:-1,1:-1][remove]=False;changed=True
        if not changed:return image[1:-1,1:-1]


def skeleton_path(mask,raster,start_hint):
    holes,n=ndimage.label(ndimage.binary_fill_holes(mask)&~mask)
    sizes=np.bincount(holes.ravel());small=[i for i in range(1,n+1) if sizes[i]*raster.resolution**2<=.02]
    seed_mask=mask|np.isin(holes,small)
    # Hole suppression affects only the seed. Actual occupancy and wall queries
    # remain unchanged, and every fitted reference sample must still be free.
    points=np.argwhere(thin(seed_mask));lookup={tuple(p):i for i,p in enumerate(points)}
    graph=[[] for _ in points]
    for i,(row,col) in enumerate(points):
        for dr in (-1,0,1):
            for dc in (-1,0,1):
                if dr==dc==0:continue
                j=lookup.get((row+dr,col+dc))
                if dr and dc and ((row+dr,col) in lookup or (row,col+dc) in lookup):continue
                if j is not None:graph[i].append((j,float(np.hypot(dr,dc))))
    ends=[i for i,g in enumerate(graph) if len(g)==1]
    if len(ends)<2:raise ValueError('unsupported_open_topology: no two skeleton endpoints')
    xy=raster.pixel_to_map(points[:,1],points[:,0])
    # The hint selects direction. Longest geodesic selects the primary corridor,
    # with side branches retained in the provenance, not silently made into laps.
    start=min(ends,key=lambda i:np.linalg.norm(xy[i]-start_hint))
    if np.linalg.norm(xy[start]-start_hint)>2:raise ValueError('start hint is not near a terminal skeleton endpoint')
    distances=np.full(len(points),np.inf);distances[start]=0;previous={};queue=[(0.,start)]
    while queue:
        distance,i=heapq.heappop(queue)
        if distance!=distances[i]:continue
        for j,length in graph[i]:
            candidate=distance+length
            if candidate<distances[j]:
                distances[j]=candidate;previous[j]=i;heapq.heappush(queue,(candidate,j))
    end=max((i for i in ends if np.isfinite(distances[i])),key=lambda i:distances[i])
    path=[end]
    while path[-1]!=start:path.append(previous[path[-1]])
    path.reverse()
    return xy[path],dict(skeleton_nodes=len(points),skeleton_endpoints=len(ends),
        selected_nodes=len(path),start_hint_xy=np.asarray(start_hint).tolist(),
        seed_suppressed_holes=len(small),seed_hole_area_limit_m2=.02,
        endpoint_selection='nearest start hint; farthest reachable skeleton endpoint',
        side_branches_are_not_in_reference=True)


def build_open_geometry(raster,start_hint,smoothing_m=.3,step_m=.05):
    if not 0<smoothing_m<=1 or not 0<step_m<=.05:raise ValueError('invalid open extraction options')
    hint=np.asarray(start_hint,float)
    if hint.shape!=(2,) or not np.isfinite(hint).all():raise ValueError('invalid start hint')
    labels,count=ndimage.label(raster.occupancy==FREE)
    pixel=raster.map_to_pixel(hint)
    if not pixel.inside_map or raster.classify(hint)!='free':raise ValueError('start hint must be free')
    mask=labels==labels[pixel.cell[1],pixel.cell[0]]
    seed,provenance=skeleton_path(mask,raster,hint)
    terminal=[]
    for point,outward in ((seed[0],seed[0]-seed[min(10,len(seed)-1)]),
                          (seed[-1],seed[-1]-seed[max(0,len(seed)-11)])):
        outward=outward/np.linalg.norm(outward)
        _,distance,_,reason=ray_boundary(raster,point,outward,mask)
        if reason is not None:raise ValueError('missing terminal boundary')
        terminal.append(point+max(0,distance-raster.resolution/2)*outward)
    seed=np.vstack((terminal[0],seed,terminal[1]))
    seed=seed[np.r_[True,np.linalg.norm(np.diff(seed,axis=0),axis=1)>1e-8]]
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(seed,axis=0),axis=1))]
    positions=np.linspace(0,arc[-1],int(np.ceil(arc[-1]/.1))+1)
    seed=np.column_stack([np.interp(positions,arc,seed[:,axis]) for axis in (0,1)])
    def free(curve):
        dense=np.linspace(0,curve.L,int(np.ceil(curve.L/(raster.resolution/4)))+1)
        xy,_,_,_=curve.sample(dense)
        return all(raster.classify(p)=='free' for p in xy)
    curve=None
    for factor in (1.,.5,.25):
        smoothed=ndimage.gaussian_filter1d(seed,smoothing_m*factor/(positions[1]-positions[0]),axis=0,mode='nearest')
        smoothed[0]=seed[0];smoothed[-1]=seed[-1]
        tck,_=interpolate.splprep(smoothed.T,s=len(seed)*.01**2,per=False,k=3)
        candidate=ReferenceCurve(tck,closed=False)
        if free(candidate):curve=candidate;break
    if curve is None:raise ValueError('open centerline smoothing leaves free corridor')
    dense=curve.sample(np.linspace(0,curve.L,int(np.ceil(curve.L/(raster.resolution/4)))+1))[0]
    if intersections(dense,closed=False):raise ValueError('open reference self intersection')
    s=np.linspace(0,curve.L,int(np.ceil(curve.L/step_m))+1)
    xy,tangent,psi,kappa=curve.sample(s);boundaries=[]
    for point,t in zip(xy,tangent):
        normal=np.array([-t[1],t[0]])
        left,dl,lk,lr=ray_boundary(raster,point,normal,mask)
        right,dr,rk,rr=ray_boundary(raster,point,-normal,mask)
        boundaries.append(dict(left=left,right=right,d_left=dl,d_right=None if dr is None else -dr,
            left_kind=lk,right_kind=rk,valid=lr is None and rr is None,reason=lr or rr or ''))
    options=dict(method='skeleton_open_v1',smoothing_length_m=smoothing_m,
        effective_smoothing_length_m=smoothing_m*factor,output_step_m=step_m,
        endpoint_policy='extend terminal direction to original occupancy cap minus half pixel; no extrapolation',**provenance)
    return dict(curve=curve,s=s,xy=xy,psi=psi,kappa=kappa,boundaries=boundaries,corridor=mask,
        anchor=dict(input_xy=hint.tolist(),projected_xy=xy[0].tolist(),
            input_heading_rad=float(psi[0]),displacement_m=float(np.linalg.norm(xy[0]-hint)),
            heading_error_rad=0.,selection='user direction hint selects terminal skeleton endpoint'),
        options=options,refinement=options)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-yaml',type=Path,required=True)
    parser.add_argument('--start-hint',type=float,nargs=2,required=True)
    parser.add_argument('--output-root',type=Path,required=True)
    args=parser.parse_args();raster=load_map(args.map_yaml)
    build=build_open_geometry(raster,args.start_hint)
    invalid=sum(not b['valid'] for b in build['boundaries'])
    validation=dict(stage='B',stage_b_passed=invalid==0,closed=False,L_m=build['curve'].L,
        invalid_boundary_samples=invalid,centerline_free=True,planning_allowed=False,
        limitations=['open primary corridor selected from explicit direction hint',
            'terminal cap regions beyond reference endpoints have no Frenet coordinates',
            'source endpoint d samples require exact checks; no closed lap or global domain approval'])
    target=save_geometry(raster,build,args.output_root,args.map_yaml.stem,validation)
    render_overlay(raster,build,target);verify_artifacts(target)
    print(json_bytes(dict(output=str(target),validation=validation)).decode(),flush=True)
    if not validation['stage_b_passed']:raise SystemExit(1)


if __name__=='__main__':main()
