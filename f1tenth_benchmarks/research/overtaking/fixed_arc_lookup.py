"""Reusable fixed-s inverse brackets; queries retain exact arc quadrature.

Grid values only seed root brackets. Reference XY/curvature and road boundaries
are evaluated at each requested s, never interpolated or snapped to grid sites.
Unresolved inverse roots fall back to ReferenceCurve.u_at_s.
"""
from collections import OrderedDict
import numpy as np
from scipy.special import roots_legendre
from .fixed_grid import FixedRoadWindow

POLICY = 'fixed_s_brackets_exact_gauss16_inverse_v1'


class FixedArcLookup:
    def __init__(self, curve, spacing=.05, max_anchors=4096):
        if type(max_anchors) is not int or max_anchors<2:
            raise ValueError('invalid arc anchor capacity')
        self.curve=curve
        self.window=FixedRoadWindow(spacing,curve.L,curve.closed)
        self.spacing=float(spacing);self.max_anchors=max_anchors
        self.anchors=OrderedDict()
        self.nodes,self.weights=roots_legendre(16)
        self.stats=dict(anchor_hits=0,anchor_misses=0,anchor_evictions=0,
                        inverse_points=0,scalar_fallbacks=0,fixed_bracket_points=0,
                        batch_rounds=0)
        self.window_summary=None
        self._rebuild()

    def _rebuild(self):
        pairs=dict(zip(self.curve.s_nodes,self.curve.u_nodes))
        for target,u in self.anchors.values():
            pairs.setdefault(target,u)
        self.arc_nodes=np.asarray(sorted(pairs))
        self.u_nodes=np.asarray([pairs[s] for s in self.arc_nodes])

    def prepare_window(self,start,distance):
        _,sites,summary=self.window.sample(start,distance)
        # The two request endpoints are not fixed-grid sites and are not cached.
        for s in sites[1:-1]:
            key=float(s)
            if key in self.anchors:
                self.stats['anchor_hits']+=1;self.anchors.move_to_end(key)
            else:
                self.stats['anchor_misses']+=1
                target=(key+self.curve.start_s)%self.curve.L if self.curve.closed else key
                self.anchors[key]=(float(target),self.curve.u_at_s(key))
                if len(self.anchors)>self.max_anchors:
                    self.anchors.popitem(last=False);self.stats['anchor_evictions']+=1
        self._rebuild();self.window_summary=summary

    def _residual(self,u,a,base,target):
        """Same per-knot Gauss-Legendre(16) equation as scalar u_at_s."""
        half=(u-a)*.5
        nodes=half[:,None]*self.nodes+(u+a)[:,None]*.5
        first=self.curve.evaluate(nodes.ravel(),1).reshape(len(u),16,2)
        speed=np.linalg.norm(first,axis=2)
        return base+half*(speed@self.weights)-target

    def inverse_many(self,source_s):
        s=np.asarray(source_s,dtype=float).reshape(-1)
        if not np.isfinite(s).all():raise ValueError('nonfinite_input')
        if not self.curve.closed and np.any((s<0)|(s>self.curve.L)):
            raise ValueError('outside_reference')
        if not len(s):return np.empty(0)
        target=(s+self.curve.start_s)%self.curve.L if self.curve.closed else s
        spans=np.clip(np.searchsorted(self.curve.s_nodes,target,side='right')-1,
                      0,len(self.curve.u_nodes)-2)
        a=self.curve.u_nodes[spans];b=self.curve.u_nodes[spans+1]
        base=self.curve.s_nodes[spans]
        i=np.clip(np.searchsorted(self.arc_nodes,target,side='right')-1,0,len(self.arc_nodes)-2)
        # Cached scalar roots have finite tolerance: pad, intersect original
        # knot spans, then verify signs instead of trusting cached endpoints.
        lo=np.maximum(a,self.u_nodes[i]-1e-12)
        hi=np.minimum(b,self.u_nodes[i+1]+1e-12)
        flo=self._residual(lo,a,base,target);fhi=self._residual(hi,a,base,target)
        valid=(lo<=hi)&np.isfinite(flo)&np.isfinite(fhi)&(flo<=0)&(fhi>=0)
        lo=np.where(valid,lo,a);hi=np.where(valid,hi,b)
        self.stats['fixed_bracket_points']+=int(np.count_nonzero(valid&((hi-lo)<(b-a)*.99)))
        fraction=(target-self.arc_nodes[i])/(self.arc_nodes[i+1]-self.arc_nodes[i])
        u=np.clip(self.u_nodes[i]+fraction*(self.u_nodes[i+1]-self.u_nodes[i]),lo,hi)
        result=np.empty(len(s));active=np.arange(len(s))
        # Use parameter-width certification, not residual-only convergence.
        # A 4e-14 bracket is stricter than the scalar solver's 1e-13 xtol.
        for _ in range(16):
            if not len(active):break
            self.stats['batch_rounds']+=1
            x=u[active];aa=a[active];bb=base[active];tt=target[active]
            f=self._residual(x,aa,bb,tt)
            left=np.maximum(lo[active],x-2e-14)
            right=np.minimum(hi[active],x+2e-14)
            fl=self._residual(left,aa,bb,tt);fr=self._residual(right,aa,bb,tt)
            solved=(left<=right)&np.isfinite(fl)&np.isfinite(fr)&(fl<=0)&(fr>=0)
            result[active[solved]]=x[solved]
            remain=~solved;active=active[remain]
            if not len(active):break
            x=x[remain];f=f[remain]
            lo[active]=np.where(f<0,x,lo[active])
            hi[active]=np.where(f>=0,x,hi[active])
            speed=np.linalg.norm(self.curve.evaluate(x,1),axis=1)
            with np.errstate(divide='ignore',invalid='ignore'):
                proposed=x-f/speed
            good=np.isfinite(proposed)&(proposed>lo[active])&(proposed<hi[active])
            u[active]=np.where(good,proposed,(lo[active]+hi[active])*.5)
        for j in active:
            result[j]=self.curve.u_at_s(float(s[j]))
        self.stats['inverse_points']+=len(s)
        self.stats['scalar_fallbacks']+=len(active)
        return result

    def sample(self,s):
        u=self.inverse_many(s)
        xy=self.curve.evaluate(u);first=self.curve.evaluate(u,1);second=self.curve.evaluate(u,2)
        norm=np.linalg.norm(first,axis=1)
        if np.any(norm<1e-9):raise ValueError('degenerate spline derivative')
        tangent=first/norm[:,None]
        psi=np.arctan2(tangent[:,1],tangent[:,0])
        curvature=(first[:,0]*second[:,1]-first[:,1]*second[:,0])/norm**3
        return xy,tangent,psi,curvature

    def summary(self):
        return dict(self.stats,policy=POLICY,anchor_entries=len(self.anchors),
                    max_anchors=self.max_anchors,grid_spacing_m=self.spacing,
                    window=self.window_summary,reference_interpolation=False,
                    boundary_interpolation=False,old_curve_checks_reused=False)


def get_arc_lookup(table):
    lookup=getattr(table,'_fixed_arc_lookup',None)
    if lookup is None:
        lookup=FixedArcLookup(table.geometry.curve)
        table._fixed_arc_lookup=lookup
    return lookup
