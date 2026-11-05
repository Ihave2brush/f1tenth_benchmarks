"""V5-inspired width fractions with C2 interpolation and real pose endpoints.

This is an experimental direct proposal, not the unchanged PR #5 V5. It
interpolates road-fraction sites, then adds a whole-horizon Hermite slope
correction before matching endpoint slopes. It
does not optimize or certify the curve; the shared validator must accept it.
"""
import numpy as np
from scipy.interpolate import BSpline, make_interp_spline


class _Evaluation:
    def __init__(self, basis, x, nu):
        self.basis, self.x, self.nu = basis, x, nu

    def __matmul__(self, coefficients):
        return BSpline(self.basis.t, coefficients, self.basis.k)(self.x, nu=self.nu)


class DirectSplineBasis:
    """Direct-only linear evaluation, avoiding a dense N-by-N identity basis."""
    def __init__(self, knots, degree):
        self.t, self.k = np.asarray(knots), degree

    def __call__(self, x, nu=0):
        return _Evaluation(self, x, nu)


def width_fraction_spline(dense, sections, fraction0, fraction, d0, slope0, target):
    dense = np.asarray(dense)
    u = dense/dense[-1]
    blend = 3*u*u-2*u*u*u
    fractions = (1-blend)*fraction0+blend*fraction
    lo = np.asarray([v['d_min'] for v in sections])
    hi = np.asarray([v['d_max'] for v in sections])
    preferred = lo+fractions*(hi-lo)
    preferred[0], preferred[-1] = d0, target
    raw = make_interp_spline(dense, preferred, k=3)
    # Distribute pose correction over the horizon instead of forcing a sharp
    # boundary layer in the first/last interpolation cell. End values stay put.
    correction = dense[-1]*((slope0-float(raw(0., nu=1)))*(u**3-2*u*u+u)
                           -float(raw(dense[-1], nu=1))*(u**3-u*u))
    preferred += correction
    preferred[0], preferred[-1] = d0, target
    curve = make_interp_spline(dense, preferred, k=3,
                              bc_type=([(1, slope0)], [(1, 0.)]))
    return DirectSplineBasis(curve.t, curve.k), np.asarray(curve.c)
