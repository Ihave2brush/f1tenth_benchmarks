"""Offline annular-corridor candidates; independent of old benchmark loaders."""
from dataclasses import dataclass
import math

import numpy as np
from scipy import ndimage, interpolate, optimize, integrate
from scipy.spatial import cKDTree

from .map_raster import FREE


@dataclass(frozen=True)
class ExtractionOptions:
    wall_sample_m: float = .1
    smoothing_rms_m: float = .01
    output_step_m: float = .05
    anchor_max_pixels: float = 2.
    anchor_heading_max_deg: float = 15.
    centering_smoothing_m: float = .3
    method: str = 'nearest_wall_midpoint_v1'

    def validate(self):
        if self.method not in ('nearest_wall_midpoint_v1','equidistant_periodic_v1'):
            raise ValueError('unsupported centerline method')
        for key, value in vars(self).items():
            if key == 'method':
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError('invalid extraction option: ' + key)


def corridor_loops(raster, anchor):
    """Select the anchor's 4-connected free annulus and exact grid interfaces.

    A degree-four boundary vertex, multiple holes or image-edge contact is
    rejected instead of guessing a branch or making a loop across the unknown.
    """
    pixel = raster.map_to_pixel(anchor)
    if not pixel.inside_map or raster.classify(anchor) != 'free':
        raise ValueError('anchor must be inside a free cell')
    labels, _ = ndimage.label(raster.occupancy == FREE)
    c, v = pixel.cell
    mask = labels == labels[v, c]
    if mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any():
        raise ValueError('unsupported_topology: corridor touches map edge')
    holes = ndimage.binary_fill_holes(mask) & ~mask
    _, count = ndimage.label(holes)
    if count != 1:
        raise ValueError('unsupported_topology: expected one enclosed nonfree island')
    # Build undirected grid-interface edges in bottom-left integer coordinates.
    up = np.zeros_like(mask); up[1:] = mask[:-1]
    down = np.zeros_like(mask); down[:-1] = mask[1:]
    left = np.zeros_like(mask); left[:, 1:] = mask[:, :-1]
    right = np.zeros_like(mask); right[:, :-1] = mask[:, 1:]
    adjacency = {}
    for other, endpoints in [
            (up, lambda x, y: ((x, y+1), (x+1, y+1))),
            (down, lambda x, y: ((x, y), (x+1, y))),
            (left, lambda x, y: ((x, y), (x, y+1))),
            (right, lambda x, y: ((x+1, y), (x+1, y+1)))]:
        for row, col in np.argwhere(mask & ~other):
            a, b = endpoints(int(col), raster.height - 1 - int(row))
            adjacency.setdefault(a, []).append(b)
            adjacency.setdefault(b, []).append(a)
    if any(len(neighbors) != 2 for neighbors in adjacency.values()):
        raise ValueError('unsupported_topology: touching or branching interface')
    remaining = set(adjacency)
    loops = []
    while remaining:
        start = min(remaining)
        previous, current = None, start
        loop = []
        while True:
            loop.append(current)
            remaining.remove(current)
            neighbors = adjacency[current]
            following = neighbors[0] if neighbors[0] != previous else neighbors[1]
            previous, current = current, following
            if current == start:
                break
        loops.append(raster.local_to_map(np.asarray(loop) * raster.resolution))
    if len(loops) != 2:
        raise ValueError('unsupported_topology: expected two boundary loops')
    loops.sort(key=lambda p: abs(signed_area(p)), reverse=True)
    return mask, loops


def signed_area(points):
    return float(np.sum(points[:, 0] * np.roll(points[:, 1], -1)
                        - np.roll(points[:, 0], -1) * points[:, 1]) / 2)


def resample_closed(points, step):
    closed = np.vstack((points, points[0]))
    s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))]
    query = np.linspace(0, s[-1], int(np.ceil(s[-1] / step)), endpoint=False)
    return np.column_stack([np.interp(query, s, closed[:, i]) for i in range(2)])


def nearest_segments(query, contour):
    # Nearest grid vertex: an interface edge has length exactly one pixel.
    # Adjacent segments plus nearest four vertices cover local corners/ties.
    tree = cKDTree(contour)
    _, vertices = tree.query(query, k=min(4, len(contour)))
    vertices = np.asarray(vertices).reshape(len(query), -1)
    indices = np.concatenate((vertices, (vertices - 1) % len(contour)), axis=1)
    a = contour[indices]; edge = contour[(indices + 1) % len(contour)] - a
    fraction = np.clip(np.sum((query[:, None] - a) * edge, axis=2)
                       / np.sum(edge * edge, axis=2), 0, 1)
    projected = a + fraction[:, :, None] * edge
    distances = np.sum((query[:, None] - projected)**2, axis=2)
    return projected[np.arange(len(query)), np.argmin(distances, axis=1)]


class ReferenceCurve:
    """Serialized periodic spline and physical arclength, no Frenet API yet."""
    def __init__(self, tck, start_u=0., closed=True):
        self.tck = (np.asarray(tck[0]), np.asarray(tck[1]), int(tck[2]))
        self.closed = bool(closed)
        if not self.closed and start_u != 0:raise ValueError('open curve must start at u=0')
        self.start_u = float(start_u) % 1.
        knots = np.unique(np.clip(self.tck[0], 0, 1))
        self.u_nodes = knots
        self.s_nodes = np.r_[0., np.cumsum([self.integral(a, b) for a, b in zip(knots[:-1], knots[1:])])]
        self.L = float(self.s_nodes[-1])
        self.start_s = self.arc(self.start_u)

    def evaluate(self, u, derivative=0):
        return np.asarray(interpolate.splev(u, self.tck, der=derivative)).T

    def integral(self, a, b):
        return float(integrate.fixed_quad(lambda u: np.linalg.norm(self.evaluate(u, 1), axis=1), a, b, n=16)[0])

    def arc(self, u):
        if u >= 1:
            return self.L
        i = max(0, min(len(self.u_nodes)-2, int(np.searchsorted(self.u_nodes, u, side='right')-1)))
        return float(self.s_nodes[i] + self.integral(self.u_nodes[i], u))

    def u_at_s(self, s):
        if not self.closed and not 0 <= float(s) <= self.L:raise ValueError('outside_reference')
        target = (float(s) + self.start_s) % self.L if self.closed else float(s)
        if not self.closed and target == self.L:return 1.
        i = max(0, min(len(self.u_nodes)-2, int(np.searchsorted(self.s_nodes, target, side='right')-1)))
        a, b = self.u_nodes[i:i+2]
        return optimize.brentq(lambda u: self.s_nodes[i] + self.integral(a, u) - target,
                               a, b, xtol=1e-13)

    def sample(self, s):
        u = np.array([self.u_at_s(value) for value in np.atleast_1d(s)])
        xy = self.evaluate(u); first = self.evaluate(u, 1); second = self.evaluate(u, 2)
        norm = np.linalg.norm(first, axis=1)
        if np.any(norm < 1e-9):
            raise ValueError('degenerate spline derivative')
        tangent = first / norm[:, None]
        psi = np.arctan2(tangent[:, 1], tangent[:, 0])
        kappa = (first[:, 0]*second[:, 1]-first[:, 1]*second[:, 0]) / norm**3
        return xy, tangent, psi, kappa

    def to_dict(self):
        return dict(degree=self.tck[2], knots=self.tck[0].tolist(),
                    coefficients=self.tck[1].tolist(), parameter_range=[0., 1.],
                    periodic=self.closed, start_u=self.start_u, L_m=self.L,
                    arclength_u=self.u_nodes.tolist(), arclength_s_m=self.s_nodes.tolist(),
                    integration=dict(method='gauss_legendre_per_knot', order=16,
                                     inverse_root_xtol=1e-13))


def ray_boundary(raster, point, direction, corridor=None):
    """First exact free/nonfree grid face using grid traversal, no step bias."""
    q = raster.map_to_local(point) / raster.resolution
    world_direction = np.asarray(direction, float)
    if world_direction.shape != (2,) or not np.isfinite(world_direction).all() or np.linalg.norm(world_direction) == 0:
        raise ValueError('invalid ray direction')
    world_direction = world_direction / np.linalg.norm(world_direction)
    direction = world_direction @ raster.rotation / raster.resolution
    if raster.classify(point) != 'free':
        return None, None, 'missing', 'center_not_free'
    cell = np.floor(q).astype(int)
    sign = np.sign(direction).astype(int)
    delta = np.full(2, np.inf); t = np.full(2, np.inf)
    active = direction != 0
    delta[active] = 1 / np.abs(direction[active])
    face = cell + (sign > 0)
    t[active] = (face[active] - q[active]) / direction[active]
    for _ in range(raster.width + raster.height + 4):
        distance = float(t.min())
        axes = np.flatnonzero(np.abs(t - distance) <= 1e-10)
        # At a corner test both incident cells; never jump across a blocked corner.
        candidates = []
        for axis in axes:
            neighbor = cell.copy(); neighbor[axis] += sign[axis]
            candidates.append(neighbor)
        following = cell.copy(); following[axes] += sign[axes]
        candidates.append(following)
        for neighbor in candidates:
            col, row = neighbor
            if not (0 <= col < raster.width and 0 <= row < raster.height):
                return None, None, 'missing', 'outside_map'
            v = raster.height - 1 - row
            value = int(raster.occupancy[v, col])
            if value != FREE:
                kind = 'occupied' if value == 1 else 'unknown'
                return np.asarray(point) + distance*world_direction, distance, kind, None
            if corridor is not None and not corridor[v, col]:
                return None, None, 'missing', 'wrong_corridor'
        cell = following
        t[axes] += delta[axes]
    return None, None, 'missing', 'no_boundary'


def distance_balanced_seed(raster, corridor):
    """Extract the single closed equal-distance contour between the two walls.

    Distances use occupied pixel centres at the map resolution. This is a
    raster-derived reference seed, not an exact wall-distance certificate.
    Its complete connected contour avoids independent nearest-wall pair jumps.
    """
    filled = ndimage.binary_fill_holes(corridor)
    island = filled & ~corridor
    inner_distance = ndimage.distance_transform_edt(~island)*raster.resolution
    outer_distance = ndimage.distance_transform_edt(filled)*raster.resolution
    # Resolve exact zero vertices consistently without grid-edge duplicates.
    field = inner_distance-outer_distance+1e-10*raster.resolution
    corners = np.stack((field[:-1,:-1],field[:-1,1:],field[1:,1:],field[1:,:-1]))
    active = (corners.min(axis=0)<0)&(corners.max(axis=0)>0)
    adjacency, positions = {}, {}
    for row, col in np.argwhere(active):
        row, col = int(row), int(col)
        edges = [('h',row,col,(row,col),(row,col+1)),
                 ('v',row,col+1,(row,col+1),(row+1,col+1)),
                 ('h',row+1,col,(row+1,col),(row+1,col+1)),
                 ('v',row,col,(row,col),(row+1,col))]
        crossings = []
        for axis, v, c, a, b in edges:
            fa, fb = field[a], field[b]
            if (fa<0)==(fb<0):
                continue
            key = (axis,v,c)
            fraction = float(fa/(fa-fb))
            pixel = np.array(a)+fraction*(np.array(b)-a)
            positions[key] = raster.pixel_to_map(pixel[1],pixel[0])
            crossings.append(key)
        if len(crossings)!=2:
            raise ValueError('unsupported_topology: ambiguous equal-distance contour')
        a,b = crossings
        adjacency.setdefault(a,[]).append(b);adjacency.setdefault(b,[]).append(a)
    if not adjacency or any(len(neighbors)!=2 for neighbors in adjacency.values()):
        raise ValueError('unsupported_topology: unclosed equal-distance contour')
    start = min(adjacency);previous,current = None,start;ordered=[];visited=set()
    while current not in visited:
        visited.add(current);ordered.append(positions[current])
        neighbors = adjacency[current]
        following = neighbors[0] if neighbors[0]!=previous else neighbors[1]
        previous,current = current,following
    if current!=start or len(visited)!=len(adjacency):
        raise ValueError('unsupported_topology: multiple equal-distance contours')
    return np.asarray(ordered)


def refine_centerline(raster, corridor, seed, options):
    """Regularize the connected distance contour at a physical length scale.

    Uniform arclength sampling avoids assigning extra weight to pixel corners.
    Periodic Gaussian regularization suppresses raster-scale zigzags without
    changing the measured boundaries or assuming a uniform corridor width.
    """
    step = min(options.wall_sample_m, 2*raster.resolution)

    def free_polyline(points):
        checked = resample_closed(points, raster.resolution/4)
        local = raster.map_to_local(checked)/raster.resolution
        cells = np.floor(local).astype(int)
        cols, rows = cells[:, 0], raster.height-1-cells[:, 1]
        if np.any((cols<0)|(cols>=raster.width)|(rows<0)|(rows>=raster.height)):
            return False
        return bool(corridor[rows, cols].all())

    def constrained_update(before, proposed):
        # Averages can cut across an inside corner although every input point is
        # free. Backtrack the entire update to preserve the selected corridor.
        for exponent in range(13):
            candidate = before+(proposed-before)*(.5**exponent)
            if free_polyline(candidate):
                return candidate
        raise ValueError('centerline update cannot preserve free corridor')

    def regularize(points):
        sampled = resample_closed(points, step)
        if not free_polyline(sampled):
            raise ValueError('centerline resampling leaves free corridor')
        actual_step = np.linalg.norm(np.roll(points, -1, axis=0)-points, axis=1).sum()/len(sampled)
        smoothed = ndimage.gaussian_filter1d(sampled, options.centering_smoothing_m/actual_step,
                                            axis=0, mode='wrap')
        return constrained_update(sampled, smoothed)

    points = regularize(seed)
    if any(raster.classify(point) != 'free' for point in points):
        raise ValueError('centerline regularization leaves free corridor')
    return points, dict(method='equidistant_periodic_v1', seed_samples=len(seed),
                        smoothing_length_m=options.centering_smoothing_m,
                        distance_method='euclidean_raster_centres')


def build_geometry(raster, options, anchor, heading):
    options.validate()
    mask, loops = corridor_loops(raster, anchor)
    if options.method == 'equidistant_periodic_v1':
        middle = distance_balanced_seed(raster, mask)
    else:
        inner = resample_closed(loops[1], options.wall_sample_m)
        middle = (inner+nearest_segments(inner, loops[0]))/2
    keep = np.r_[True, np.linalg.norm(np.diff(middle, axis=0), axis=1) > 1e-6]
    middle = middle[keep]
    if options.method == 'equidistant_periodic_v1':
        middle, refinement = refine_centerline(raster, mask, middle, options)
    else:
        refinement = dict(method=options.method)
    # Preserve direction from hint rather than relying on contour ordering.
    nearest = np.argmin(np.linalg.norm(middle - anchor, axis=1))
    tangent = middle[(nearest+1) % len(middle)] - middle[(nearest-1) % len(middle)]
    if np.dot(tangent, [math.cos(heading), math.sin(heading)]) < 0:
        middle = middle[::-1]
    data = np.vstack((middle, middle[0]))
    tck, _ = interpolate.splprep(data.T, s=len(middle)*options.smoothing_rms_m**2, per=True, k=3)
    initial = ReferenceCurve(tck)
    dense_u = np.linspace(0, 1, max(4000, len(middle)*2), endpoint=False)
    nearest = np.argmin(np.linalg.norm(initial.evaluate(dense_u)-anchor, axis=1))
    du = 1 / len(dense_u)
    def objective(u):
        return float(np.sum((initial.evaluate(u % 1)-anchor)**2))
    result = optimize.minimize_scalar(objective, bounds=(dense_u[nearest]-du, dense_u[nearest]+du), method='bounded',
                                      options={'xatol': 1e-13})
    curve = ReferenceCurve(tck, result.x)
    anchor_xy, tangent, _, _ = curve.sample([0.])
    shift = float(np.linalg.norm(anchor_xy[0]-anchor))
    angle = float(math.acos(np.clip(np.dot(tangent[0], [math.cos(heading), math.sin(heading)]), -1, 1)))
    if shift > options.anchor_max_pixels*raster.resolution or angle > math.radians(options.anchor_heading_max_deg):
        raise ValueError('anchor_mismatch: distance=%g m, heading=%g deg' % (shift, math.degrees(angle)))
    step = min(options.output_step_m, .1, 2*raster.resolution)
    s = np.linspace(0, curve.L, int(np.ceil(curve.L / step)), endpoint=False)
    xy, tangent, psi, kappa = curve.sample(s)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    boundaries = []
    for position, n in zip(xy, normal):
        left, dl, lk, lr = ray_boundary(raster, position, n, mask)
        right, dr, rk, rr = ray_boundary(raster, position, -n, mask)
        boundaries.append(dict(left=left, right=right, d_left=dl,
                               d_right=None if dr is None else -dr,
                               left_kind=lk, right_kind=rk,
                               valid=lr is None and rr is None, reason=lr or rr or ''))
    return dict(curve=curve, s=s, xy=xy, psi=psi, kappa=kappa,
                boundaries=boundaries, corridor=mask, loops=loops,
                anchor=dict(input_xy=list(map(float, anchor)), input_heading_rad=float(heading),
                            projected_xy=anchor_xy[0].tolist(), displacement_m=shift,
                            heading_error_rad=angle),
                options=dict(vars(options), method=refinement['method']), refinement=refinement)
