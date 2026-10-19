"""Weighted local identification with an explicitly labelled actuator prior.

Paper P/Q/h/kernel and five-coefficient regressors are reported. The executable
model uses original physical integration as a prior, plus an affine residual,
and three actuator-memory states. Periodic templates never enter identification.
"""
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from f1tenth_benchmarks.simulator.dynamic_models import vehicle_dynamics_st, pid
from .database import sha256
from .dynamics import acceleration_target_speed


def augmented(state, physical, buffer):
    buffer = np.asarray(buffer, float)
    if buffer.shape != (2,):
        raise ValueError('B3 requires the initialized two-entry steering buffer')
    return np.r_[state, physical[2], buffer]


def physical_step(z, u, params, dt, smooth=False, return_physical=False):
    """Physical prior; optional differentiable target servo for QP prediction.

    Original bang-bang PID has locally zero command sensitivity almost everywhere.
    smooth=True replaces only prediction's steering rate by clipped proportional
    target tracking at gain 1/physics_dt. Actual plant/shadow still use original PID.
    Low-speed beta inversion is regularized only in this prediction mode.
    """
    vx, vy = z[:2]
    speed = np.hypot(vx, vy) * (-1 if vx < 0 else 1)
    beta = np.arctan2(vy, max(vx, .1)) if smooth else (np.arctan2(vy, vx) if speed > 1e-8 else 0.)
    if smooth and vx < .1:
        speed = vx/np.cos(beta)
    physical = np.array([0., 0., z[6], speed, 0., z[2], beta])
    buffer = z[7:9].copy()
    for _ in range(round(dt/params['timestep'])):
        delayed = buffer[-1]
        buffer = np.r_[u[0], buffer[:-1]]
        target = acceleration_target_speed(u[1], physical[3], dict(
            max_v=params['v_max'], min_v=params['v_min'], max_a=params['a_max']))
        a, sv = pid(target, delayed, physical[3], physical[2], params['sv_max'],
                    params['a_max'], params['v_max'], params['v_min'])
        if smooth:
            sv = np.clip((delayed-physical[2])/params['timestep'], params['sv_min'], params['sv_max'])
        derivative = vehicle_dynamics_st(physical, np.array([sv, a]), *[
            params[k] for k in ('mu', 'C_Sf', 'C_Sr', 'lf', 'lr', 'h', 'm', 'I',
                               's_min', 's_max', 'sv_min', 'sv_max', 'v_switch',
                               'a_max', 'v_min', 'v_max')])
        physical += params['timestep']*derivative
    result = np.array([physical[3]*np.cos(physical[6]), physical[3]*np.sin(physical[6]),
                       physical[5], physical[2], *buffer])
    return (result, physical) if return_physical else result


def geometric_kinematic_step(z, physical_end, geometry):
    """Integrate original body motion, then query the unchanged core frame.

    This engineering variant replaces the paper's .1s Euler pose update.
    Physical integration starts in the initial vehicle body frame. A numerical
    projection is only a prediction query, not a global planning permission.
    """
    ref = geometry.reference(z[4])
    tangent = np.asarray(ref['tangent'])
    start_xy = np.asarray(ref['xy'])+z[5]*np.array([-tangent[1], tangent[0]])
    yaw = ref['psi']+z[3]
    rotate = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    xy = start_xy+rotate@physical_end[:2]
    projected = geometry.to_frenet(xy, yaw+physical_end[4], s_hint=z[4], search_window_m=3.)
    if not projected['valid']:
        raise ValueError('geometric prediction projection: '+str(projected['reason']))
    ds = (projected['s_wrapped']-z[4]+geometry.curve.L/2)%geometry.curve.L-geometry.curve.L/2
    return np.array([projected['e_psi'], z[4]+ds, projected['d']])


def kinematic_step(x, geometry, dt):
    kappa = geometry.reference(x[4])['kappa']
    jacobian = 1-kappa*x[5]
    if jacobian < .2:
        raise ValueError('singular Frenet linearization')
    ds = (x[0]*np.cos(x[3])-x[1]*np.sin(x[3]))/jacobian
    return x[3:6]+dt*np.array([x[2]-kappa*ds, ds,
                              x[0]*np.sin(x[3])+x[1]*np.cos(x[3])])


class LocalModel:
    def __init__(self, database, geometry, params, ridge, prediction_value_mode='smoothed_servo',
                 training_source_ids=None):
        self.database, self.geometry, self.params = database, geometry, params
        self.dt, self.ridge = database.dt, ridge
        if prediction_value_mode not in ('smoothed_servo', 'original_pid', 'original_pid_geometric',
                                          'original_pid_geometric_secant',
                                          'smoothed_servo_geometric'):
            raise ValueError('unknown prediction value mode')
        self.smooth_nominal = prediction_value_mode.startswith('smoothed_servo')
        self.geometric_pose = '_geometric' in prediction_value_mode
        self.secant_sensitivities = prediction_value_mode.endswith('_secant')
        root = Path(database.metadata['source_directory'])
        for name, digest in database.metadata['source_hashes'].items():
            if sha256(root/name) != digest:
                raise ValueError('identification raw source hash mismatch: '+name)
        rows = [json.loads(s) for s in (root/'samples.jsonl').read_text().splitlines()]
        self.source_ids = np.unique(np.concatenate([lap.sample_ids[:-1] for lap in database.laps]))
        if training_source_ids is not None:
            requested = np.asarray(training_source_ids)
            if (requested.ndim != 1 or requested.dtype.kind not in 'iu'
                    or len(np.unique(requested)) != len(requested)
                    or not np.isin(requested, self.source_ids).all() or len(requested) < 80):
                raise ValueError('training IDs must be at least 80 unique measured transitions')
            self.source_ids = np.sort(requested)
        states, controls, targets, residuals = [], [], [], []
        for index in self.source_ids:
            before, after = rows[index], rows[index+1]
            z = augmented(before['paper_state'], before['physical_state'], before['steering_buffer'])
            ticks = after['physics_ticks']
            a = sum(t['applied_acceleration']*t['dt'] for t in ticks)/self.dt
            u = np.array([after['command'][0], a])
            target = np.asarray(after['paper_state'][:3])
            states.append(z); controls.append(u); targets.append(target)
            residuals.append(target-physical_step(z, u, params, self.dt, smooth=self.smooth_nominal)[:3])
        self.states, self.controls = np.array(states), np.array(controls)
        self.targets, self.residuals = np.array(targets), np.array(residuals)
        self.features = np.column_stack([self.states[:, :3], self.states[:, 6:9], self.controls])
        self.distance_diagonal = np.array([.1, 1., 1.])

    def identify(self, z, u):
        start = perf_counter()
        # Paper ||d||_Q^2 = d.T Q.T Q d. Kernel argument is squared distance / h.
        distance = np.sum(((self.states[:, :3]-z[:3])*self.distance_diagonal)**2, axis=1)
        idx = np.lexsort((self.source_ids, distance))[:80]
        argument = distance[idx]/10.
        weight = .75*np.maximum(1-argument**2, 0.)
        if np.count_nonzero(weight) < 9:
            raise ValueError('insufficient positively weighted identification data')
        query = np.r_[z[:3], z[6:9], u]
        design = np.column_stack([self.features[idx]-query, np.ones(len(idx))])
        weighted = design*np.sqrt(weight[:, None])
        # Isotropic ridge includes intercept; weak residuals remain near the prior.
        coeff = np.linalg.solve(weighted.T@weighted+self.ridge*np.eye(9),
                                design.T@(weight[:, None]*self.residuals[idx]))
        paper = []
        for channel, input_index in enumerate([1, 0, 0]):
            f = np.column_stack([self.states[idx, :3], self.controls[idx, input_index], np.ones(len(idx))])
            wf = f*np.sqrt(weight[:, None])
            theta = np.linalg.lstsq(wf, self.targets[idx, channel]*np.sqrt(weight), rcond=1e-8)[0]
            singular = np.linalg.svd(wf, compute_uv=False)
            paper.append(dict(rank=int(np.linalg.matrix_rank(wf, tol=1e-8)), coefficients=theta.tolist(),
                              singular_values=singular.tolist(),
                              condition_number=float(singular[0]/max(singular[-1], 1e-300)),
                              weighted_rmse=float(np.sqrt(np.average((f@theta-self.targets[idx, channel])**2, weights=weight)))))
        return coeff, query, dict(source_ids=self.source_ids[idx].tolist(),
                                  measured_only=True, points=80, kernel='Epanechnikov',
                                  kernel_argument='squared_Q_distance / h', bandwidth=10.,
                                  residual_rank=int(np.linalg.matrix_rank(weighted, tol=1e-8)),
                                  residual_ridge=self.ridge, paper_regressors=paper,
                                  prior_residual_rmse=np.sqrt(np.average(self.residuals[idx]**2, axis=0, weights=weight)).tolist(),
                                  coefficients=coeff.tolist(), feature_origin=query.tolist(),
                                  identification_time_s=perf_counter()-start)

    def evaluate(self, z, u, fit=None, surrogate=False):
        result = np.empty(9)
        prior, physical = physical_step(z, u, self.params, self.dt,
                                       smooth=surrogate or getattr(self, 'smooth_nominal', False), return_physical=True)
        result[:3], result[6:9] = prior[:3], prior[3:]
        result[3:6] = (geometric_kinematic_step(z, physical, self.geometry)
                       if getattr(self, 'geometric_pose', False) else kinematic_step(z, self.geometry, self.dt))
        if fit is not None:
            coeff, origin = fit
            result[:3] += np.r_[np.r_[z[:3], z[6:9], u]-origin, 1.]@coeff
        return result

    def linearize(self, z, u):
        coeff, origin, diagnostic = self.identify(z, u)
        fit = coeff, origin
        value = self.evaluate(z, u, fit)
        aa, bb = np.empty((9, 9)), np.empty((9, 2))
        secant = getattr(self, 'secant_sensitivities', False)
        # One original actuator quantum for steering/memory; finite changes
        # across switching cells. These are secant approximations, not Jacobians.
        state_steps = [.02, .01, .05, .005, .01, .005, .032, .032, .032]
        for i in range(9):
            step = state_steps[i] if secant else (1e-5 if i != 4 else 1e-3)
            perturb = np.eye(9)[i]*step
            aa[:, i] = (self.evaluate(z+perturb, u, fit, surrogate=not secant)-self.evaluate(z-perturb, u, fit, surrogate=not secant))/(2*step)
        for i in range(2):
            step = [.032, .1][i] if secant else 1e-5
            perturb = np.eye(2)[i]*step
            bb[:, i] = (self.evaluate(z, u+perturb, fit, surrogate=not secant)-self.evaluate(z, u-perturb, fit, surrogate=not secant))/(2*step)
        diagnostic['linearization'] = ('smoothed_servo_nominal_and_sensitivities' if getattr(self, 'smooth_nominal', False)
                                      else 'original_PID_nominal_value_with_regularized_surrogate_sensitivities')
        diagnostic['pose_update'] = ('original_substeps_and_core_projection' if getattr(self, 'geometric_pose', False)
                                    else 'paper_frenet_euler_0.1s')
        if secant:
            diagnostic.update(linearization='original_PID_finite_amplitude_secants_not_exact_Jacobian',
                              state_secant_steps=state_steps, input_secant_steps=[.032, .1])
        return aa, bb, value-aa@z-bb@u, diagnostic
