"""A8 only: bypass the steering FIFO; keep original PID/rate/vehicle physics.

Prefilling both legacy FIFO slots with the current command makes the original
implementation consume that command on this tick. No global simulator edits.
"""
from argparse import Namespace
import numpy as np
from f1tenth_benchmarks.simulator.dynamics_simulator import DynamicsSimulator
from .dynamics import TracedDynamics
from .run_initialization import ValidatedContinuousSim

class NoDelayDynamics(DynamicsSimulator):
    def update_pose(self, raw_steer, vel):
        self.steer_buffer=np.full(self.steer_buffer_size,float(raw_steer))
        return super().update_pose(raw_steer,vel)

class NoDelayTracedDynamics(TracedDynamics):
    def update_pose(self, raw_steer, command):
        self.steer_buffer=np.full(self.steer_buffer_size,float(raw_steer))
        return super().update_pose(raw_steer,command)

class NoDelayContinuousSim(ValidatedContinuousSim):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.dynamics_simulator=NoDelayTracedDynamics(self.params)

def replay(rows,params,start_pose):
    """Uninstrumented original physics, current command replacing FIFO output."""
    plant=NoDelayDynamics(Namespace(**params));plant.reset(start_pose);count=0
    for row in rows:
        for tick in row['physics_ticks']:
            np.testing.assert_allclose(plant.state,tick['pre_state'],rtol=0,atol=1e-12)
            if tick['delayed_target_steering']!=tick['command'][0]:
                raise ValueError('A8 trace contains delayed steering')
            state=plant.update_pose(tick['command'][0],tick['pid_target_speed'])
            np.testing.assert_allclose(state,tick['post_state'],rtol=0,atol=1e-12);count+=1
        np.testing.assert_allclose(plant.state,row['physical_state'],rtol=0,atol=1e-12)
        np.testing.assert_array_equal(plant.steer_buffer,row['steering_buffer'])
    return dict(passed=True,physics_ticks=count,tolerance=1e-12,
                method='original_vehicle_PID_rate_limits_with_current_steering_command',steering_delay_s=0.)
