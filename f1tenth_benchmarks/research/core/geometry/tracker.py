"""ROS-independent timestamped Frenet tracking with explicit recovery epochs."""
from dataclasses import dataclass,asdict
import math

import numpy as np

from .frenet import adapt_pose


@dataclass(frozen=True)
class TrackingOptions:
    max_pose_age_s: float = .1
    max_dt_s: float = .25
    max_speed_mps: float = 15.
    max_acceleration_mps2: float = 20.
    position_slack_m: float = .15
    progress_slack_m: float = .25
    heading_tolerance_rad: float = 1.2

    def validate(self):
        for name,value in asdict(self).items():
            if isinstance(value,bool) or not np.isfinite(value) or value<=0:
                raise ValueError('invalid tracking option: '+name)
        if self.heading_tolerance_rad>=math.pi:
            raise ValueError('heading tolerance must be less than pi')


class FrenetTracker:
    """Commit only accepted poses. Any rejected pose suspends until reset.

    History may resolve a competitive projection only when exactly one local
    candidate satisfies motion and heading gates. Static queries stay strict.
    """
    def __init__(self,geometry,options=None,offset=(0.,0.,0.),
                 source_reference='base_link',offset_calibrated=False):
        self.geometry=geometry;self.options=options or TrackingOptions();self.options.validate()
        if np.asarray(offset).shape!=(3,) or not np.isfinite(offset).all():
            raise ValueError('invalid pose offset')
        if not isinstance(source_reference,str) or not source_reference:
            raise ValueError('source_reference is required')
        self.offset=tuple(offset);self.source_reference=source_reference
        self.offset_calibrated=bool(offset_calibrated)
        self.epoch=0;self.state=None;self.needs_reset=False

    def reset(self):
        self.state=None;self.needs_reset=False;self.epoch+=1

    def replace_geometry(self,geometry):
        self.geometry=geometry;self.reset()

    def _result(self):
        return dict(valid=False,reason=None,geometry_id=self.geometry.geometry_id,frame_id='map',
                    epoch=self.epoch,needs_reset=self.needs_reset,s_wrapped=None,s_unwrapped=None,
                    d=None,e_psi=None,wrap_count=None,selection_mode=None,
                    planning_allowed=False,domain_verified=False,
                    source_reference=self.source_reference,offset_calibrated=self.offset_calibrated)

    def _reject(self,reason,**details):
        self.needs_reset=True
        return dict(self._result(),reason=reason,needs_reset=True,**details)

    def update_tf(self,transform,*,now,geometry_id):
        """Duck-typed TransformStamped input; no ROS installation is required.

        Use header.stamp, not receipt time. The child frame must identify the
        configured physical source point, e.g. base_link with an explicit offset.
        """
        if now is None or geometry_id is None:
            return self._reject('missing_tf_context')
        try:
            if transform.child_frame_id!=self.source_reference:
                return self._reject('source_frame_mismatch')
            stamp=transform.header.stamp
            if (not isinstance(stamp.sec,int) or not isinstance(stamp.nanosec,int)
                or isinstance(stamp.sec,bool) or isinstance(stamp.nanosec,bool)
                or not 0<=stamp.nanosec<1000000000):
                return self._reject('invalid_tf_timestamp')
            timestamp=stamp.sec+stamp.nanosec*1e-9
            translation=transform.transform.translation;rotation=transform.transform.rotation
            q=np.asarray([rotation.x,rotation.y,rotation.z,rotation.w],float)
            norm=np.linalg.norm(q)
            if not np.isfinite(q).all() or not np.isfinite(norm) or norm<1e-9:
                return self._reject('invalid_tf_rotation')
            x,y,z,w=q/norm
            yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
            result=self.update([translation.x,translation.y],yaw,timestamp,now=now,
                frame_id=transform.header.frame_id,geometry_id=geometry_id)
            return dict(result,pose_input='tf_transform')
        except (AttributeError,TypeError,ValueError):
            return self._reject('invalid_tf')

    def update(self,xy,yaw,timestamp,*,now=None,frame_id='map',geometry_id=None,signed_speed=None):
        """Use TF's source timestamp and a matching clock for `now`.

        Replay may omit `now`, but freshness_verified then remains false.
        A pose's physical reference and calibrated offset are constructor data.
        """
        if self.needs_reset:
            return dict(self._result(),reason='reinitialization_required')
        if geometry_id is not None and geometry_id!=self.geometry.geometry_id:
            return self._reject('geometry_mismatch')
        try:
            pose=adapt_pose(xy,yaw,timestamp,offset=self.offset,frame_id=frame_id)
        except (ValueError,TypeError):
            return self._reject('frame_mismatch' if frame_id!='map' else 'nonfinite_input')
        point=np.asarray(pose['xy']);timestamp=pose['timestamp'];yaw=pose['yaw']
        if now is not None:
            if not np.isfinite(now):
                return self._reject('nonfinite_input')
            age=float(now)-timestamp
            if age<0:
                return self._reject('future_timestamp')
            if age>self.options.max_pose_age_s:
                return self._reject('stale_pose',pose_age_s=age)
        if signed_speed is not None and (not np.isfinite(signed_speed) or abs(signed_speed)>self.options.max_speed_mps):
            return self._reject('invalid_speed')
        if self.state is None:
            selected=self.geometry.to_frenet(point,yaw)
            if not selected['valid']:
                return self._reject(selected['reason'])
            if abs(selected['e_psi'])>self.options.heading_tolerance_rad:
                return self._reject('heading_mismatch')
            unwrapped=selected['s_wrapped'];initial=unwrapped;mode='global_initialization'
        else:
            previous=self.state;dt=timestamp-previous['timestamp']
            if dt<=0:
                return self._reject('nonmonotonic_timestamp')
            if dt>self.options.max_dt_s:
                return self._reject('pose_gap',dt_s=dt)
            displacement=point-np.asarray(previous['xy'])
            if np.linalg.norm(displacement)>self.options.max_speed_mps*dt+self.options.position_slack_m:
                return self._reject('localization_jump')
            ref=self.geometry.reference(previous['s_wrapped'])
            if signed_speed is None:
                predicted=float(np.dot(displacement,ref['tangent']))/previous['jacobian']
            else:
                predicted=float(signed_speed)*dt*math.cos(previous['e_psi'])/previous['jacobian']
            progress_gate=(self.options.progress_slack_m+
                           .5*self.options.max_acceleration_mps2*dt**2/.2)
            maximum_step=self.options.max_speed_mps*dt/.2+self.options.progress_slack_m
            if self.geometry.curve.closed and maximum_step>=self.geometry.curve.L/2:
                return self._reject('ambiguous_wrap')
            diagnostics=self.geometry.projection_candidates(point,yaw)
            if not diagnostics['valid']:
                return self._reject(diagnostics['reason'])
            candidates=diagnostics['candidates'];minimum=candidates[0]['distance'];eligible=[]
            for candidate in candidates:
                if candidate['distance']>minimum+self.geometry.raster.resolution*.5:
                    continue
                delta=candidate['s']-previous['s_wrapped']
                if self.geometry.curve.closed:delta=wrap_progress(delta,self.geometry.curve.L)
                if (candidate['valid'] and abs(candidate['e_psi'])<=self.options.heading_tolerance_rad
                    and abs(delta)<=maximum_step and abs(delta-predicted)<=progress_gate):
                    eligible.append((candidate,delta))
            if len(eligible)>1:
                return self._reject('ambiguous_projection',eligible_candidates=len(eligible))
            if not eligible:
                if not any(c['valid'] for c in candidates):
                    reason=candidates[0]['reason']
                elif not any(c['valid'] and abs(c['e_psi'])<=self.options.heading_tolerance_rad for c in candidates):
                    reason='heading_mismatch'
                else:
                    reason='progress_jump'
                return self._reject(reason)
            candidate,delta=eligible[0]
            selected=dict(candidate,s_wrapped=candidate['s'])
            unwrapped=previous['s_unwrapped']+delta;initial=previous['initial_s'];mode='continuity'
        # Only this block changes the accepted pose/progress state.
        self.state=dict(timestamp=timestamp,xy=point.tolist(),yaw=yaw,
                        s_wrapped=selected['s_wrapped'],s_unwrapped=unwrapped,initial_s=initial,
                        jacobian=selected['jacobian'],d=selected['d'],e_psi=selected['e_psi'])
        return dict(self._result(),valid=True,reason=None,s_wrapped=selected['s_wrapped'],
                    s_unwrapped=unwrapped,d=selected['d'],e_psi=selected['e_psi'],
                    jacobian=selected['jacobian'],projection_xy=selected['projection_xy'],
                    timestamp=timestamp,xy=point.tolist(),yaw=yaw,selection_mode=mode,
                    inside_track=True,inside_valid_domain=True,
                    domain_membership_method='history_constrained_projection',
                    wrap_count=math.floor(unwrapped/self.geometry.curve.L)-math.floor(initial/self.geometry.curve.L) if self.geometry.curve.closed else 0,
                    freshness_verified=now is not None,version_verified=geometry_id is not None)


def wrap_progress(delta,length):
    return float((delta+length/2)%length-length/2)
