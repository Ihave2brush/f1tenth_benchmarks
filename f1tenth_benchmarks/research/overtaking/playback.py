"""Loopback-only recorded-session viewer; never runs simulation or planning."""
import argparse
from bisect import bisect_right
from functools import lru_cache
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

import numpy as np

from ..core.geometry.viewer import MapInspector
from .track_data import TrackData


class PlaybackSession:
    def __init__(self, directory, track):
        self.root = Path(directory)
        self.metadata = json.loads((self.root/'metadata.json').read_text())
        self.summary = json.loads((self.root/'summary.json').read_text())
        if self.metadata.get('schema_version') != 1 or self.summary.get('schema_version') != 1:
            raise ValueError('unsupported session schema')
        source = self.metadata.get('track', {})
        if (self.metadata.get('geometry_id') != track.geometry.geometry_id or
                self.metadata.get('frame_id') != 'map' or
                source.get('file_hashes') != track.provenance['file_hashes'] or
                source.get('source_hashes') != track.provenance['source_hashes'] or
                source.get('manifest_sha256') != track.provenance['manifest_sha256']):
            raise ValueError('session geometry/source mismatch')
        self.track = track
        self.inspector = MapInspector(track.geometry)
        self.index = dict(states=[], plans=[])
        self.state_times, self.trail = [], []
        self.plan_headers = []
        # Rebuild byte ranges from complete records. This also detects a stale
        # or damaged saved index; source artifacts are never rewritten.
        for name in self.index:
            with (self.root/(name+'.jsonl')).open('rb') as stream:
                while True:
                    offset = stream.tell()
                    raw = stream.readline()
                    if not raw:
                        break
                    row = json.loads(raw)
                    self.index[name].append(dict(byte_offset=offset, byte_length=len(raw)))
                    if name == 'states':
                        step = len(self.state_times)
                        if row['control_step'] != step or not math.isfinite(row['sim_time_s']):
                            raise ValueError('invalid recorded state order')
                        if self.state_times and row['sim_time_s'] <= self.state_times[-1]:
                            raise ValueError('non-increasing recorded time')
                        self.state_times.append(row['sim_time_s'])
                        self.trail.append(self._pixel(row['pose'][:2]))
                    else:
                        self.plan_headers.append({key: row[key] for key in (
                            'plan_id', 'source_control_step', 'source_sim_time_s',
                            'available_control_step', 'available_sim_time_s', 'geometry_id')})
        if not self.state_times or len(self.state_times) != self.summary['state_count']:
            raise ValueError('incomplete session state count')
        if len(self.plan_headers) != self.summary['plan_count']:
            raise ValueError('incomplete session plan count')
        self.origin_s = self.state_times[0]
        self.duration_s = self.state_times[-1]-self.origin_s
        if self.metadata['playback_origin_s'] != self.origin_s:
            raise ValueError('playback origin mismatch')
        for i, plan in enumerate(self.plan_headers):
            source_step, available_step = plan['source_control_step'], plan['available_control_step']
            if (plan['plan_id'] != i or type(source_step) is not int or type(available_step) is not int or
                    not 0 <= source_step < len(self.state_times) or available_step < source_step or
                    not math.isfinite(plan['available_sim_time_s']) or
                    plan['source_sim_time_s'] != self.state_times[source_step] or
                    plan['available_sim_time_s'] < plan['source_sim_time_s'] or
                    plan['geometry_id'] != self.metadata['geometry_id']):
                raise ValueError('invalid plan source/availability')
        self.index_status = 'rebuilt'
        try:
            saved = json.loads((self.root/'playback_index.json').read_text())
            if saved['schema_version'] == 1 and all(
                    len(saved['streams'][name]) == len(self.index[name]) and all(
                        all(old[k] == new[k] for k in new)
                        for old, new in zip(saved['streams'][name], self.index[name]))
                    for name in self.index):
                self.index_status = 'verified'
        except (OSError, ValueError, KeyError, TypeError):
            pass

    def _pixel(self, xy):
        if len(xy) != 2 or any(v is None or not math.isfinite(v) for v in xy):
            return None
        return self.inspector.pixels(xy)

    @lru_cache(maxsize=32)
    def record(self, name, index):
        item = self.index[name][index]
        with (self.root/(name+'.jsonl')).open('rb') as stream:
            stream.seek(item['byte_offset'])
            return json.loads(stream.read(item['byte_length']))

    def map_data(self):
        left, right = [], []
        for row in self.track.boundary_samples:
            if row['valid'] == '1':
                left.append([float(row['left_x_m']), float(row['left_y_m'])])
                right.append([float(row['right_x_m']), float(row['right_y_m'])])
        raster = self.track.geometry.raster
        return dict(width=raster.width, height=raster.height,
            center=self.inspector.pixels(self.track.control_path),
            left=self.inspector.pixels(left), right=self.inspector.pixels(right),
            session_name=self.root.name, geometry_id=self.metadata['geometry_id'],
            state_count=len(self.state_times), plan_count=len(self.plan_headers),
            origin_sim_time_s=self.origin_s, duration_s=self.duration_s,
            timeline=[time-self.origin_s for time in self.state_times],
            end_reason=self.summary['end_reason'], completed_full_laps=self.summary['completed_full_laps'],
            lookahead_m=self.metadata['config']['lookahead_m'],
            plan_period_s=self.metadata['config']['plan_period_s'],
            control_dt_s=self.metadata['config']['control_dt_s'],
            recording_complete=self.summary.get('recording_complete', False),
            index_status=self.index_status, candidate_control_enabled=False,
            validation_scope='sampled_center_points_and_polyline_only',
            footprint_verified=False)

    def seek_time(self, relative_s):
        if not math.isfinite(relative_s) or not 0 <= relative_s <= self.duration_s:
            raise ValueError('time outside recorded session')
        return min(len(self.state_times)-1,
                   max(0, bisect_right(self.state_times, self.origin_s+relative_s+1e-12)-1))

    def _body(self, pose):
        if len(pose) != 3 or any(v is None or not math.isfinite(v) for v in pose):
            return []
        physics = self.metadata['simulator_params']
        length, width = physics['vehicle_length']/2, physics['vehicle_width']/2
        corners = np.array([[length, width], [length, -width], [-length, -width], [-length, width]])
        c, s = math.cos(pose[2]), math.sin(pose[2])
        xy = corners @ np.array([[c, s], [-s, c]]) + np.asarray(pose[:2])
        return self.inspector.pixels(xy)

    @lru_cache(maxsize=16)
    def plan_data(self, plan_index):
        plan = self.record('plans', plan_index)
        source = self.record('states', plan['source_control_step'])
        if plan['source_pose'] != source['pose']:
            raise ValueError('plan source pose mismatch')
        if [p['candidate_id'] for p in plan['paths']] != [1, 2, 3, 4, 5]:
            raise ValueError('plan must contain five ordered candidate IDs')
        paths = []
        for p in plan['paths']:
            # Transform the whole recorded polyline in one NumPy call; a 40 m
            # plan can contain 8,005 points across five paths. This changes only
            # rendering, not the recorded samples or validation results.
            xy = [[q.get('x'), q.get('y')] for q in p['points']]
            finite = [all(v is not None and math.isfinite(v) for v in point) for point in xy]
            pixels = self.inspector.pixels([point if valid else [0.,0.] for point,valid in zip(xy,finite)])
            pixels = [point if valid else None for point,valid in zip(pixels,finite)]
            invalid = p.get('first_invalid_sample')
            paths.append(dict(candidate_id=p['candidate_id'], status=p['status'],
                sampled_valid=p['sampled_valid'], reason=p['reason'], pixels=pixels,
                first_invalid_pixel=pixels[invalid] if type(invalid) is int and 0 <= invalid < len(pixels) else None,
                requested_target_d=p.get('requested_target_d'), actual_target_d=p.get('actual_target_d'),
                start_position_error_m=p.get('start_position_error_m'),
                start_heading_error_rad=p.get('start_heading_error_rad')))
        return dict(plan_id=plan['plan_id'], source_control_step=plan['source_control_step'],
            source_sim_time_s=plan['source_sim_time_s'], source_pose=plan['source_pose'],
            source_pixel=self._pixel(plan['source_pose'][:2]), source_body=self._body(plan['source_pose']),
            status=plan['status'], reason=plan.get('reason'), wall_time_s=plan.get('wall_time_s'),
            sampled_valid_count=sum(p['sampled_valid'] for p in plan['paths']), paths=paths)

    def frame(self, step):
        if type(step) is not int or not 0 <= step < len(self.state_times):
            raise ValueError('step outside recorded session')
        state = self.record('states', step)
        eligible = [(p['source_control_step'], p['plan_id'], i) for i, p in enumerate(self.plan_headers)
            if max(p['source_control_step'], p['available_control_step']) <= step and
            max(p['source_sim_time_s'], p['available_sim_time_s']) <= state['sim_time_s']]
        plan = self.plan_data(max(eligible)[2]) if eligible else None
        return dict(control_step=step, sim_time_s=state['sim_time_s'],
            relative_time_s=state['sim_time_s']-self.origin_s,
            valid=state['valid'], reason=state.get('reason'), pose=state['pose'],
            pixel=self._pixel(state['pose'][:2]), body=self._body(state['pose']),
            heading_pixel=self._pixel((np.asarray(state['pose'][:2])+ .7*np.array([
                math.cos(state['pose'][2]), math.sin(state['pose'][2])])).tolist())
                if all(v is not None and math.isfinite(v) for v in state['pose']) else None,
            speed_mps=state['speed_mps'], frenet=state['frenet'], trail=self.trail[:step+1],
            plan=plan, candidate_age_s=None if plan is None else state['sim_time_s']-plan['source_sim_time_s'],
            at_end=step == len(self.state_times)-1,
            end_reason=self.summary['end_reason'] if step == len(self.state_times)-1 else None)


def make_server(session, port=8766, progress_directory=None):
    static = {'/': ('playback.html', 'text/html; charset=utf-8'),
              '/playback.js': ('playback.js', 'text/javascript; charset=utf-8')}
    map_payload = session.map_data()
    map_payload['generation_progress_enabled'] = progress_directory is not None
    picture = session.inspector.image()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            path = urlsplit(self.path)
            try:
                if path.path in static:
                    name, mime = static[path.path]
                    body = Path(__file__).with_name(name).read_bytes()
                elif path.path == '/api/image':
                    body, mime = picture, 'image/png'
                elif path.path == '/api/session':
                    body, mime = json.dumps(map_payload, allow_nan=False).encode(), 'application/json'
                elif path.path == '/api/progress' and progress_directory is not None:
                    try:
                        progress = json.loads((Path(progress_directory)/'progress.json').read_text())
                    except (OSError, ValueError):
                        progress = dict(status='starting')
                    body, mime = json.dumps(progress, allow_nan=False).encode(), 'application/json'
                elif path.path == '/api/frame':
                    params = parse_qs(path.query, keep_blank_values=True)
                    if set(params) not in ({'step'}, {'time'}) or any(len(v) != 1 for v in params.values()):
                        raise ValueError('one step or relative time required')
                    step = int(params['step'][0]) if 'step' in params else session.seek_time(float(params['time'][0]))
                    body, mime = json.dumps(session.frame(step), allow_nan=False).encode(), 'application/json'
                else:
                    self.send_error(404)
                    return
                status = 200
            except (ValueError, KeyError, TypeError, IndexError, OverflowError) as exc:
                body, mime, status = json.dumps(dict(error=str(exc))).encode(), 'application/json', 400
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass  # A newer seek can abort an older browser request.

    return ThreadingHTTPServer(('127.0.0.1', port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, type=Path)
    parser.add_argument('--geometry-dir', type=Path, default=Path('maps/frenet/esp/geometry'))
    parser.add_argument('--map-yaml', type=Path, default=Path('maps/esp.yaml'))
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--progress-session', type=Path, help='optional background-generation progress only')
    args = parser.parse_args()
    track = TrackData(args.geometry_dir, args.map_yaml)
    session = PlaybackSession(args.session, track)
    server = make_server(session, args.port, args.progress_session)
    print('Session playback: http://127.0.0.1:'+str(server.server_port), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
