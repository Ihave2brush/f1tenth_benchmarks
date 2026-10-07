"""Loopback-only map inspector; browser rendering shares the Python Frenet core."""
import argparse
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import io
import json
import math
from pathlib import Path
from urllib.parse import parse_qs,urlsplit

import numpy as np
from PIL import Image

from .domain import json_bytes
from .frenet import load_geometry
from .validate_stage_d import load_report
from .complete_stage_d import load_diagnostics
from .section_ranges import query_section,load_ranges


class MapInspector:
    def __init__(self,geometry,report=None,label=None,diagnostics=None,ranges=None):
        self.geometry=geometry
        self.label=label or (geometry.raster.yaml_path.stem if geometry.raster.yaml_path else geometry.geometry_id)
        if report is not None and report.get('geometry_id')!=geometry.geometry_id:
            raise ValueError('report geometry mismatch')
        self.report=report
        if diagnostics is not None and (diagnostics.get('geometry_id')!=geometry.geometry_id
                or diagnostics.get('planning_allowed') is not False):
            raise ValueError('diagnostics geometry or approval mismatch')
        self.diagnostics=diagnostics
        if ranges is not None and ranges.get('geometry_id')!=geometry.geometry_id:raise ValueError('ranges geometry mismatch')
        self.ranges=ranges

    def pixels(self,xy):
        if np.asarray(xy).size==0:return []
        q=self.geometry.raster.map_to_local(xy)/self.geometry.raster.resolution
        # Canvas coordinates refer to image edges, not pixel-centre indices.
        return np.stack((q[...,0],self.geometry.raster.height-q[...,1]),axis=-1).tolist()

    def map_data(self):
        g=self.geometry;r=g.raster;positions=np.linspace(0,g.curve.L,int(np.ceil(g.curve.L/.05))+(0 if g.curve.closed else 1),endpoint=not g.curve.closed)
        xy,t,_,_=g.curve.sample(positions)
        boundaries=[g.boundaries(s)for s in positions]
        left=[b['left_xy']for b in boundaries if b['valid']];right=[b['right_xy']for b in boundaries if b['valid']]
        ref=g.reference(0);arrow=np.asarray(ref['xy'])+np.asarray(ref['tangent'])*2
        rejected=[] if self.report is None else self.report['rejected_points']
        issues=[] if self.diagnostics is None else [dict(p,
            center_pixels=self.pixels(p['center_xy']),
            samples=[dict(sample,pixel=self.pixels(sample['xy'])) for sample in p['samples']])
            for p in self.diagnostics['issues']]
        if self.ranges is not None and self.report is None:
            rejected=[dict(s_m=p['s_m'],**q) for p in self.ranges['sections'] for q in p['samples'] if not q['valid']]
            for section in self.ranges['sections']:
                samples=[dict(q,s_m=section['s_m'],pixel=self.pixels(q['xy'])) for q in section['samples'] if not q['valid']]
                if not samples:continue
                reasons=sorted(set(q['reason'] for q in samples))
                for reason in reasons:
                    group=[q for q in samples if q['reason']==reason]
                    issues.append(dict(id='S-'+str(len(issues)+1).zfill(3),category=reason,
                        label='截面限制：'+reason,s_start_m=section['s_m'],s_end_m=section['s_m'],
                        samples=group,sample_count=len(group),example_index=int(np.argmin([q['J'] for q in group])),crosses_seam=False,
                        min_J=min(q['J'] for q in group),max_J=max(q['J'] for q in group),
                        nonpositive_J_samples=sum(q['J']<=0 for q in group),
                        center_pixels=[self.pixels(section['reference']['xy'])],
                        suggested_action='查詢此 s 的道路與 Frenet 採樣範圍；候選仍須逐點確認'))
        audit=None if self.diagnostics is None else self.diagnostics['geometry_audit']
        if audit is None and self.ranges is not None:audit=self.ranges.get('geometry_audit')
        return dict(label=self.label,geometry_id=g.geometry_id,width=r.width,height=r.height,L_m=g.curve.L,closed=g.curve.closed,
            center=self.pixels(xy),left=self.pixels(left),right=self.pixels(right),
            start=self.pixels([ref['xy'],arrow]),rejected=[dict(pixel=self.pixels(p['xy']),reason=p['reason'])for p in rejected],
            numeric_checks_passed=None if self.report is None else self.report['numeric_checks_passed'],
            issues=issues,geometry_audit=audit,
            range_summary=None if self.ranges is None else self.ranges['summary'],
            diagnostic_ready=False if self.diagnostics is None else self.diagnostics['diagnostic_ready'],
            planning_allowed=False,domain_verified=False)

    def image(self):
        buffer=io.BytesIO();Image.fromarray(self.geometry.raster.gray).save(buffer,format='PNG');return buffer.getvalue()

    def query_pixel(self,c,v):
        if not all(math.isfinite(z)for z in (c,v)):raise ValueError('nonfinite pixel')
        xy=self.geometry.raster.pixel_to_map(c,v)
        return self.query_xy(xy)

    def query_sd(self,s,d):
        result=self.geometry.to_cartesian(s,d)
        if result['xy'] is None:raise ValueError('nonfinite coordinates')
        response=self.query_xy(result['xy'])
        response['source_frenet']=dict(s_m=s,d_m=d,valid=result['valid'],reason=result['reason'])
        return response

    def query_section(self,s):
        result=query_section(self.geometry,s)
        ref=result['reference'];normal=np.array([-ref['tangent'][1],ref['tangent'][0]])
        def endpoints(run):
            return self.pixels([np.asarray(ref['xy'])+normal*run[key] for key in ('d_min_m','d_max_m')])
        result['road_pixels']=[] if result['road_range'] is None else endpoints(result['road_range'])
        result['range_pixels']=[endpoints(run) for run in result['frenet_ranges']]
        result['reference_pixel']=self.pixels(ref['xy'])
        return result

    def query_xy(self,xy):
        g=self.geometry;pixel=g.raster.map_to_pixel(xy);result=g.to_frenet(xy)
        candidates=g.projection_candidates(xy)['candidates'];diagnostics=[]
        for c in candidates:
            b=c['boundary'];normal=[b['right_xy'],b['left_xy']] if b['valid'] else []
            diagnostics.append(dict(s_m=c['s'],d_m=c['d'],J=c['jacobian'],valid=c['valid'],reason=c['reason'],
                projection_pixel=self.pixels(c['projection_xy']),normal_pixels=self.pixels(normal) if normal else [],
                boundary=b))
        reconstructed=None;error=None
        if result['valid']:
            inverse=g.to_cartesian(result['s_wrapped'],result['d'])
            reconstructed=inverse['xy'];error=float(np.linalg.norm(np.asarray(reconstructed)-xy))
        return dict(geometry_id=g.geometry_id,frame_id='map',xy=np.asarray(xy).tolist(),pixel=list(pixel.pixel),
            cell=pixel.cell,occupancy=g.raster.classify(xy),query_pixel=self.pixels(xy),frenet=result,
            candidates=diagnostics,reconstructed_xy=reconstructed,
            reconstructed_pixel=None if reconstructed is None else self.pixels(reconstructed),reconstruction_error_m=error,
            planning_allowed=False,domain_verified=False)


def make_server(inspectors,port=8765):
    if not inspectors:raise ValueError('no maps')
    payloads=[m.map_data()for m in inspectors];images=[m.image()for m in inspectors]
    html=Path(__file__).with_name('viewer.html').read_bytes()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def do_GET(self):
            parsed=urlsplit(self.path);params=parse_qs(parsed.query)
            try:
                if parsed.path=='/':body,mime=html,'text/html; charset=utf-8'
                elif parsed.path=='/api/catalog':
                    body,mime=json_bytes([dict(index=i,label=m.label,geometry_id=m.geometry.geometry_id)for i,m in enumerate(inspectors)]),'application/json'
                elif parsed.path in ('/api/map','/api/image','/api/query','/api/inverse','/api/section'):
                    index=int(params.get('map',['0'])[0])
                    if not 0<=index<len(inspectors):raise ValueError('unknown map')
                    if parsed.path=='/api/map':body,mime=json_bytes(payloads[index]),'application/json'
                    elif parsed.path=='/api/image':body,mime=images[index],'image/png'
                    elif parsed.path=='/api/query':
                        result=inspectors[index].query_pixel(float(params['c'][0]),float(params['v'][0]));body,mime=json_bytes(result),'application/json'
                    elif parsed.path=='/api/section':
                        result=inspectors[index].query_section(float(params['s'][0]));body,mime=json_bytes(result),'application/json'
                    else:
                        result=inspectors[index].query_sd(float(params['s'][0]),float(params['d'][0]));body,mime=json_bytes(result),'application/json'
                else:self.send_error(404);return
                self.send_response(200)
            except (ValueError,KeyError,TypeError):
                body,mime=json_bytes(dict(error='invalid query or map')),'application/json';self.send_response(400)
            self.send_header('Content-Type',mime);self.send_header('Content-Length',str(len(body)))
            self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(body)
    return ThreadingHTTPServer(('127.0.0.1',port),Handler)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry',type=Path);parser.add_argument('--map-yaml',type=Path)
    parser.add_argument('--report',type=Path);parser.add_argument('--catalog',type=Path)
    parser.add_argument('--diagnostics',type=Path)
    parser.add_argument('--ranges',type=Path)
    parser.add_argument('--port',type=int,default=8765);args=parser.parse_args()
    if args.catalog:
        if args.geometry or args.map_yaml or args.report or args.diagnostics or args.ranges:parser.error('catalog cannot be combined with single-map arguments')
        entries=json.loads(args.catalog.read_text());base=args.catalog.parent
    else:
        if args.geometry is None or args.map_yaml is None:parser.error('geometry and map-yaml required')
        entries=[dict(geometry=str(args.geometry),map_yaml=str(args.map_yaml),report=None if args.report is None else str(args.report),
            diagnostics=None if args.diagnostics is None else str(args.diagnostics),ranges=None if args.ranges is None else str(args.ranges))];base=Path.cwd()
    inspectors=[]
    for entry in entries:
        geometry=load_geometry(base/entry['geometry'],base/entry['map_yaml'],inspect=True)
        report=load_report(base/entry['report'],geometry) if entry.get('report') else None
        diagnostics=load_diagnostics(base/entry['diagnostics'],geometry,
            base/entry['report'] if entry.get('report') else None,base/entry['geometry']) if entry.get('diagnostics') else None
        ranges=load_ranges(base/entry['ranges'],geometry,base/entry['geometry']) if entry.get('ranges') else None
        inspectors.append(MapInspector(geometry,report,entry.get('label'),diagnostics,ranges))
    server=make_server(inspectors,args.port)
    print('Frenet inspector: http://127.0.0.1:'+str(server.server_port),flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()


if __name__=='__main__':main()
