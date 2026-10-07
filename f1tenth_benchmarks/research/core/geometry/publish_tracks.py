"""Package validated track files at uniform paths; keep research runs outside maps.

This creates local files for Git review. It does not commit or push them.
"""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil

from .domain import json_bytes
from .frenet import load_geometry
from .map_raster import sha256
from .section_ranges import load_ranges


def publish_catalog(catalog, output):
    catalog,output=Path(catalog).resolve(),Path(output).resolve()
    if output.exists():raise FileExistsError('choose a new publication directory')
    sources=[];ids=set()
    # Check every source before creating the destination.
    for entry in json.loads(catalog.read_text()):
        geometry=(catalog.parent/entry['geometry']).resolve()
        ranges=(catalog.parent/entry['ranges']).resolve()
        map_yaml=(catalog.parent/entry['map_yaml']).resolve()
        map_id=map_yaml.stem
        if not re.fullmatch(r'[A-Za-z0-9_-]+',map_id) or map_id in ids:
            raise ValueError('invalid or duplicate map id')
        ids.add(map_id)
        g=load_geometry(geometry,map_yaml,inspect=True)
        payload=load_ranges(ranges,g,geometry)
        if (not payload['summary']['range_interface_passed'] or
                not payload['summary']['numerical_checks_passed'] or
                not payload['geometry_audit']['passed']):
            raise ValueError('track range validation not passed')
        for name in ('centerline.csv','boundaries.csv'):
            if (geometry/name).read_bytes()!=(ranges/name).read_bytes():
                raise ValueError('geometry/range CSV disagreement')
        sources.append((entry,map_id,geometry,ranges,map_yaml,payload))
    output.mkdir(parents=True)
    published=[]
    for entry,map_id,geometry,ranges,map_yaml,payload in sources:
        target=output/map_id;gd=target/'geometry';rd=target/'ranges'
        gd.mkdir(parents=True);rd.mkdir()
        gm=json.loads((geometry/'manifest.json').read_text())
        for name in ('manifest.json',*gm['file_hashes']):
            shutil.copyfile(geometry/name,gd/name)
        if (geometry/'validation.json').exists():
            shutil.copyfile(geometry/'validation.json',gd/'validation.json')
        rm=json.loads((ranges/'manifest.json').read_text())
        original_bundle=rm.pop('bundle_id')
        original_payload=ranges/('section_ranges.json.gz' if 'section_ranges.json.gz' in rm['file_hashes'] else 'section_ranges.json')
        raw=original_payload.read_bytes()
        if original_payload.suffix=='.gz':raw=gzip.decompress(raw)
        (rd/'section_ranges.json.gz').write_bytes(gzip.compress(raw,mtime=0))
        shutil.copyfile(ranges/'report.json',rd/'report.json')
        rm['csv_location']='source_geometry'
        rm['file_hashes']={name:sha256((gd if name.endswith('.csv') else rd)/name) for name in
            ('centerline.csv','boundaries.csv','report.json','section_ranges.json.gz')}
        rm['publication']=dict(source_bundle_id=original_bundle,
            uncompressed_payload_sha256=hashlib.sha256(raw).hexdigest(),
            transformation='lossless gzip; numerical payload and CSV unchanged')
        rm['bundle_id']=hashlib.sha256(json_bytes(rm)).hexdigest()
        (rd/'manifest.json').write_bytes(json_bytes(rm))
        published.append(dict(label=entry['label'],geometry=f'{map_id}/geometry',
            ranges=f'{map_id}/ranges',map_yaml=os.path.relpath(map_yaml,output)))
        # Reload the portable copy, including geometry, source map and gzip hashes.
        g=load_geometry(gd,map_yaml,inspect=True)
        assert load_ranges(rd,g,gd)==payload
    (output/'inspector_catalog.json').write_text(json.dumps(published,ensure_ascii=False,indent=2)+'\n')
    files={str(p.relative_to(output)):sha256(p) for p in sorted(output.rglob('*')) if p.is_file()}
    (output/'publication.json').write_bytes(json_bytes(dict(schema_version=1,
        file_hashes=files,track_count=len(published),payloads_preserved=True,
        source_code_hash_semantics='historical numerical export, not publication runtime',
        continuous_verified=False,domain_verified=False,planning_allowed=False)))
    return published


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--catalog',required=True,type=Path)
    parser.add_argument('--output-root',required=True,type=Path)
    args=parser.parse_args()
    print(json.dumps(publish_catalog(args.catalog,args.output_root),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
