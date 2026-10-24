#!/usr/bin/env python3
"""Export mapping-session passage candidates for one saved navigation map.

Run only after the map files have been saved and mapping has stopped. The
result is a new record file, never an entry permit. Navigation still checks
the exact saved-map identity, map geometry, TF association and every recorded
static-map cell before using a historical candidate.
"""
import argparse
from dataclasses import asdict, replace
import hashlib
import importlib.util
import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from global_passage_memory import MapContext
from passage_record_io import read_record
from continuous_passage_memory import ContinuousPassageMemory


def export_record(source, output, target_epoch):
    source, output = Path(source).expanduser().resolve(), Path(output).expanduser().absolute()
    if source == output or output.exists():
        raise ValueError('Output must be a new file, separate from the mapping record')
    if not target_epoch.startswith('saved-map-') or len(target_epoch) != 74:
        raise ValueError('Target must be a verified saved-map identity')
    body = read_record(source)
    old = MapContext(**body['context'])
    if not old.valid() or not old.epoch.startswith('mapping-'):
        raise ValueError('Source is not a valid mapping-session record')
    if body['version'] == 4:
        ContinuousPassageMemory.from_body(body)
        structures = body['geometry_groups'][:1]  # One map-associated history collection.
    else:
        structures = body.get('structures', []) if body['version'] == 3 else []
    if (not isinstance(body['segments'], list) or not isinstance(structures, list) or
            not 0 < len(body['segments'])+len(structures) <= 128):
        raise ValueError('No historical passage candidates to export')
    # The original is immutable. The navigation runtime will independently
    # reject geometry, TF or segment-cell mismatches on loading this copy.
    new_body = dict(body)
    new_body['context'] = asdict(replace(old, epoch=target_epoch))
    new_body['handover'] = {'source_epoch': old.epoch,
                            'kind': 'historical_candidates_only'}
    new_packed = json.dumps(new_body, sort_keys=True,
                            separators=(',', ':'), allow_nan=False).encode()
    if len(new_packed) > 4_000_000:
        raise ValueError('Exported record is too large')
    result = {'body': new_body, 'sha256': hashlib.sha256(new_packed).hexdigest()}
    fd = os.open(output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(result, stream, sort_keys=True, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        output.unlink(missing_ok=True)
        raise
    return len(body['segments'])+len(structures)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', required=True, help='Saved map filename prefix')
    parser.add_argument('--localization', default='slam_toolbox',
                        choices=['slam_toolbox', 'amcl', 'icp'])
    parser.add_argument('--source', default='~/.ros/passage_memory/runtime.json')
    parser.add_argument('--output', required=True,
                        help='New record path for navigation; never overwrites source')
    args = parser.parse_args()
    bringup = get_package_share_directory('rm_nav_bringup')
    fusion = get_package_share_directory('sentry_passage_fusion_test')
    launch_file = os.path.join(fusion, 'launch', 'passage_real.launch.py')
    spec = importlib.util.spec_from_file_location('passage_real_identity', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    try:
        target = module.saved_map_identity(os.path.join(bringup, 'map'), args.world,
                                           args.localization, os.path.join(bringup, 'PCD'))
        count = export_record(args.source, args.output, target)
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
        parser.exit(2, f'Passage record handover refused: {exc}\n')
    print(f'Exported {count} historical candidates for {args.world} to {args.output}')
    print('No entry permit was exported; navigation must verify map, TF and fresh entry evidence.')


if __name__ == '__main__':
    main()
