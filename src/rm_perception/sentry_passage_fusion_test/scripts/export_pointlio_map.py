#!/usr/bin/env python3
"""Export a frozen SLAM map and Point-LIO PCD in the same map frame."""
import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import tempfile
import time
import uuid
from pathlib import Path

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from slam_toolbox.srv import SerializePoseGraph
from std_srvs.srv import Trigger
import yaml


def digest(path):
    h = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def publish_file(source, destination):
    """Durable complete-file publication without overwriting any existing name."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='.map-export-', dir=destination.parent)
    linked = False
    try:
        with os.fdopen(fd, 'wb') as out, open(source, 'rb') as data:
            shutil.copyfileobj(data, out)
            out.flush()
            os.fsync(out.fileno())
        os.link(name, destination)  # EEXIST is an error, including dangling symlinks.
        linked = True
        directory = os.open(destination.parent, os.O_RDONLY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except BaseException:
        if linked:
            destination.unlink()
        raise
    finally:
        os.unlink(name)


class Exporter:
    def __init__(self):
        self.node = rclpy.create_node('pointlio_map_export')
        self.latest = None
        self.subscription = self.node.create_subscription(
            OccupancyGrid, '/map', self.receive,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def receive(self, msg):
        self.latest = msg

    def call(self, kind, name, request, timeout=15):
        client = self.node.create_client(kind, name)
        try:
            if not client.wait_for_service(timeout_sec=5):
                raise RuntimeError(f'Service unavailable: {name}. Start Point-LIO mapping first.')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
            if not future.done() or future.result() is None:
                raise RuntimeError(f'Service failed or timed out: {name}')
            return future.result()
        finally:
            self.node.destroy_client(client)

    def snapshot(self, path):
        end = time.monotonic() + 8
        while self.latest is None and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.1)
        msg = self.latest
        if (msg is None or msg.header.frame_id != 'map' or msg.header.stamp.sec <= 0
                or msg.info.width <= 0 or msg.info.height <= 0
                or len(msg.data) != msg.info.width * msg.info.height):
            raise RuntimeError('A timestamped nonempty SLAM /map in map frame is required')
        stamp = [msg.header.stamp.sec, msg.header.stamp.nanosec]
        values = [('pcd_save.export_path', str(path)),
                  ('pcd_save.map_stamp_sec', stamp[0]),
                  ('pcd_save.map_stamp_nanosec', stamp[1])]
        request = SetParametersAtomically.Request()
        for name, value in values:
            parameter = Parameter(name=name)
            parameter.value = (ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
                               if isinstance(value, str) else
                               ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value))
            request.parameters.append(parameter)
        response = self.call(SetParametersAtomically, '/laserMapping/set_parameters_atomically', request)
        if not response.result.successful:
            raise RuntimeError(f'Point-LIO rejected export parameters: {response.result.reason}')
        response = self.call(Trigger, '/pointlio/save_map_snapshot', Trigger.Request(), timeout=60)
        if not response.success:
            raise RuntimeError(f'Point-LIO PCD export failed: {response.message}')
        association = json.loads(response.message)
        if (association.get('frame') != 'map' or association.get('lio') != 'pointlio'
                or association.get('source_frame') != 'odom' or association.get('stamp') != stamp
                or association.get('points', 0) <= 0 or not path.is_file()):
            raise RuntimeError('PCD export did not provide the requested frame/timestamp association')
        return msg, association

    def save_grid(self, snapshot, prefix):
        topic = '/pointlio_export/map_snapshot_' + uuid.uuid4().hex
        pub = self.node.create_publisher(OccupancyGrid, topic,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        try:
            with tempfile.TemporaryFile(mode='w+') as log:
                process = subprocess.Popen([
                    'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(prefix),
                    '--ros-args', '-r', '/map:=' + topic,
                    '-p', 'map_subscribe_transient_local:=true'], stdout=log, stderr=subprocess.STDOUT)
                try:
                    end = time.monotonic() + 30
                    while process.poll() is None and time.monotonic() < end:
                        pub.publish(snapshot)
                        rclpy.spin_once(self.node, timeout_sec=.05)
                    if process.poll() is None:
                        raise RuntimeError('Saving frozen occupancy map timed out')
                    if process.returncode:
                        log.seek(0)
                        raise RuntimeError('Map saver failed: ' + log.read()[-4000:])
                finally:
                    if process.poll() is None:
                        process.terminate()
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait()
        finally:
            self.node.destroy_publisher(pub)

    def export(self, workspace, name):
        share = workspace / 'src/rm_nav_bringup'
        installed = workspace / 'install/rm_nav_bringup/share/rm_nav_bringup'
        targets = {'.yaml': 'map', '.pgm': 'map', '.posegraph': 'map', '.data': 'map',
                   '.pcd': 'PCD', '.registration.json': 'map'}
        roots = [share]
        if not installed.is_dir():
            raise RuntimeError('Build rm_nav_bringup before saving maps')
        if installed.resolve() != share.resolve():
            roots.append(installed)
        for root in roots:
            for suffix, folder in targets.items():
                if os.path.lexists(root / folder / (name + suffix)):
                    raise FileExistsError(f'Refusing to replace existing map: {root / folder / (name + suffix)}')
        # Check required mapping services before creating any artifacts.
        check = self.node.create_client(Trigger, '/pointlio/save_map_snapshot')
        try:
            if not check.wait_for_service(timeout_sec=5):
                raise RuntimeError('Point-LIO snapshot service unavailable; restart updated mapping.sh')
        finally:
            self.node.destroy_client(check)
        created = []
        with tempfile.TemporaryDirectory(prefix='.pointlio-export-', dir=share) as temporary:
            stage = Path(temporary)
            prefix = stage / name
            response = self.call(SerializePoseGraph, '/slam_toolbox/serialize_map',
                                 SerializePoseGraph.Request(filename=str(prefix)), timeout=60)
            if response.result != SerializePoseGraph.Response.RESULT_SUCCESS:
                raise RuntimeError('SLAM pose-graph serialization failed')
            # Freeze after serialization; save PCD immediately so its exact map-time TF is cached.
            snapshot, association = self.snapshot(prefix.with_suffix('.pcd'))
            self.save_grid(snapshot, prefix)
            metadata = yaml.safe_load(prefix.with_suffix('.yaml').read_text())
            metadata['image'] = name + '.pgm'
            prefix.with_suffix('.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
            manifest = {'version': 1, 'lio': 'pointlio', 'scan_frame': 'lidar_scan',
                        'frame': 'map', 'association': association,
                        'map_geometry': {'resolution': snapshot.info.resolution,
                            'size': [snapshot.info.width, snapshot.info.height],
                            'origin': [snapshot.info.origin.position.x,
                                       snapshot.info.origin.position.y,
                                       snapshot.info.origin.position.z],
                            'origin_quaternion': [snapshot.info.origin.orientation.x,
                                snapshot.info.origin.orientation.y, snapshot.info.origin.orientation.z,
                                snapshot.info.origin.orientation.w]}, 'files': {}}
            for suffix, folder in targets.items():
                if suffix == '.registration.json':
                    continue
                path = stage / (name + suffix)
                if not path.is_file() or path.stat().st_size == 0:
                    raise RuntimeError(f'Missing exported artifact: {path.name}')
                manifest['files'][folder + '/' + path.name] = digest(path)
            (stage / (name + '.registration.json')).write_text(
                json.dumps(manifest, indent=2, allow_nan=False) + '\n')
            try:
                for root in roots:
                    for suffix, folder in targets.items():
                        destination = root / folder / (name + suffix)
                        # Colcon symlink installs may alias individual folders.
                        if any(destination.resolve() == p.resolve() for p in created):
                            continue
                        publish_file(stage / (name + suffix), destination)
                        created.append(destination)
            except BaseException:
                for path in reversed(created):
                    path.unlink()
                raise
        print(f'Saved new map {name}: YAML/PGM, pose graph, map-frame PCD and registration manifest.', flush=True)
        print(f'PCD points={association["points"]}; map stamp={association["stamp"]}; source and installed artifacts synchronized.', flush=True)
        return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workspace', type=Path)
    parser.add_argument('map_name')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', args.map_name):
        parser.error('map_name must be a new basename using letters, digits, _ or -')
    workspace = args.workspace.resolve()
    if not (workspace / 'src/rm_nav_bringup').is_dir():
        parser.error('Not a navigation workspace')
    rclpy.init()
    exporter = Exporter()
    try:
        exporter.export(workspace, args.map_name)
    except Exception as error:
        parser.exit(1, f'Map export failed: {error}\n')
    finally:
        exporter.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
