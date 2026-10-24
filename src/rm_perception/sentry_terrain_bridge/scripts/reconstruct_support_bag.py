#!/usr/bin/env python3
"""Read-only passage_nuc_02 reconstruction, not a general calibrated bag player.

Only launches terrain_node and publishes clock, recorded IMU-body cloud/odom,
and a reconstructed probe. No navigation, serial or velocity publication.
Requires ROS Humble, numpy, scipy, yaml. Outputs are analytical artifacts.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def plot_result(events, arrays, status, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Polygon
    odom = [m for _, n, m in events if n == '/Odometry']
    stamps = np.array([m.header.stamp.sec+m.header.stamp.nanosec/1e9 for m in odom])
    points = []
    for m in [m for _, n, m in events if n == '/cloud_registered_body'][::5]:
        stamp = m.header.stamp.sec+m.header.stamp.nanosec/1e9
        pose = odom[np.argmin(abs(stamps-stamp))].pose.pose
        q = pose.orientation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        translation = np.array([pose.position.x, pose.position.y, pose.position.z])
        dt = np.dtype({'names': ['x','y','z'], 'formats': ['<f4']*3,
                       'offsets': [0,4,8], 'itemsize': m.point_step})
        a = np.frombuffer(m.data, dtype=dt)
        # Registered body points have an IMU mathematical frame, despite their label.
        points.append(np.column_stack([a[k] for k in ['x','y','z']])@rotation.T+translation)
    cloud = np.concatenate(points)
    x,y,z,yaw = arrays['probe']; co,si=np.cos(yaw),np.sin(yaw)
    cloud=cloud[(abs(cloud[:,0]-x)<.85)&(abs(cloud[:,1]-y)<.85)]
    footprint=np.array([[-.325,-.225],[.325,-.225],[.325,.225],[-.325,.225]])@np.array([[co,si],[-si,co]])+[x,y]
    fields=dict(token.split('=',1) for token in status.split() if '=' in token)
    height=float(fields.get('robot_height',.7))
    bottom=(np.floor(z/.05)+1)*.05; depth=(np.ceil(height/.05)+1)*.05
    fig,axes=plt.subplots(1,3,figsize=(16,5))
    axes[0].scatter(cloud[:,0],cloud[:,1],s=.5,c='0.75')
    axes[0].add_patch(Polygon(footprint,fill=False,edgecolor='blue'))
    axes[0].set(xlabel='odom x [m]',ylabel='odom y [m]',title='Top view')
    for ax,dim,title in [(axes[1],0,'Front view'),(axes[2],1,'Side view: approach to passage')]:
        ax.scatter(cloud[:,dim],cloud[:,2],s=.5,c='0.75')
        ax.add_patch(Rectangle((footprint[:,dim].min(),bottom),np.ptp(footprint[:,dim]),depth,
                               fill=False,edgecolor='blue',label='Original voxel query'))
        ax.axhline(z,color='magenta',ls='--',label='Configured ground')
        ax.set(xlabel=('odom x [m]' if dim==0 else 'odom y [m]'),ylabel='odom z [m]',title=title)
        ax.grid(alpha=.2)
    for ns,color,label in [('body_blocked_red','red','Blocked voxels'),
                           ('measured_body_blocked_shadow','black','Remaining measured-plane blockers'),
                           ('support_missing_magenta','magenta','Missing original support'),
                           ('support_candidates_yellow_diagnostic','darkorange','Measured support candidates')]:
        p=arrays.get(ns,np.empty((0,3)))
        if len(p):
            axes[0].scatter(p[:,0],p[:,1],s=12,c=color,label=label)
            for ax,dim in [(axes[1],0),(axes[2],1)]:ax.scatter(p[:,dim],p[:,2],s=12,c=color,label=label)
    unobserved=arrays.get('support_unobserved_xy_diagnostic',np.empty((0,3)))
    if len(unobserved):
        axes[0].scatter(unobserved[:,0],unobserved[:,1],s=14,c='purple',marker='x',
                        label='Support not observed (XY only)')
    profile=arrays.get('support_profile_diagnostic',np.empty((0,3)))
    for i in range(0,len(profile)-1,2):
        segment=profile[i:i+2]
        label='Observed support profile' if i==0 else None
        axes[0].plot(segment[:,0],segment[:,1],c='gold',lw=2,label=label)
        axes[1].plot(segment[:,0],segment[:,2],c='gold',lw=2,label=label)
        axes[2].plot(segment[:,1],segment[:,2],c='gold',lw=2,label=label)
    reason=status.split(' support_diagnostic=')[-1].split()[0]
    if fields.get('measured_evaluated')=='1':
        along=np.linspace(-.325,.325,30)
        center_z=float(fields['measured_ground_z']);slope=float(fields['measured_slope'])
        bend=float(fields.get('measured_break_along',0))
        delta=float(fields.get('measured_slope_after',slope))-slope
        modeled_z=center_z+slope*along+delta*(np.maximum(0,along-bend)-max(0,-bend))
        for ax,dim in [(axes[1],0),(axes[2],1)]:
            horizontal=(x+co*along) if dim==0 else (y+si*along)
            ax.plot(horizontal,modeled_z,c='green',lw=2,ls='--',label='Fitted support (not permission)')
    reason+=' | '+fields.get('measured_model','')+' / '+fields.get('measured_reason','')
    axes[1].legend(fontsize=7,loc='upper left')
    fig.suptitle('Recorded cloud + reconstructed ROGMap | '+reason+' | DIAGNOSTIC ONLY')
    fig.tight_layout();fig.savefig(out/'support_overlay.png',dpi=150);plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--db', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--terrain-executable', required=True)
    parser.add_argument('--output', required=True, help='New output directory')
    parser.add_argument('--domain', type=int, default=173)
    parser.add_argument('--robot-height', type=float, default=.7,
                        help='Read-only counterfactual; never publishes evidence')
    args = parser.parse_args()
    if not 100 <= args.domain <= 230:
        parser.error('Use a dedicated domain 100..230, not the live robot domain')
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = str(out/'roslogs')
    import rclpy
    from rclpy.serialization import deserialize_message
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from visualization_msgs.msg import MarkerArray

    db = sqlite3.connect(Path(args.db).resolve().as_uri()+'?mode=ro', uri=True)
    topics = {i: (n, get_message(t)) for i, n, t in db.execute('select id,name,type from topics')}
    events, transforms, probe = [], {}, None
    for tid, ts, data in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
        name, cls = topics[tid]
        if name not in ('/tf', '/Odometry', '/cloud_registered_body', '/passage/probe_status'):
            continue
        msg = deserialize_message(data, cls)
        if name == '/tf':
            for tr in msg.transforms:
                transforms[tr.header.frame_id, tr.child_frame_id] = tr.transform
        elif name == '/passage/probe_status':
            state = json.loads(msg.data)
            if probe is None and state['state'] == 'PROBE_SENT_NOT_PERMISSION':
                x, y, yaw = state['candidate']
                a, b = transforms['odom', 'base_link'], transforms['map', 'odom']
                q = b.rotation
                rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                translation = np.array([b.translation.x, b.translation.y, b.translation.z])
                base = np.array([a.translation.x, a.translation.y, a.translation.z])
                map_base = rotation@base+translation
                # Reproduce coordinator's planar XY and z offset, not a new calibration.
                inverse_t = -rotation.T@translation
                tf_yaw = -Rotation.from_matrix(rotation).as_euler('xyz')[2]
                point = [inverse_t[0]+np.cos(tf_yaw)*x-np.sin(tf_yaw)*y,
                         inverse_t[1]+np.sin(tf_yaw)*x+np.cos(tf_yaw)*y,
                         inverse_t[2]+map_base[2]-.23]
                probe = (ts, point, yaw+tf_yaw)
        else:
            events.append((ts, name, msg))
    db.close()
    if not probe or not events:
        raise RuntimeError('No recorded probe or required cloud/odom; do not invent a target')
    cfg = yaml.safe_load(Path(args.config).read_text())['sentry_terrain']['ros__parameters']
    # Explicit recorded-session assumptions; not production defaults or new vehicle calibration.
    cfg.update({'use_sim_time': True, 'sensor_offset': [-.011, -.02329, .04412],
                'clearance.enabled': True, 'clearance.support_diagnostic': True,
                'clearance.support_mode': 'shadow',
                'clearance.robot_height': args.robot_height, 'clearance.robot_length': .45,
                'clearance.robot_width': .45, 'clearance.travel_length': .2,
                'clearance.max_observation_age': 60., 'clearance.entry_recent_age': 2.,
                'clearance.confirmations': 3, 'clearance.publish_verified_segments': False})
    cfg['rog_map']['visualization']['enable'] = False
    params = out/'params.yaml'
    params.write_text(yaml.safe_dump({'/**': {'ros__parameters': cfg}}))
    (out/'assumptions.json').write_text(json.dumps({'probe': probe,
        'ground_offset': -.23, 'robot_height': args.robot_height,
        'support_mode': 'shadow', 'source': str(Path(args.db).resolve()),
        'note': 'Current local ROGMap config, not recorded parameter dump. Approximate reconstruction.'}, indent=2))
    rclpy.init()
    node = rclpy.create_node('offline_support_reconstruction')
    clock = node.create_publisher(Clock, '/clock', 10)
    pubs = {'/Odometry': node.create_publisher(Odometry, '/Odometry', qos_profile_sensor_data),
            '/cloud_registered_body': node.create_publisher(PointCloud2, '/cloud_registered_body', qos_profile_sensor_data)}
    probe_pub = node.create_publisher(PoseStamped, '/terrain/clearance/probe', 10)
    markers, statuses = [], []
    node.create_subscription(MarkerArray, '/terrain/clearance/markers', lambda m: markers.append(m), 10)
    node.create_subscription(String, '/terrain/clearance/status', lambda m: statuses.append(m.data), 10)
    def spin(duration):
        until = time.monotonic()+duration
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.005)
    def clock_at(ts):
        msg = Clock(); msg.clock.sec, msg.clock.nanosec = divmod(ts, 10**9); clock.publish(msg)
    proc = None
    with open(out/'terrain.log', 'w') as log:
        try:
            proc = subprocess.Popen([args.terrain_executable, '--ros-args', '--params-file', str(params)],
                                    stdout=log, stderr=subprocess.STDOUT)
            clock_at(events[0][0]); spin(2.)
            sent = False
            for k, (ts, name, msg) in enumerate(events):
                if proc.poll() is not None:
                    raise RuntimeError('terrain_node exited; see terrain.log')
                clock_at(ts); pubs[name].publish(msg)
                if not sent and ts >= probe[0]:
                    p = PoseStamped(); p.header.frame_id = 'odom'
                    p.header.stamp.sec, p.header.stamp.nanosec = divmod(ts, 10**9)
                    p.pose.position.x, p.pose.position.y, p.pose.position.z = map(float, probe[1])
                    p.pose.orientation.z = float(np.sin(probe[2]/2)); p.pose.orientation.w = float(np.cos(probe[2]/2))
                    probe_pub.publish(p); sent = True
                if k+1 < len(events):
                    spin(min(.15, max(.002, (events[k+1][0]-ts)/1e9)))
            spin(.5)
            (out/'status.txt').write_text('\n'.join(statuses))
            if not markers or not statuses:
                raise RuntimeError('No diagnostic output received')
            arrays = {'probe': np.array([*probe[1], probe[2]])}
            for marker in markers[-1].markers:
                if marker.points:
                    arrays[marker.ns] = np.array([[p.x, p.y, p.z] for p in marker.points])
            np.savez(out/'cells.npz', **arrays)
            plot_result(events, arrays, statuses[-1], out)
            print(statuses[-1])
        finally:
            if proc is not None and proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try: proc.wait(timeout=10)
                except subprocess.TimeoutExpired: proc.terminate(); proc.wait(timeout=5)
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
