#!/usr/bin/env python3
"""Read-only NUC02 frame audit. No ROS nodes, playback or velocity output.

Extrinsics below are session assumptions, not a measured vehicle calibration.
This audits recorded TF and the corrected composition; it does not re-run LIO.
"""
import argparse
import json
import sqlite3

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation


def matrix(p):
    t = p.position if hasattr(p, 'position') else p.translation
    q = p.orientation if hasattr(p, 'orientation') else p.rotation
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    out[:3, 3] = [t.x, t.y, t.z]
    return out


def stamp(m):
    return m.header.stamp.sec * 10**9 + m.header.stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('db')
    args = parser.parse_args()
    from pathlib import Path
    db = sqlite3.connect(Path(args.db).resolve().as_uri() + '?mode=ro', uri=True)
    topics = {i: (n, get_message(t)) for i, n, t in
              db.execute('select id,name,type from topics')}
    odom, base_tf, base_lidar = {}, {}, None
    for tid, data in db.execute('select topic_id,data from messages order by timestamp'):
        name, cls = topics[tid]
        if name not in ('/Odometry', '/tf', '/tf_static'):
            continue
        msg = deserialize_message(data, cls)
        if name == '/Odometry':
            odom[stamp(msg)] = matrix(msg.pose.pose)
        else:
            for tr in msg.transforms:
                if (tr.header.frame_id, tr.child_frame_id) == ('odom', 'base_link'):
                    base_tf[stamp(tr)] = matrix(tr.transform)
                if (tr.header.frame_id, tr.child_frame_id) == ('base_link', 'livox_frame'):
                    base_lidar = matrix(tr.transform)
    db.close()
    if base_lidar is None:
        raise RuntimeError('No recorded base_link -> livox_frame; cannot invent extrinsics')
    common = sorted(odom.keys() & base_tf.keys())
    if not common:
        raise RuntimeError('No exactly timestamp-matched odometry and base TF')
    # FAST-LIO mapping.extrinsic_T: lidar origin in IMU coordinates.
    imu_lidar = np.eye(4)
    imu_lidar[:3, 3] = [-.011, -.02329, .04412]
    lidar_base = np.linalg.inv(base_lidar)
    old_errors, corrected_shifts, closure_errors = [], [], []
    for key in common:
        old = odom[key] @ lidar_base
        corrected = odom[key] @ imu_lidar @ lidar_base
        old_errors.append(np.max(np.abs(old - base_tf[key])))
        corrected_shifts.append(corrected[:3, 3] - base_tf[key][:3, 3])
        # Recompose URDF lidar pose: must agree with lidar point transformation.
        closure_errors.append(np.max(np.abs(corrected @ base_lidar - odom[key] @ imu_lidar)))
    delta = np.asarray(corrected_shifts)
    print(json.dumps({
        'matched_samples': len(common),
        'recorded_matches_old_formula_max_abs': max(old_errors),
        'corrected_lidar_pose_closure_max_abs': max(closure_errors),
        'base_shift_xyz_min_m': delta.min(axis=0).tolist(),
        'base_shift_xyz_max_m': delta.max(axis=0).tolist(),
        'assumption': 'NUC02 extrinsic_R identity, extrinsic_T [-.011,-.02329,.04412]; not a new calibration',
        'scope': 'Does not correct the flat-ground assumption at a ramp or authorize passage'
    }, indent=2))


if __name__ == '__main__':
    main()
