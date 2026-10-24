#!/usr/bin/env python3
"""Read-only NUC02 blocker provenance; no ROS nodes or motion commands.

Uses recorded IMU poses and the session's assumed lidar->IMU extrinsics.
Raw scans are NOT deskewed here; endpoint agreement is evidence, not calibration.
"""
import argparse
import json
from pathlib import Path
import sqlite3
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def seconds(m):
    return m.header.stamp.sec+m.header.stamp.nanosec*1e-9


def xyz(m):
    fields={f.name:f for f in m.fields}
    formats={7:'f4',8:'f8'}
    dtype=np.dtype({'names':['x','y','z'],
        'formats':[('>' if m.is_bigendian else '<')+formats[fields[k].datatype] for k in ('x','y','z')],
        'offsets':[fields[k].offset for k in ('x','y','z')], 'itemsize':m.point_step})
    a=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step))
    return np.column_stack([a[k].ravel() for k in ('x','y','z')])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--db',required=True);p.add_argument('--snapshot',required=True)
    p.add_argument('--output',required=True)
    args=p.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    snap=np.load(args.snapshot);blocked=snap['measured_body_blocked_shadow'];r=.05
    ids=np.floor(blocked/r).astype(int);lookup={tuple(k):i for i,k in enumerate(ids)}
    db=sqlite3.connect(Path(args.db).resolve().as_uri()+'?mode=ro',uri=True)
    topics={i:(n,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
    odom=[]
    for tid,data in db.execute('select topic_id,data from messages order by timestamp'):
        name,cls=topics[tid]
        if name=='/Odometry':
            m=deserialize_message(data,cls);pose=m.pose.pose;q=pose.orientation
            odom.append((seconds(m),Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),
                         np.array([pose.position.x,pose.position.y,pose.position.z])))
    stamps=np.array([a[0] for a in odom]);reports={};scene=[]
    for topic in ['/cloud_registered_body','/livox/lidar/pointcloud']:
        counts=np.zeros(len(ids),dtype=int);frames=counts.copy()
        first=np.full(len(ids),np.inf);last=np.full(len(ids),-np.inf)
        total=0;skipped=0;dtmax=0
        tid,cls=next((i,t) for i,(n,t) in topics.items() if n==topic)
        for data, in db.execute('select data from messages where topic_id=? order by timestamp',(tid,)):
            m=deserialize_message(data,cls);s=seconds(m);j=int(np.argmin(abs(stamps-s)))
            dt=abs(stamps[j]-s);total+=1
            if dt>.1:skipped+=1;continue
            dtmax=max(dtmax,dt);pts=xyz(m)
            if topic=='/livox/lidar/pointcloud':pts=pts+[-.011,-.02329,.04412]
            pts=pts@odom[j][1].T+odom[j][2]
            pts=pts[np.isfinite(pts).all(axis=1)]
            if topic=='/cloud_registered_body' and total%10==0:
                mask=(abs(pts[:,0]-.05)<.7)&(abs(pts[:,1]-.85)<.65)&(pts[:,2]>-.5)&(pts[:,2]<.6)
                scene.append(pts[mask])
            mask=((pts>=blocked.min(axis=0)-r/2)&(pts<blocked.max(axis=0)+r/2)).all(axis=1)
            keys,n=np.unique(np.floor(pts[mask]/r).astype(int),axis=0,return_counts=True)
            for k,c in zip(keys,n):
                i=lookup.get(tuple(k))
                if i is not None:
                    counts[i]+=c;frames[i]+=1;first[i]=min(first[i],s);last[i]=max(last[i],s)
        reports[topic]={'frames_total':total,'frames_skipped_time_mismatch':skipped,
            'max_pose_dt_seconds':dtmax,'supported_voxels':int(sum(counts>0)),
            'min_hit_frames':int(frames.min()),'max_hit_frames':int(frames.max()),
            'points_per_voxel':counts.tolist(),'frames_per_voxel':frames.tolist(),
            'last_hit_age_seconds':[None if not np.isfinite(t) else float(stamps[-1]-t) for t in last]}
    db.close()
    report={'voxel_centers_odom_m':blocked.tolist(),'resolution':r,'sources':reports,
        'assumptions':'Recorded IMU poses; raw lidar extrinsic_R=identity, extrinsic_T=[-.011,-.02329,.04412]; raw not deskewed.',
        'scope':'Endpoint support only; does not prove obstacle identity, calibration, or passage permission.'}
    (out/'provenance.json').write_text(json.dumps(report,indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    scene=np.concatenate(scene)
    fig,axes=plt.subplots(1,2,figsize=(12,5))
    for ax,dim,title in [(axes[0],0,'Front: cross-passage x vs height'),(axes[1],1,'Side: along-passage y vs height')]:
        ax.scatter(scene[:,dim],scene[:,2],s=.5,c='0.7',label='Recorded registered cloud')
        ax.scatter(blocked[:,dim],blocked[:,2],s=25,c='red',label='Remaining occupied voxels')
        support=snap['support_candidates_yellow_diagnostic']
        ax.scatter(support[:,dim],support[:,2],s=8,c='green',label='Observed support candidates')
        ax.set(xlabel=('odom x [m]' if dim==0 else 'odom y [m]'),ylabel='odom z [m]',title=title)
        ax.grid(alpha=.2)
    axes[0].legend(fontsize=8);fig.tight_layout();fig.savefig(out/'upper_blockers.png',dpi=150)
    print(json.dumps({k:{a:v for a,v in d.items() if not isinstance(v,list)} for k,d in reports.items()},indent=2))


if __name__=='__main__':main()
