"""Offline wall-band association anchored to observed 3D hollow structure footprints.

This does not classify free terrain as a tunnel. Only static wall fragments
connected to a structure seed, inside that component's measured lateral extent,
can be added. A single missing raster cell may separate wall fragments. Live
occupied, unknown and out-of-window cells cannot be added or bridge fragments.
"""
from collections import deque
from math import cos, sin, floor, isfinite
import numpy as np
from scipy.ndimage import label
from scipy.spatial.transform import Rotation


def terrain_free_cells(geometry, terrain, transform):
    """Same conservative XY sampling extent as TerrainLayer::updateCosts."""
    width,height,resolution,ox,oy=geometry
    rotation=Rotation.from_quat(transform[3:]);roll,pitch,yaw=rotation.as_euler('xyz')
    if abs(roll)>.1 or abs(pitch)>.1:
        raise ValueError('TERRAIN_TRANSFORM_NOT_PLANAR')
    c,s=cos(yaw),sin(yaw);half=.5*resolution*(abs(c)+abs(s));rr=terrain.info.resolution
    if not isfinite(rr) or rr<=0 or len(terrain.data)!=terrain.info.width*terrain.info.height:
        raise ValueError('INVALID_TERRAIN_GRID')
    values=np.asarray(terrain.data).reshape(terrain.info.height,terrain.info.width)
    iy,ix=np.indices((height,width));wx=ox+(ix.ravel()+.5)*resolution-transform[0];wy=oy+(iy.ravel()+.5)*resolution-transform[1]
    x,y=c*wx+s*wy,-s*wx+c*wy
    x0=np.floor((x-half-terrain.info.origin.position.x)/rr).astype(int);x1=np.floor((x+half-1e-9-terrain.info.origin.position.x)/rr).astype(int)
    y0=np.floor((y-half-terrain.info.origin.position.y)/rr).astype(int);y1=np.floor((y+half-1e-9-terrain.info.origin.position.y)/rr).astype(int)
    valid=(x0>=0)&(y0>=0)&(x1<terrain.info.width)&(y1<terrain.info.height)
    ids=np.flatnonzero(valid)
    # Summed-area table: every overlapped source cell must be exactly free.
    blocked=np.pad((values!=0).astype(np.int64),((1,0),(1,0))).cumsum(0).cumsum(1)
    count=blocked[y1[ids]+1,x1[ids]+1]-blocked[y0[ids],x1[ids]+1]-blocked[y1[ids]+1,x0[ids]]+blocked[y0[ids],x0[ids]]
    return set(int(i) for i in ids[count==0])


def continuous_wall_cells(geometry, grid, observed_cells, free_cells):
    width,height,resolution,ox,oy=geometry
    static=np.asarray(grid)==100
    if width<=0 or height<=0 or resolution<=0 or len(grid)!=width*height:
        raise ValueError('INVALID_MAP_GEOMETRY')
    if any(type(i) is not int or not 0<=i<width*height for i in observed_cells|free_cells):
        raise ValueError('INVALID_REGION_CELL')
    observed=np.zeros(width*height,dtype=bool);observed[list(observed_cells)]=True
    components,count=label(observed.reshape(height,width),structure=np.ones((3,3)))
    xy=np.array([[ox+(i%width+.5)*resolution,oy+(i//width+.5)*resolution] for i in range(width*height)])
    known=np.asarray(grid)>=0;available=np.zeros(width*height,dtype=bool);available[list(free_cells)]=True;available &= known
    selected=set();details=[]
    neighbours=[(dx,dy) for dx in range(-2,3) for dy in range(-2,3) if (dx,dy)!=(0,0)]
    for tag in range(1,count+1):
        ids=np.flatnonzero(components.ravel()==tag)
        if len(ids)<3:continue
        points=xy[ids];centre=points.mean(axis=0);values,vectors=np.linalg.eigh(np.cov(points.T));axis=vectors[:,-1]
        # Without a resolved lateral axis, retain direct footprints only.
        if values[0]<=resolution*resolution/12 or values[1]-values[0]<=resolution*resolution:
            continue
        lateral=(points[:,0]-centre[0])*axis[0]+(points[:,1]-centre[1])*axis[1];half=resolution*.5*np.abs(axis).sum();lo,hi=lateral.min()-half,lateral.max()+half
        all_lateral=(xy[:,0]-centre[0])*axis[0]+(xy[:,1]-centre[1])*axis[1];inside=(all_lateral>=lo)&(all_lateral<=hi)&static&available
        seeds=[int(i) for i in ids if inside[i]];queue=deque(seeds);seen=set(seeds)
        while queue:
            i=queue.popleft();x,y=i%width,i//width
            for dx,dy in neighbours:
                nx,ny=x+dx,y+dy
                if not(0<=nx<width and 0<=ny<height):continue
                j=ny*width+nx
                if not inside[j] or j in seen:continue
                # Do not jump an unknown/live obstacle while joining raster fragments.
                steps=2*max(abs(dx),abs(dy))
                bridge=[floor(y+.5+dy*k/steps)*width+floor(x+.5+dx*k/steps) for k in range(1,steps)]
                if not all(available[k] for k in bridge):continue
                seen.add(j);queue.append(j)
        selected.update(seen)
        if seeds:details.append(dict(observed_cells=ids.tolist(),wall_cells=sorted(seen),centre=centre.tolist(),width_axis=axis.tolist(),lateral_bounds=[float(lo),float(hi)]))
    return selected,details
