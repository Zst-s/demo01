"""Offline observed roof regions; no vehicle dimensions or manually chosen map ROI."""
import numpy as np
from scipy.spatial.transform import Rotation


def clip(poly, axis, bound, greater):
    result = []
    for a, b in zip(poly, poly[1:] + poly[:1]):
        ia = a[axis] >= bound if greater else a[axis] <= bound
        ib = b[axis] >= bound if greater else b[axis] <= bound
        if ia:
            result.append(a)
        if ia != ib:
            alpha = (bound - a[axis]) / (b[axis] - a[axis])
            result.append([a[0] + alpha*(b[0]-a[0]), a[1] + alpha*(b[1]-a[1])])
    return result


def area(poly):
    return abs(sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(poly,poly[1:]+poly[:1])))*.5


def polygon_cells(poly, geometry):
    width, height, resolution, ox, oy = geometry
    p = np.asarray(poly)
    low = np.floor((p.min(axis=0)-[ox,oy])/resolution).astype(int)
    high = np.floor((p.max(axis=0)-[ox,oy])/resolution).astype(int)
    cells = set()
    for y in range(max(0,low[1]),min(height-1,high[1])+1):
        for x in range(max(0,low[0]),min(width-1,high[0])+1):
            clipped = poly
            for axis,bound,greater in [(0,ox+x*resolution,True),(0,ox+(x+1)*resolution,False),
                                       (1,oy+y*resolution,True),(1,oy+(y+1)*resolution,False)]:
                clipped = clip(clipped,axis,bound,greater)
            if area(clipped)>resolution*resolution*1e-8:
                cells.add(y*width+x)
    return cells


def project_columns(parts, geometry, endpoint='lower'):
    result=set()
    for part in parts:
        rr=part['resolution'];tr=part['map_transform'];rotation=Rotation.from_quat(tr[3:]);translation=np.array(tr[:3])
        for x,y,lower,upper in part['columns']:
            z=lower if endpoint=='lower' else upper
            poly=(rotation.apply([[x-rr/2,y-rr/2,z],[x+rr/2,y-rr/2,z],
                                  [x+rr/2,y+rr/2,z],[x-rr/2,y+rr/2,z]])+translation)[:,:2].tolist()
            result.update(polygon_cells(poly,geometry))
    return result
