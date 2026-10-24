"""Map-associated full 3D history and persistent continuous static corrections.

A single map-session history collection holds exact source geometry groups.
Observation duplicates do not allocate separate clearance/permit records.
Previously admitted geometry is never replaced by current mask coverage.
"""
from dataclasses import asdict, dataclass
from math import isfinite
import numpy as np
from scipy.spatial.transform import Rotation

from global_passage_memory import MapContext
from rigid_transform import normalize_transform, context_transform, transforms_close
from structure_observation import validate_columns
from passage_memory_v2 import PassageMemoryV2, _is_mapping_expansion, _cell_hash
from passage_record_io import read_record, packed_body, atomic_write_record, backup_previous_record
from passage_region import project_columns, polygon_cells
from continuous_wall_region import terrain_free_cells, continuous_wall_cells


@dataclass(frozen=True)
class RegionState:
    cells: tuple


class ContinuousPassageMemory:
    continuous = True

    def __init__(self):
        self.context = None
        self.groups = {}
        self.regions = set()
        self.cells = set()  # Current-canvas coverage, including temporarily free/unknown cells.
        self.mask_cells = set()
        self.segments = {}  # This model never creates old body proofs or entry permits.
        self.state, self.reason = 'UNKNOWN', 'NO_HOLLOW_HISTORY'
        self.needs_save = False
        self._grid = None
        self._observed = None
        self._source_seen, self._world_seen = set(), set()
        self._last_refinement = None
        self.component_count = 0
        self.backup_path = None
        self.migrated_from = None

    @property
    def structures(self):
        # One history collection per map association; its geometry groups are not permits.
        return {'classified_history': RegionState(tuple(sorted(self.mask_cells)))} if self.groups else {}

    def suspend(self, reason):
        self.state, self.reason = 'SUSPENDED', reason

    @staticmethod
    def _geometry(context):
        return (context.width, context.height, context.resolution, context.origin_x, context.origin_y)

    def parts(self):
        for (frame, resolution, transform), columns in sorted(self.groups.items()):
            yield dict(source_frame=frame, resolution=resolution,
                       map_transform=transform, columns=sorted(columns))

    @staticmethod
    def _world_keys(part):
        columns=np.asarray(part['columns'],dtype=float);tr=part['map_transform'];r=part['resolution']
        rotation=Rotation.from_quat(tr[3:]);lower=rotation.apply(columns[:,:3])+tr[:3]
        upper=columns[:,:3].copy();upper[:,2]=columns[:,3];upper=rotation.apply(upper)+tr[:3]
        quantized=np.floor(np.column_stack((lower,upper[:,2]))/r+1e-7).astype(np.int64)
        return {tuple(int(v) for v in row) for row in quantized}

    def _add_part(self, part, *, force=False):
        validate_columns(part['columns'], part['resolution'])
        frame=part['source_frame']
        if not isinstance(frame,str) or not frame:
            raise ValueError('INVALID_SOURCE_FRAME')
        key=(frame,part['resolution'],normalize_transform(part['map_transform']))
        rows={tuple(row) for row in part['columns']}
        source={(frame,part['resolution'],row) for row in rows}
        world=self._world_keys(dict(part,map_transform=key[2]))
        # Admission removes repeated views with no new source/world voxel geometry.
        # For every admitted view, keep all columns and its exact original TF.
        if not force and source<=self._source_seen and world<=self._world_seen:
            return False
        self.groups.setdefault(key,set()).update(rows)
        self._source_seen.update(source);self._world_seen.update(world)
        if self._observed is not None:
            self._observed.update(project_columns([dict(part,map_transform=key[2])],self._geometry(self.context)))
        self._last_refinement=None
        self.needs_save=True
        return True

    def _add_regions(self, context, selected):
        r=context.resolution
        rects={(context.origin_x+(i%context.width)*r,context.origin_y+(i//context.width)*r,
                context.origin_x+(i%context.width+1)*r,context.origin_y+(i//context.width+1)*r) for i in selected}
        new=rects-self.regions
        if new:
            self.regions.update(new);self.cells.update(selected);self.needs_save=True
        return bool(new)

    def check(self, context, grid):
        if self.state=='SUSPENDED':
            return False
        if not context.valid() or len(grid)!=context.width*context.height or self.context is None:
            self.suspend('MAP_UNAVAILABLE');return False
        previous=self.context
        if context.epoch!=previous.epoch:
            self.suspend('MAP_RELOAD_OR_GEOMETRY_CHANGED');return False
        if not transforms_close(context_transform(previous),context_transform(context),.25,.15):
            self.suspend('MAP_ODOM_ASSOCIATION_MOVED');return False
        if context.geometry()!=previous.geometry():
            # Growing SLAM canvases may retract an opposite edge by one cell.
            # Keep world geometry, clip only its current raster, and retain TF/epoch guards.
            if not _is_mapping_expansion(previous,context,max_edge_retraction=context.resolution):
                self.suspend('MAP_RELOAD_OR_GEOMETRY_CHANGED');return False
            cells=set()
            for x0,y0,x1,y1 in self.regions:
                cells.update(polygon_cells([[x0,y0],[x1,y0],[x1,y1],[x0,y1]],self._geometry(context)))
            self.cells=cells;self.context=context;self._observed=None;self._last_refinement=None
            self.needs_save=True
            self.reason='MAPPING_EXPANSION_REPROJECTED'
        values=tuple(grid)
        if self._grid is not None and values!=self._grid:
            if context.epoch.startswith('saved-map-') and any(grid[i]!=self._grid[i] for i in self.cells):
                self.suspend('SAVED_MAP_PATCH_CHANGED');return False
            self.needs_save=True;self._last_refinement=None
        self._grid=values
        self.mask_cells={i for i in self.cells if grid[i]==100}
        self.state='KNOWN_PASSAGE' if self.groups else 'UNKNOWN'
        return bool(self.groups)

    def add_structure(self, *, context, grid, observation, cells, map_transform, now):
        if (not context.valid() or len(grid)!=context.width*context.height or
                not isfinite(observation['measured_at']) or not 0<=now-observation['measured_at']<=2):
            raise ValueError('INVALID_STRUCTURE_OBSERVATION')
        if self.context is not None and not self.check(context,grid) and self.state=='SUSPENDED':
            raise ValueError(self.reason)
        if self.context is None:self.context=context;self._grid=tuple(grid)
        part=dict(source_frame=observation['frame'],resolution=observation['resolution'],
                  columns=observation['columns'],map_transform=normalize_transform(map_transform))
        admitted=self._add_part(part)
        if admitted:self.context=context
        self.state,self.reason='KNOWN_PASSAGE','CLASSIFIED_HOLLOW_HISTORY'
        return 'classified_history' if admitted else None

    def refine(self, context, grid, terrain, transform):
        if not self.groups or not self.check(context,grid):return
        token=(terrain.header.stamp.sec,terrain.header.stamp.nanosec,tuple(transform))
        if token==self._last_refinement:return
        if self._observed is None:
            self._observed=project_columns(self.parts(),self._geometry(context))
        free=terrain_free_cells(self._geometry(context),terrain,transform)
        associated,components=continuous_wall_cells(self._geometry(context),grid,self._observed,free)
        self.component_count=len(components)
        selected=(self._observed|associated)&free&{i for i,v in enumerate(grid) if v==100}
        if self._add_regions(context,selected):self.reason='CONTINUOUS_STATIC_REGION_EXTENDED'
        self.mask_cells={i for i in self.cells if grid[i]==100}
        self._last_refinement=token

    def planning_cells(self, context, grid):
        return set(self.mask_cells) if self.check(context,grid) else set()

    def body(self):
        return dict(version=4,context=asdict(self.context),segments=[],
                    geometry_groups=list(self.parts()),correction_regions=sorted(self.regions),
                    cells=sorted(self.mask_cells),map_patch_hash=_cell_hash(self._grid,sorted(self.mask_cells),allow_empty=True))

    def save(self, path):
        if self.state!='KNOWN_PASSAGE' or not self.groups:raise ValueError('NO_ACTIVE_RECORDS')
        try:
            body=self.body();packed_body(body)
            self.backup_path=backup_previous_record(path, self.context.epoch) or self.backup_path
            atomic_write_record(path,body)
            self.needs_save=False
        except (OSError,ValueError,TypeError,OverflowError):
            self.suspend('RECORD_WRITE_FAILED')
            raise

    @classmethod
    def from_body(cls, body):
        if body.get('version')!=4 or body.get('segments')!=[]:
            raise ValueError('INVALID_CONTINUOUS_RECORD')
        m=cls();m.context=MapContext(**body['context'])
        if not m.context.valid():raise ValueError('INVALID_RECORD_CONTEXT')
        parts=body['geometry_groups'];regions=body['correction_regions'];cells=body['cells']
        if (not isinstance(parts,list) or not 0<len(parts)<=100_000 or not isinstance(regions,list) or
                len(regions)>100_000 or not isinstance(cells,list) or len(cells)!=len(set(cells)) or
                any(type(i) is not int or not 0<=i<m.context.width*m.context.height for i in cells)):
            raise ValueError('INVALID_CONTINUOUS_RECORD')
        for part in parts:m._add_part(part,force=True)
        for rect in regions:
            if (not isinstance(rect,(list,tuple)) or len(rect)!=4 or
                    any(type(v) not in (int,float) or not isfinite(v) for v in rect) or
                    rect[0]>=rect[2] or rect[1]>=rect[3]):raise ValueError('INVALID_CORRECTION_REGION')
            m.regions.add(tuple(rect))
        for x0,y0,x1,y1 in m.regions:
            m.cells.update(polygon_cells([[x0,y0],[x1,y0],[x1,y1],[x0,y1]],m._geometry(m.context)))
        if not set(cells)<=m.cells:raise ValueError('CORRECTION_CELLS_OUTSIDE_REGION')
        m.mask_cells=set(cells);m.state='KNOWN_PASSAGE';m.needs_save=False
        return m

    @classmethod
    def restore(cls, path, context, grid):
        m=cls()
        try:
            body=read_record(path)
            if body['version'] in (2,3):
                legacy=PassageMemoryV2.restore(path,context,grid)
                if legacy.state=='SUSPENDED':raise ValueError(legacy.reason)
                m.context=legacy.context;m._grid=tuple(grid)
                for record in legacy.structures.values():m._add_part(asdict(record),force=True)
                if not m.groups:
                    m.reason='LEGACY_BODY_RECORD_HAS_NO_HOLLOW_STRUCTURE';return m
                m._add_regions(context,legacy.planning_cells(context,grid))
                m.migrated_from=body['version'];m.needs_save=True
                m.state,m.reason='KNOWN_PASSAGE','LEGACY_HOLLOW_HISTORY_MIGRATED'
            else:
                m=cls.from_body(body)
                saved=m.context
                if saved.epoch.startswith('saved-map-') and saved.epoch==context.epoch and saved.geometry()==context.geometry():
                    if _cell_hash(grid,body['cells'],allow_empty=True)!=body['map_patch_hash']:
                        raise ValueError('SAVED_MAP_PATCH_CHANGED')
                    m.context=context
                m.reason='RESTORED_CONTINUOUS_HISTORY'
            m.check(context,grid)
        except (OSError,ValueError,TypeError,KeyError,OverflowError,AttributeError):
            m=cls();m.suspend('RECORD_UNAVAILABLE_OR_INVALID')
        return m

    def diagnostics(self):
        return dict(memory_model='continuous_v4',history_group_count=len(self.groups),
                    history_column_count=sum(len(c) for c in self.groups.values()),
                    continuous_component_count=self.component_count,
                    correction_region_count=len(self.regions),migrated_from=self.migrated_from,
                    migration_backup=self.backup_path)
