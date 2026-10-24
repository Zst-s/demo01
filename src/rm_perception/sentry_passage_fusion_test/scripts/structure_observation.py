"""ROGMap HOLLOW_TUNNEL columns; no vehicle or clearance-proof fields."""
from math import floor, isfinite
from rigid_transform import normalize_transform, rotation_matrix


def validate_columns(columns, resolution):
    if (type(resolution) not in (int, float) or not isfinite(resolution) or
            not 0 < resolution <= 1 or not isinstance(columns, (list, tuple)) or
            not 0 < len(columns) <= 100_000):
        raise ValueError('INVALID_STRUCTURE_COLUMNS')
    for column in columns:
        if (not isinstance(column, (list, tuple)) or len(column) != 4 or
                any(type(v) not in (int, float) or not isfinite(v) for v in column) or
                column[2] >= column[3]):
            raise ValueError('INVALID_STRUCTURE_COLUMN')


def parse_structure(msg, now, expected_frame):
    from sensor_msgs.msg import PointField
    from sensor_msgs_py.point_cloud2 import read_points
    measured_at = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if (msg.header.frame_id != expected_frame or not isfinite(now) or
            measured_at <= 0 or not 0 <= now-measured_at <= 2 or
            not 0 < msg.width * msg.height <= 100_000 or len(msg.data) > 4_000_000 or
            msg.point_step <= 0 or msg.row_step < msg.width * msg.point_step or
            len(msg.data) != msg.row_step * msg.height):
        raise ValueError('STALE_OR_INVALID_STRUCTURE')
    names = ('x', 'y', 'z', 'ceiling_z', 'resolution')
    fields = {field.name: field for field in msg.fields}
    if (len(fields) != len(msg.fields) or any(name not in fields or
            fields[name].datatype != PointField.FLOAT32 or fields[name].count != 1 or
            fields[name].offset < 0 or fields[name].offset+4 > msg.point_step
            for name in names)):
        raise ValueError('INVALID_STRUCTURE_FIELDS')
    rows = [tuple(float(v) for v in row) for row in
            read_points(msg, field_names=names, skip_nans=False)]
    resolution = rows[0][4]
    columns = [row[:4] for row in rows]
    validate_columns(columns, resolution)
    if any(not isfinite(row[4]) or abs(row[4]-resolution) > 1e-6 for row in rows):
        raise ValueError('INCONSISTENT_STRUCTURE_RESOLUTION')
    return dict(frame=expected_frame, measured_at=measured_at,
                resolution=resolution, columns=columns)


def structure_map_cells(observation, context, transform):
    """Map classified column centres through full 3D TF into map cells.

    A classified column contributes its lower occupied endpoint at the source
    XY centre. Requiring its face to cover a complete target square discards
    sparse observations when the two grids are translated or rotated.
    This performs no dilation, interpolation or vehicle-clearance check.
    """
    columns, resolution = observation['columns'], observation['resolution']
    validate_columns(columns, resolution)
    if not context.valid():
        raise ValueError('INVALID_STRUCTURE_MAP_CONTEXT')
    rigid = normalize_transform(transform)
    rotation = rotation_matrix(rigid)
    ox = min(p[0] for p in columns)-resolution/2
    oy = min(p[1] for p in columns)-resolution/2
    lattice, cells = set(), set()
    for x, y, lower, _ in columns:
        ix, iy = round((x-ox)/resolution-.5), round((y-oy)/resolution-.5)
        if (abs(x-(ox+(ix+.5)*resolution)) > resolution*1e-4 or
                abs(y-(oy+(iy+.5)*resolution)) > resolution*1e-4 or (ix, iy) in lattice):
            raise ValueError('INVALID_STRUCTURE_LATTICE')
        lattice.add((ix, iy))
        # Recover the shared source lattice after PointCloud2 float32 rounding.
        x, y = ox+(ix+.5)*resolution, oy+(iy+.5)*resolution
        wx = rigid[0]+rotation[0][0]*x+rotation[0][1]*y+rotation[0][2]*lower
        wy = rigid[1]+rotation[1][0]*x+rotation[1][1]*y+rotation[1][2]*lower
        mx = floor((wx-context.origin_x)/context.resolution+1e-9)
        my = floor((wy-context.origin_y)/context.resolution+1e-9)
        if 0 <= mx < context.width and 0 <= my < context.height:
            cells.add(my*context.width+mx)
    return tuple(sorted(cells))
