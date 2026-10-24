"""Build a self-contained SDF from explicit tunnel geometry; no model downloads."""
import sys
from xml.etree import ElementTree as ET


def tag(parent, tag_name, text=None, **attrs):
    element = ET.SubElement(parent, tag_name, attrs)
    if text is not None:
        element.text = str(text)
    return element


def box(link, name, pose, size):
    for kind in ('collision', 'visual'):
        part = tag(link, kind, name=name)
        tag(part, 'pose', pose)
        tag(tag(tag(part, 'geometry'), 'box'), 'size', size)


def make_world(dense=False, mobile=False):
    sdf = ET.Element('sdf', version='1.6')
    world = tag(sdf, 'world', name='passage_lab')
    tag(world, 'gravity', '0 0 -9.81')
    physics = tag(world, 'physics', name='ode', type='ode')
    tag(physics, 'max_step_size', '0.002')
    tag(physics, 'real_time_update_rate', '500')
    scene = tag(world, 'scene')
    tag(scene, 'ambient', '0.65 0.65 0.65 1')
    tag(scene, 'background', '0.2 0.22 0.25 1')
    tag(scene, 'shadows', 'false')
    light = tag(world, 'light', name='sun', type='directional')
    tag(light, 'pose', '0 0 10 0 0 0')
    tag(light, 'diffuse', '0.8 0.8 0.8 1')
    tag(light, 'direction', '-0.3 0.1 -1')
    state = tag(world, 'plugin', name='state', filename='libgazebo_ros_state.so')
    tag(tag(state, 'ros'), 'namespace', '/passage_sim')
    tag(state, 'update_rate', '20')

    def model(name, pose, static=True):
        m = tag(world, 'model', name=name)
        tag(m, 'static', 'true' if static else 'false')
        tag(m, 'pose', pose)
        return m, tag(m, 'link', name='structure')

    _, floor = model('ground', '0 0 -0.05 0 0 0')
    box(floor, 'floor', '0 0 0 0 0 0', '30 30 0.1')
    for name, y, width, height in [('wide_passage', 0, 1.2, .95),
                                    ('narrow_passage', 2, .30, .95),
                                    ('low_passage', -2, 1.2, .50)]:
        _, link = model(name, f'2.6 {y} 0 0 0 0')
        total = height + .15
        for sign, side in [(-1, 'right'), (1, 'left')]:
            box(link, side, f'0 {sign*(width/2+.1)} {total/2} 0 0 0', f'1.2 .2 {total}')
        box(link, 'roof', f'0 0 {height+.075} 0 0 0', f'1.2 {width+.4} .15')
    _, wall = model('solid_wall', '2.6 4 .65 0 0 0')
    box(wall, 'wall', '0 0 0 0 0 0', '.2 1.2 1.3')
    rig, lidar = model('sensor_rig', '0 0 0 0 0 0', static=not mobile)
    lidar.set('name', 'passage_lidar')
    tag(lidar, 'pose', '0 0 .55 0 0 0')
    if mobile:
        # The isolated planar drive is not a chassis dynamics model. Prevent
        # Gazebo's gravity from sinking this test proxy below the floor; real
        # vehicle tip-over remains explicitly outside this experiment.
        tag(lidar, 'gravity', 'false')
        # Lidar origin is 0.55 m above ground. Keep its collision ray origin
        # outside the proxy body; otherwise the scanner maps its own chassis.
        # The safety validator still checks the full 0.70 m swept envelope.
        body = tag(lidar, 'collision', name='robot_body')
        tag(body, 'pose', '0 0 -0.325 0 0 0')
        tag(tag(tag(body, 'geometry'), 'box'), 'size', '.45 .45 .45')
        inertial = tag(lidar, 'inertial')
        tag(inertial, 'mass', '8')
        moments = tag(inertial, 'inertia')
        for key, value in [('ixx', .46), ('iyy', .46), ('izz', .27),
                           ('ixy', 0), ('ixz', 0), ('iyz', 0)]:
            tag(moments, key, value)
        chassis = tag(lidar, 'visual', name='robot_body')
        tag(chassis, 'pose', '0 0 -0.325 0 0 0')
        tag(tag(tag(chassis, 'geometry'), 'box'), 'size', '.45 .45 .45')
        drive = tag(rig, 'plugin', name='planar_move', filename='libgazebo_ros_planar_move.so')
        drive_ros = tag(drive, 'ros')
        tag(drive_ros, 'namespace', '/passage_sim')
        tag(drive_ros, 'remapping', 'cmd_vel:=drive_cmd_vel')
        tag(drive, 'update_rate', '50')
        tag(drive, 'publish_odom', 'false')
        tag(drive, 'publish_odom_tf', 'false')
    visual = tag(lidar, 'visual', name='sensor')
    tag(tag(tag(visual, 'geometry'), 'sphere'), 'radius', '.05')
    sensor = tag(lidar, 'sensor', name='lidar', type='ray')
    tag(sensor, 'always_on', 'true')
    tag(sensor, 'visualize', 'false')
    tag(sensor, 'update_rate', '5' if not dense else '8')
    ray = tag(sensor, 'ray')
    scan = tag(ray, 'scan')
    for axis, count, extent in [('horizontal', 1440 if dense else 720, 3.14159265),
                                ('vertical', 96 if dense else 48, .8)]:
        section = tag(scan, axis)
        for key, value in [('samples', count), ('resolution', 1), ('min_angle', -extent), ('max_angle', extent)]:
            tag(section, key, value)
    limits = tag(ray, 'range')
    for key, value in [('min', .1), ('max', 8), ('resolution', .01)]:
        tag(limits, key, value)
    cloud = tag(sensor, 'plugin', name='cloud', filename='libgazebo_ros_ray_sensor.so')
    ros = tag(cloud, 'ros')
    tag(ros, 'namespace', '/passage_sim')
    tag(ros, 'remapping', '~/out:=points')
    tag(cloud, 'output_type', 'sensor_msgs/PointCloud2')
    tag(cloud, 'frame_name', 'passage_lidar')
    truth = tag(rig, 'plugin', name='truth', filename='libgazebo_ros_p3d.so')
    ros = tag(truth, 'ros')
    tag(ros, 'namespace', '/passage_sim')
    tag(ros, 'remapping', 'odom:=sensor_odom')
    for key, value in [('body_name', 'passage_lidar'), ('frame_name', 'world'), ('update_rate', 50), ('gaussian_noise', 0)]:
        tag(truth, key, value)
    tag(tag(tag(world, 'gui'), 'camera', name='overview'), 'pose', '-6 -7 7 0 .55 .65')
    return sdf


if __name__ == '__main__':
    root = make_world('--dense' in sys.argv[2:], '--mobile' in sys.argv[2:])
    ET.indent(root)
    ET.ElementTree(root).write(sys.argv[1], encoding='utf-8', xml_declaration=True)
