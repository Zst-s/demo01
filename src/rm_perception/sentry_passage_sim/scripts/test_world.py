"""Check the generated scene's geometry and absence of actuator plugins."""
import unittest
from generate_world import make_world


class WorldContract(unittest.TestCase):
    def test_no_external_model_or_drive_plugin(self):
        root = make_world()
        self.assertFalse(root.findall('.//include'))
        self.assertTrue(all(m.findtext('static') == 'true' for m in root.findall('.//model')))
        allowed = {'libgazebo_ros_state.so', 'libgazebo_ros_ray_sensor.so', 'libgazebo_ros_p3d.so'}
        self.assertEqual({p.attrib['filename'] for p in root.findall('.//plugin')}, allowed)

    def test_clear_openings_match_labels(self):
        root = make_world()
        for name, width, height in [('wide_passage', 1.2, .95), ('narrow_passage', .3, .95), ('low_passage', 1.2, .5)]:
            link = root.find(f".//model[@name='{name}']/link")
            left = link.find("collision[@name='left']")
            roof = link.find("collision[@name='roof']")
            center_y = float(left.findtext('pose').split()[1])
            side_width = float(left.findtext('geometry/box/size').split()[1])
            self.assertAlmostEqual(2*center_y-side_width, width)
            roof_z = float(roof.findtext('pose').split()[2])
            roof_height = float(roof.findtext('geometry/box/size').split()[2])
            self.assertAlmostEqual(roof_z-roof_height/2, height)

    def test_cloud_and_truth_use_same_sensor(self):
        root = make_world()
        rig = root.find(".//model[@name='sensor_rig']")
        link = rig.find('link')
        self.assertEqual(link.attrib['name'], rig.findtext("plugin[@name='truth']/body_name"))
        self.assertEqual(link.attrib['name'], link.findtext('sensor/plugin/frame_name'))
        self.assertEqual(link.findtext('pose'), '0 0 .55 0 0 0')

    def test_mobile_world_has_one_controlled_body(self):
        root = make_world(dense=True, mobile=True)
        dynamic = [m for m in root.findall('.//model') if m.findtext('static') == 'false']
        self.assertEqual([m.attrib['name'] for m in dynamic], ['sensor_rig'])
        rig = dynamic[0]
        self.assertEqual(rig.findtext("plugin[@name='planar_move']/ros/remapping"),
                         'cmd_vel:=drive_cmd_vel')
        self.assertEqual(rig.findtext("plugin[@name='planar_move']/publish_odom_tf"), 'false')
        self.assertEqual(rig.findtext("link/collision[@name='robot_body']/geometry/box/size"),
                         '.45 .45 .70')


if __name__ == '__main__':
    unittest.main()
