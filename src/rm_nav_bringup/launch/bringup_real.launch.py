import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction, LogInfo
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals, IfCondition


def generate_launch_description():
    # Get the launch directory
    rm_nav_bringup_dir = get_package_share_directory('rm_nav_bringup')
    navigation2_launch_dir = os.path.join(get_package_share_directory('rm_navigation'), 'launch')

    # Create the launch configuration variables
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lio_rviz = LaunchConfiguration('lio_rviz')
    use_nav_rviz = LaunchConfiguration('nav_rviz')
    rviz_mapping_config = os.path.join(get_package_share_directory('rm_navigation'), 'rviz', 'rog_map_mapping.rviz')
    rviz_default_config = os.path.join(get_package_share_directory('rm_navigation'), 'rviz', 'nav2.rviz')
    perception = LaunchConfiguration('perception')
    passage_observer = LaunchConfiguration('passage_observer')
    passage_virtual_preview = LaunchConfiguration('passage_virtual_preview')

    ################################ robot_description parameters start ###############################
    launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('rm_nav_bringup'), 'config', 'reality', 'measurement_params_real.yaml')))
    robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('rm_nav_bringup'), 'urdf', 'sentry_robot_real.xacro'),
    ' xyz:=', launch_params['base_link2livox_frame']['xyz'], ' rpy:=', launch_params['base_link2livox_frame']['rpy']])
    ################################# robot_description parameters end ################################

    ########################## linefit_ground_segementation parameters start ##########################
    segmentation_params = os.path.join(rm_nav_bringup_dir, 'config', 'reality', 'segmentation_real.yaml')
    ########################## linefit_ground_segementation parameters end ############################

    #################################### FAST_LIO parameters start ####################################
    fastlio_mid360_params = os.path.join(rm_nav_bringup_dir, 'config', 'reality', 'fastlio_mid360_real.yaml')
    fastlio_rviz_cfg_dir = os.path.join(rm_nav_bringup_dir, 'rviz', 'fastlio.rviz')
    ##################################### FAST_LIO parameters end #####################################

    ################################### POINT_LIO parameters start ####################################
    pointlio_mid360_params = os.path.join(rm_nav_bringup_dir, 'config', 'reality', 'pointlio_mid360_real.yaml')
    pointlio_rviz_cfg_dir = os.path.join(rm_nav_bringup_dir, 'rviz', 'pointlio.rviz')
    #################################### POINT_LIO parameters end #####################################

    ################################## slam_toolbox parameters start ##################################
    slam_toolbox_map_dir = PathJoinSubstitution([rm_nav_bringup_dir, 'map', world])
    slam_toolbox_localization_file_dir = os.path.join(rm_nav_bringup_dir, 'config', 'reality', 'mapper_params_localization_real.yaml')
    slam_toolbox_mapping_file_dir = os.path.join(rm_nav_bringup_dir, 'config', 'reality', 'mapper_params_online_async_real.yaml')
    with open(slam_toolbox_mapping_file_dir, encoding='utf-8') as stream:
        mapper_params = yaml.safe_load(stream)['slam_toolbox']['ros__parameters']
    mapper_min_travel_distance = float(mapper_params['minimum_travel_distance'])
    mapper_min_travel_heading = float(mapper_params['minimum_travel_heading'])
    ################################### slam_toolbox parameters end ###################################

    ################################### navigation2 parameters start ##################################
    nav2_map_dir = PathJoinSubstitution([rm_nav_bringup_dir, 'map', world]), ".yaml"
    # shadow 仅旁路运行新感知；legacy 完全使用原感知。
    # SLAM 使用原始 /scan 建持久地图；ROGMap 仅提供滚动地形层，不能把其
    # 随时间衰减的二维投影反算成激光扫描写入全局地图。
    nav2_params_file_dir = PathJoinSubstitution([
        rm_nav_bringup_dir, 'config', 'reality',
        PythonExpression(["'nav2_params_rog_map_real.yaml' if '", perception,
                          "' == 'rog_map' else 'nav2_params_real.yaml'"])
    ])
    ################################### navigation2 parameters end ####################################

    ################################ icp_registration parameters start ################################
    icp_pcd_dir = PathJoinSubstitution([rm_nav_bringup_dir, 'PCD', world]), ".pcd"
    icp_registration_params_dir = os.path.join(rm_nav_bringup_dir, 'config', 'reality', 'icp_registration_real.yaml')
    ################################# icp_registration parameters end #################################

    ############################# pointcloud_downsampling parameters start ############################
    pointcloud_downsampling_config_dir = os.path.join(rm_nav_bringup_dir, 'config', 'reality', 'pointcloud_downsampling_real.yaml')
    ############################# pointcloud_downsampling parameters start ############################

    ####################### Livox_ros_driver2 parameters start #######################
    xfer_format   = 4    # 0-PointCloud2Msg(PointXYZRTL), 1-LivoxCustomMsg, 2-PclPxyziMsg, 3-LivoxImuMsg, 4-AllMsg
    multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src      = 0    # 0-lidar, others-Invalid data src
    publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
    output_type   = 0
    frame_id      = 'livox_frame'
    lvx_file_path = '/home/livox/livox_test.lvx'
    cmdline_bd_code = 'livox0000000001'

    cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
    cur_config_path = cur_path + '../config'
    user_config_path = os.path.join(cur_config_path, 'reality', 'MID360_config.json')

    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code}
    ]
    ####################### Livox_ros_driver2 parameters end #########################

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use reality (Gazebo) clock if true')

    declare_use_lio_rviz_cmd = DeclareLaunchArgument(
        'lio_rviz',
        default_value='False',
        description='Visualize FAST_LIO or Point_LIO cloud_map if true')
    
    declare_nav_rviz_cmd = DeclareLaunchArgument(
        'nav_rviz',
        default_value='True',
        description='Visualize navigation2 if true')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='328',
        description='Select world (map file, pcd file, world file share the same name prefix as the this parameter)')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='',
        description='Choose mode: nav, mapping')
    
    declare_LIO_cmd = DeclareLaunchArgument(
        'lio',
        default_value='pointlio',
        choices=['fastlio', 'pointlio'],
        description='Choose lio alogrithm: fastlio or pointlio')

    declare_perception_cmd = DeclareLaunchArgument(
        'perception', default_value='rog_map', choices=['rog_map', 'shadow', 'legacy'],
        description='ROGMap terrain, shadow comparison, or original perception')

    declare_passage_observer_cmd = DeclareLaunchArgument(
        'passage_observer', default_value='false', choices=['true', 'false'],
        description='Opt-in read-only 3D passage clearance diagnostics; never issues motion permits')

    declare_passage_virtual_preview_cmd = DeclareLaunchArgument(
        'passage_virtual_preview', default_value='false', choices=['true', 'false'],
        description='Visualize 0.32 m virtual body clearance on the old car; no passage evidence or motion permit')

    terrain_nodes = []
    original_body_publish = {}
    for lio_name, lio_params in [('fastlio', fastlio_mid360_params),
                                 ('pointlio', pointlio_mid360_params)]:
        # 射线起点使用本车已有的雷达-IMU 外参，不复制来源双雷达车的安装标定。
        with open(lio_params, encoding='utf-8') as stream:
            lio_config = yaml.safe_load(stream)['/**']['ros__parameters']
        sensor_offset = lio_config['mapping']['extrinsic_T']
        original_body_publish[lio_name] = lio_config['publish']['scan_bodyframe_pub_en']
        terrain_input = {}
        if lio_name == 'pointlio':
            # 旧 Point-LIO 的 body 输出没有逐点去畸变，不能套用 FAST-LIO 的整帧变换。
            # 稠密支路复用逐点滤波位姿，不改变定位降采样与原有 world 输出。
            terrain_input = {'cloud_topic': '/cloud_registered_dense_terrain',
                             'input_frame': 'odom', 'cloud_coordinates': 'odom'}
        terrain_nodes.append(Node(
            package='sentry_terrain_bridge', executable='terrain_node', name='sentry_terrain',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", perception, "' != 'legacy' and '", LaunchConfiguration('lio'),
                "' == '", lio_name, "'"
            ])),
            parameters=[os.path.join(rm_nav_bringup_dir, 'config', 'reality', 'rog_map_real.yaml'),
                        {'use_sim_time': use_sim_time, 'sensor_offset': sensor_offset,
                         'mapping_scan_enabled': False,
                         'clearance.enabled': ParameterValue(PythonExpression([
                             "'", passage_observer, "' == 'true' or '",
                             passage_virtual_preview, "' == 'true'"]), value_type=bool),
                         'clearance.virtual_preview': ParameterValue(
                             passage_virtual_preview, value_type=bool),
                         # 正式洞口观测来自 ROGMap 分类；净空仅供显式诊断/虚拟预览。
                         'clearance.publish_verified_segments': False,
                         'clearance.robot_length': ParameterValue(
                             LaunchConfiguration('passage_robot_length'), value_type=float),
                         'clearance.robot_width': ParameterValue(
                             LaunchConfiguration('passage_robot_width'), value_type=float),
                         # 当前接入的仍是高约 0.70 m 的老车。新车目标高度
                         # 0.32 m 不能在雷达/底盘坐标标定前套用到实车启动。
                         'clearance.robot_height': ParameterValue(PythonExpression([
                             "0.32 if '", passage_virtual_preview,
                             "' == 'true' else ", LaunchConfiguration('passage_robot_height')]), value_type=float),
                         'clearance.travel_length': 0.20,
                         'clearance.max_observation_age': 60.0,
                         'clearance.entry_recent_age': 2.0,
                         'clearance.confirmations': 3}, terrain_input]
        ))
    pointlio_terrain_warning = LogInfo(
        condition=IfCondition(PythonExpression([
            "'", perception, "' != 'legacy' and '", LaunchConfiguration('lio'), "' == 'pointlio'"
        ])),
        msg='[地形迁移] 当前旧 Point-LIO 仅提供稀疏配准点云；先用 shadow 检查低矮/细小障碍，不能视作密集感知已验证。')
    virtual_passage_warning = LogInfo(
        condition=IfCondition(passage_virtual_preview),
        msg='[虚拟洞口预览] 仅按 0.32 m 虚拟车体显示净空；老车实际高约 0.70 m。不会发布通行证据、修正地图或给底盘速度。')

    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='',
        description='Choose localization method: slam_toolbox, amcl, icp')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
        output='screen'
    )

    # Specify the actions
    start_livox_ros_driver2_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    bringup_imu_complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ],
        remappings=[
            ('/imu/data_raw', '/livox/imu'),
        ]
    )

    bringup_linefit_ground_segmentation_node = Node(
        # SLAM 建图和导航定位都使用实测点云生成的 /scan。
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        output='screen',
        parameters=[segmentation_params]
    )

    bringup_pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in',  ['/segmentation/obstacle']),
                    ('scan',  ['/scan'])],
        parameters=[{
            'target_frame': PythonExpression([
                "'lidar_scan' if '", LaunchConfiguration('lio'), "' == 'pointlio' else 'livox_frame'"]),
            'transform_tolerance': 0.01,
            'min_height': -0.20,
            'max_height': 0.5,
            'angle_min': -3.14159,  # -M_PI/2
            'angle_max': 3.14159,   # M_PI/2
            'angle_increment': 0.003,  # M_PI/360.0    0.0043
            'scan_time': 0.3333,
            'range_min': 0.48,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )

    bringup_LIO_group = GroupAction([
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     # Copy from the 'livox_joint' in 'sentry_robot.xacro'.
        #     arguments=[
        #         # Useless arguments, provided by LIO in publish_odometry() function
        #         # '--x', '0.0',
        #         # '--y', '0.0',
        #         # '--z', '0.0',
        #         # '--roll', '0.0',
        #         # '--pitch', '0.0',
        #         # '--yaw', '0.0',
        #         '--frame-id', 'odom',
        #         '--child-frame-id', 'odom'
        #     ],
        # ),

        GroupAction(
            condition = LaunchConfigurationEquals('lio', 'fastlio'),
            actions=[
            Node(
                package='fast_lio',
                executable='fastlio_mapping',
                parameters=[
                    fastlio_mid360_params,
                    {'use_sim_time': use_sim_time,
                     # 只启用已有的 body 点云输出；原 /cloud_registered 密度完全不变。
                     'publish.scan_bodyframe_pub_en': ParameterValue(PythonExpression([
                         str(original_body_publish['fastlio']), " if '", perception,
                         "' == 'legacy' else True"]), value_type=bool)}
                ],
                output='screen'
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', fastlio_rviz_cfg_dir],
                condition = IfCondition(use_lio_rviz),
            ),
        ]),

        GroupAction(
            condition = LaunchConfigurationEquals('lio', 'pointlio'),
            actions=[
            Node(
                package='point_lio',
                executable='pointlio_mapping',
                name='laserMapping',
                output='screen',
                parameters=[
                    pointlio_mid360_params,
                    {'use_sim_time': use_sim_time,
                    'use_imu_as_input': False,  # Change to True to use IMU as input of Point-LIO
                    'prop_at_freq_of_imu': True,
                    'check_satu': True,
                    'init_map_size': 10,
                    'point_filter_num': 3,  # Options: 1, 3
                    'space_down_sample': True,
                    'filter_size_surf': 0.5,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1
                    'filter_size_map': 0.5,  # Options: 0.5, 0.3, 0.15, 0.1
                    'ivox_nearby_type': 6,   # Options: 0, 6, 18, 26
                    'publish.dense_terrain_en': ParameterValue(PythonExpression([
                        "'", perception, "' != 'legacy'"]), value_type=bool),
                    'runtime_pos_log_enable': False,
                    'pcd_save.pcd_save_en': ParameterValue(
                        PythonExpression(["'", LaunchConfiguration('mode'), "' == 'mapping'"]), value_type=bool),
                    'pcd_save.interval': -1}
                ],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', pointlio_rviz_cfg_dir],
                condition = IfCondition(use_lio_rviz),
            )
        ])
    ])

    start_localization_group = GroupAction(
        condition = LaunchConfigurationEquals('mode', 'nav'),
        actions=[
            Node(
                condition = LaunchConfigurationEquals('localization', 'slam_toolbox'),
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                    slam_toolbox_localization_file_dir,
                    {'use_sim_time': use_sim_time,
                    'map_file_name': slam_toolbox_map_dir,
                    'map_start_pose': [0.0, 0.0, 0.0]}
                ],
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir,'localization_amcl_launch.py')),
                condition = LaunchConfigurationEquals('localization', 'amcl'),
                launch_arguments = {
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file_dir,
                    'map': nav2_map_dir}.items()
            ),

            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        condition=LaunchConfigurationEquals('localization', 'icp'),
                        package='icp_registration',
                        executable='icp_registration_node',
                        output='screen',
                        parameters=[
                            icp_registration_params_dir,
                            {'use_sim_time': use_sim_time,
                                'pcd_path': icp_pcd_dir}
                        ],
                        # arguments=['--ros-args', '--log-level', ['icp_registration:=', 'DEBUG']]
                    )
                ]
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'map_server_launch.py')),
                condition = LaunchConfigurationNotEquals('localization', 'slam_toolbox'),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': nav2_map_dir,
                    'params_file': nav2_params_file_dir,
                    'container_name': 'nav2_container'}.items())
        ]
    )

    # bringup_fake_vel_transform_node = Node(
    #     package='fake_vel_transform',
    #     executable='fake_vel_transform_node',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'spin_speed': 0.0 # rad/s
    #     }]
    # )

    start_mapping = Node(
        condition = LaunchConfigurationEquals('mode', 'mapping'),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_mapping_file_dir,
            {'use_sim_time': use_sim_time,
             'scan_topic': '/scan',
             'minimum_travel_distance': mapper_min_travel_distance,
             'minimum_travel_heading': mapper_min_travel_heading}
        ],
    )

    start_navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'bringup_rm_navigation.py')),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('mode'), "' == 'nav' and '",
            LaunchConfiguration('start_nav2'), "'.lower() == 'true'"])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': nav2_map_dir,
            'params_file': nav2_params_file_dir,
            'nav_rviz': use_nav_rviz,
            'rviz_config': rviz_default_config}.items()
    )

    # 建图只需要 SLAM Toolbox 提供 map→odom 与全局 /map；Nav2 控制器、规划器
    # 和代价地图应留到 nav 模式。RViz 独立启动，避免为看地图而拉起整套导航。
    start_mapping_rviz = GroupAction(
        condition=LaunchConfigurationEquals('mode', 'mapping'),
        actions=[Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', PythonExpression([
                repr(rviz_mapping_config), " if '", perception,
                "' == 'rog_map' else ", repr(rviz_default_config)])],
            condition=IfCondition(use_nav_rviz))]
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_lio_rviz_cmd)
    ld.add_action(declare_nav_rviz_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_localization_cmd)
    ld.add_action(declare_LIO_cmd)
    ld.add_action(declare_perception_cmd)
    ld.add_action(declare_passage_observer_cmd)
    ld.add_action(declare_passage_virtual_preview_cmd)
    ld.add_action(DeclareLaunchArgument('passage_runtime', default_value='false',
        choices=['true', 'false'], description='Publish measured passage evidence; never use with virtual preview'))
    ld.add_action(DeclareLaunchArgument('passage_robot_height', default_value='0.30'))
    ld.add_action(DeclareLaunchArgument('passage_robot_length', default_value='0.62'))
    ld.add_action(DeclareLaunchArgument('passage_robot_width', default_value='0.58'))
    ld.add_action(DeclareLaunchArgument('start_nav2', default_value='true'))
    
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_livox_ros_driver2_node)
    ld.add_action(bringup_imu_complementary_filter_node)
    ld.add_action(bringup_linefit_ground_segmentation_node)
    ld.add_action(bringup_pointcloud_to_laserscan_node)
    ld.add_action(bringup_LIO_group)
    ld.add_action(pointlio_terrain_warning)
    ld.add_action(virtual_passage_warning)
    for terrain_node in terrain_nodes:
        ld.add_action(terrain_node)
    ld.add_action(start_localization_group)
    ld.add_action(start_mapping)
    ld.add_action(start_navigation2)
    ld.add_action(start_mapping_rviz)

    return ld
