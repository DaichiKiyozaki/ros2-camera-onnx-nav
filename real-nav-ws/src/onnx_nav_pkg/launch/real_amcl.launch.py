from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # launch引数
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    global_frame_id = LaunchConfiguration('global_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')
    scan_topic = LaunchConfiguration('scan_topic')
    use_static_base_to_laser_tf = LaunchConfiguration('use_static_base_to_laser_tf')
    use_base_to_base_link_alias_tf = LaunchConfiguration('use_base_to_base_link_alias_tf')
    laser_frame_id = LaunchConfiguration('laser_frame_id')
    base_link_frame_id = LaunchConfiguration('base_link_frame_id')
    static_tf_x = LaunchConfiguration('static_tf_x')
    static_tf_y = LaunchConfiguration('static_tf_y')
    static_tf_z = LaunchConfiguration('static_tf_z')
    static_tf_yaw = LaunchConfiguration('static_tf_yaw')
    static_tf_pitch = LaunchConfiguration('static_tf_pitch')
    static_tf_roll = LaunchConfiguration('static_tf_roll')

    # 地図配信ノード
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
    )

    # AMCL 本体
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'global_frame_id': global_frame_id,
                'odom_frame_id': odom_frame_id,
                'base_frame_id': base_frame_id,
                'scan_topic': scan_topic,
                'update_min_d': 0.10,
                'update_min_a': 0.10,
                'transform_tolerance': 0.1,
                'min_particles': 500,
                'max_particles': 2000,
            }
        ],
    )

    # map_server/amcl を自動 activate する lifecycle manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': ['map_server', 'amcl']}],
    )

    # 必要に応じて RViz を起動
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    # base -> laser の静的TF（必要時）
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_static_tf',
        output='screen',
        arguments=[
            '--x',
            static_tf_x,
            '--y',
            static_tf_y,
            '--z',
            static_tf_z,
            '--yaw',
            static_tf_yaw,
            '--pitch',
            static_tf_pitch,
            '--roll',
            static_tf_roll,
            '--frame-id',
            base_frame_id,
            '--child-frame-id',
            laser_frame_id,
        ],
        condition=IfCondition(use_static_base_to_laser_tf),
    )

    # base -> base_link の互換用静的TF（必要時）
    base_link_alias_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_base_link_alias_tf',
        output='screen',
        arguments=['--frame-id', base_frame_id, '--child-frame-id', base_link_frame_id],
        condition=IfCondition(use_base_to_base_link_alias_tf),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument(
                'map',
                default_value=PathJoinSubstitution([FindPackageShare('onnx_nav_pkg'), 'map', 'my_map.yaml']),
                description='Map YAML file path for nav2_map_server',
            ),
            DeclareLaunchArgument('rviz', default_value='true'),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=PathJoinSubstitution([
                    FindPackageShare('onnx_nav_pkg'),
                    'rviz',
                    'real_nav_default.rviz',
                ]),
                description='RViz config file path',
            ),
            DeclareLaunchArgument('global_frame_id', default_value='map'),
            DeclareLaunchArgument('odom_frame_id', default_value='odom'),
            DeclareLaunchArgument('base_frame_id', default_value='base'),
            DeclareLaunchArgument('scan_topic', default_value='/scan'),
            DeclareLaunchArgument('use_static_base_to_laser_tf', default_value='true'),
            DeclareLaunchArgument('use_base_to_base_link_alias_tf', default_value='true'),
            DeclareLaunchArgument('laser_frame_id', default_value='laser'),
            DeclareLaunchArgument('base_link_frame_id', default_value='base_link'),
            DeclareLaunchArgument('static_tf_x', default_value='0.2'),
            DeclareLaunchArgument('static_tf_y', default_value='0.0'),
            DeclareLaunchArgument('static_tf_z', default_value='0.15'),
            DeclareLaunchArgument('static_tf_yaw', default_value='0.0'),
            DeclareLaunchArgument('static_tf_pitch', default_value='0.0'),
            DeclareLaunchArgument('static_tf_roll', default_value='0.0'),
            static_tf_node,
            base_link_alias_tf_node,
            map_server_node,
            amcl_node,
            lifecycle_manager_node,
            rviz_node,
        ]
    )
