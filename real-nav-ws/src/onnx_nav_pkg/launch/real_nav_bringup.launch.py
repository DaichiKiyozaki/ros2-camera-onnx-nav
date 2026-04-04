from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # 機能ごとのON/OFFフラグ
    use_camera = LaunchConfiguration('use_camera')
    use_segmentation = LaunchConfiguration('use_segmentation')
    use_localization = LaunchConfiguration('use_localization')
    use_navigation = LaunchConfiguration('use_navigation')
    use_twist_converter = LaunchConfiguration('use_twist_converter')

    camera_topic = LaunchConfiguration('camera_topic')
    cb_topic = LaunchConfiguration('cb_topic')

    # 1) Camera: 実機カメラ入力
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        output='screen',
        condition=IfCondition(use_camera),
        remappings=[('/image_raw', camera_topic)],
        arguments=[
            '--ros-args',
            '-p',
            ['video_device:=', LaunchConfiguration('video_device')],
            '-p',
            ['image_size:=', LaunchConfiguration('image_size')],
            '-p',
            ['pixel_format:=', LaunchConfiguration('pixel_format')],
        ],
    )

    # 2) Segmentation: /image_raw -> /cb_img
    segmentation_node = Node(
        package='img_seg_pkg',
        executable='pedflow_4cls_seg_node',
        name='pedflow_4cls_seg_node',
        output='screen',
        condition=IfCondition(use_segmentation),
        parameters=[
            {
                'image_topic': camera_topic,
                'output_topic': cb_topic,
                'max_segmentation_hz': LaunchConfiguration('max_segmentation_hz'),
            }
        ],
    )

    # 3) Localization: map_server + amcl + lifecycle
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('onnx_nav_pkg'), 'launch', 'real_amcl.launch.py'])
        ),
        condition=IfCondition(use_localization),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'global_frame_id': LaunchConfiguration('global_frame_id'),
            'odom_frame_id': LaunchConfiguration('odom_frame_id'),
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'use_static_base_to_laser_tf': LaunchConfiguration('use_static_base_to_laser_tf'),
            'use_base_to_base_link_alias_tf': LaunchConfiguration('use_base_to_base_link_alias_tf'),
            'laser_frame_id': LaunchConfiguration('laser_frame_id'),
            'base_link_frame_id': LaunchConfiguration('base_link_frame_id'),
            'static_tf_x': LaunchConfiguration('static_tf_x'),
            'static_tf_y': LaunchConfiguration('static_tf_y'),
            'static_tf_z': LaunchConfiguration('static_tf_z'),
            'static_tf_yaw': LaunchConfiguration('static_tf_yaw'),
            'static_tf_pitch': LaunchConfiguration('static_tf_pitch'),
            'static_tf_roll': LaunchConfiguration('static_tf_roll'),
        }.items(),
    )

    # 4) ONNX navigation: /cb_img + /goal_pose + /amcl_pose -> /agent/cmd
    nav_node = Node(
        package='onnx_nav_pkg',
        executable='real_onnx_nav_node',
        name='real_onnx_nav_node',
        output='screen',
        condition=IfCondition(use_navigation),
        parameters=[
            {
                'model_file_name': LaunchConfiguration('model_file_name'),
                'image_topic': cb_topic,
                'goal_pose_topic': LaunchConfiguration('goal_pose_topic'),
                'amcl_pose_topic': LaunchConfiguration('amcl_pose_topic'),
                'action_topic': LaunchConfiguration('action_topic'),
                'stack_size': LaunchConfiguration('stack_size'),
                'max_inference_hz': LaunchConfiguration('max_inference_hz'),
                'write_model_io_file': LaunchConfiguration('write_model_io_file'),
                'model_io_log_dir': LaunchConfiguration('model_io_log_dir'),
                'model_io_log_every_n': LaunchConfiguration('model_io_log_every_n'),
            }
        ],
    )

    # 5) Adapter: /agent/cmd(Float32MultiArray) -> cmd_vel(Twist)
    twist_converter_node = Node(
        package='onnx_nav_pkg',
        executable='action_to_twist_node',
        name='action_to_twist_node',
        output='screen',
        condition=IfCondition(use_twist_converter),
        parameters=[
            {
                'action_topic': LaunchConfiguration('action_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'linear_scale': LaunchConfiguration('linear_scale'),
                'angular_scale': LaunchConfiguration('angular_scale'),
                'max_linear_x': LaunchConfiguration('max_linear_x'),
                'max_angular_z': LaunchConfiguration('max_angular_z'),
                'allow_backward': LaunchConfiguration('allow_backward'),
                'publish_rate_hz': LaunchConfiguration('cmd_vel_rate_hz'),
                'command_timeout_sec': LaunchConfiguration('command_timeout_sec'),
            }
        ],
    )

    return LaunchDescription(
        [
            # bringup全体の有効/無効
            DeclareLaunchArgument('use_camera', default_value='true'),
            DeclareLaunchArgument('use_segmentation', default_value='true'),
            DeclareLaunchArgument('use_localization', default_value='true'),
            DeclareLaunchArgument('use_navigation', default_value='true'),
            DeclareLaunchArgument('use_twist_converter', default_value='true'),
            DeclareLaunchArgument('video_device', default_value='/dev/video0'),
            DeclareLaunchArgument('image_size', default_value='[640,480]'),
            DeclareLaunchArgument('pixel_format', default_value='YUYV'),
            DeclareLaunchArgument('camera_topic', default_value='/image_raw'),
            DeclareLaunchArgument('cb_topic', default_value='/cb_img'),
            DeclareLaunchArgument('max_segmentation_hz', default_value='10.0'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument(
                'map',
                default_value=PathJoinSubstitution([FindPackageShare('onnx_nav_pkg'), 'map', 'my_map.yaml']),
            ),
            DeclareLaunchArgument('rviz', default_value='true'),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('onnx_nav_pkg'), 'rviz', 'real_nav_default.rviz']
                ),
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
            DeclareLaunchArgument('model_file_name', default_value='balance.onnx'),
            DeclareLaunchArgument('goal_pose_topic', default_value='/goal_pose'),
            DeclareLaunchArgument('amcl_pose_topic', default_value='/amcl_pose'),
            DeclareLaunchArgument('action_topic', default_value='/agent/cmd'),
            DeclareLaunchArgument('cmd_vel_topic', default_value='/commands/velocity'),
            DeclareLaunchArgument('linear_scale', default_value='0.4'),
            DeclareLaunchArgument('angular_scale', default_value='0.2'),
            DeclareLaunchArgument('max_linear_x', default_value='0.22'),
            DeclareLaunchArgument('max_angular_z', default_value='2.84'),
            DeclareLaunchArgument('allow_backward', default_value='false'),
            DeclareLaunchArgument('cmd_vel_rate_hz', default_value='20.0'),
            DeclareLaunchArgument('command_timeout_sec', default_value='0.5'),
            DeclareLaunchArgument('stack_size', default_value='5'),
            DeclareLaunchArgument('max_inference_hz', default_value='10.0'),
            DeclareLaunchArgument('write_model_io_file', default_value='true'),
            DeclareLaunchArgument('model_io_log_dir', default_value='logs/model_io'),
            DeclareLaunchArgument('model_io_log_every_n', default_value='1'),
            camera_node,
            segmentation_node,
            amcl_launch,
            nav_node,
            twist_converter_node,
        ]
    )
