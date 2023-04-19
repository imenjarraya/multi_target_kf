import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
   
    # Add argument for the path of the yaml file
    kf_yaml = LaunchConfiguration('kf_yaml')
    detections_topic = LaunchConfiguration('detections_topic')
    namespace = LaunchConfiguration('kf_ns')

    config = os.path.join(
        get_package_share_directory('multi_target_kf'),
        'config',
        'kf_param.yaml'
    )
    
    kf_yaml_launch_arg = DeclareLaunchArgument(
        'kf_yaml',
        default_value=config
    )

    detections_topic_launch_arg = DeclareLaunchArgument(
        'detections_topic',
        default_value='measurement/pose_array'
    )

    namespace_launch_arg = DeclareLaunchArgument(
        'kf_ns',
        default_value=''
    )

    # KF node
    kf_node = Node(
        package='multi_target_kf',
        executable='tracker_node',
        name='kf_tracker_node',
        namespace=namespace,
        output='screen',
        parameters=[kf_yaml],
        remappings=[('measurement/pose_array', detections_topic)]
    )

    ld = LaunchDescription()

    ld.add_action(kf_yaml_launch_arg)
    ld.add_action(detections_topic_launch_arg)
    ld.add_action(namespace_launch_arg)
    ld.add_action(kf_node)

    return ld