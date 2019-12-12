import launch
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    print(get_package_share_directory('openvino_node')+'/config/object_detection.yaml')
    container = ComposableNodeContainer(
            node_name='vision_pipeline',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='realsense_ros',
                    node_plugin='realsense::RealSenseNodeFactory',
                    node_name='realsense',
                    parameters=[get_package_share_directory('realsense_examples')+'/config/d435i.yaml'],
                    extra_arguments=[{'use_intra_process_comms':'true'}]),
                ComposableNode(
                    package='openvino_ros',
                    node_plugin='openvino::OpenVINOFactory',
                    node_name='object_detection',
                    remappings=[('/rdk/openvino/detected_objects', '/openvino/detected_objects'), ('/rdk/openvino/image_raw', '/camera/color/image_raw')],
                    parameters=[get_package_share_directory('openvino_node')+'/config/object_detection.yaml'],
                    extra_arguments=[{'use_intra_process_comms':'true'}])
            ],
            output='screen',
    )

    visualization = Node(
            package='openvino_utils', node_executable='openvino_utils',
            remappings=[('/rdk/openvino/detected_objects', '/openvino/detected_objects'), ('/rdk/openvino/image_raw', '/camera/color/image_raw')],
            parameters=[{'infer_type':'SSD'}],
            output='screen'
    )

    default_rviz = os.path.join(get_package_share_directory('openvino_node'), 'launch', 'rviz/object_detection.rviz')
    rviz = Node(
            package='rviz2', node_executable='rviz2', output='screen',
            arguments=['--display-config', default_rviz]
    )

    return launch.LaunchDescription([container, visualization, rviz])
