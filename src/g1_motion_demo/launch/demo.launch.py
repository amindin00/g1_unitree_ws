"""Launch the G1 motion playground in its own ROS namespace."""

from pathlib import Path
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = Path(get_package_share_directory('g1_unitree_description'))
    demo = Path(get_package_share_directory('g1_motion_demo'))
    urdf = description / 'urdf/g1_29dof.urdf'
    arguments = [
        DeclareLaunchArgument('mode', default_value='wave', choices=['wave', 'webcam', 'replay']),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('python_executable', default_value=sys.executable),
        DeclareLaunchArgument('rate', default_value='30.0'),
        DeclareLaunchArgument('speed', default_value='1.0'),
        DeclareLaunchArgument('loop', default_value='true'),
        DeclareLaunchArgument('smoothing', default_value='0.10'),
        DeclareLaunchArgument('max_joint_speed', default_value='2.5'),
        DeclareLaunchArgument('record_path', default_value=''),
        DeclareLaunchArgument('replay_path', default_value=''),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('camera', default_value='0'),
        DeclareLaunchArgument('video_path', default_value=''),
        DeclareLaunchArgument('preview', default_value='true'),
    ]
    types = {
        'mode': str, 'rate': float, 'speed': float, 'loop': bool, 'smoothing': float,
        'max_joint_speed': float, 'record_path': str, 'replay_path': str,
        'model_path': str, 'camera': int, 'video_path': str, 'preview': bool,
    }
    parameters = {name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                  for name, kind in types.items()}
    parameters['urdf_path'] = str(urdf)
    remappings = [('/tf', '/g1_demo/tf'), ('/tf_static', '/g1_demo/tf_static')]
    # Snap-hosted editors can export GTK libraries incompatible with host GUI apps.
    gui_environment = {key: '' for key in ('GTK_PATH', 'GIO_MODULE_DIR')
                       if '/snap/' in os.environ.get(key, '')}
    publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='g1_demo', parameters=[{'robot_description': urdf.read_text()}],
        remappings=remappings, output='screen',
    )
    motion = Node(
        package='g1_motion_demo', executable='motion_demo', namespace='g1_demo',
        prefix=[LaunchConfiguration('python_executable')],
        parameters=[parameters], additional_env=gui_environment, output='screen',
    )
    rviz = Node(
        package='rviz2', executable='rviz2', namespace='g1_demo',
        arguments=['-d', str(demo / 'rviz/demo.rviz')],
        remappings=remappings, condition=IfCondition(LaunchConfiguration('rviz')),
        additional_env=gui_environment,
        output='screen',
    )
    return LaunchDescription(arguments + [
        RegisterEventHandler(OnProcessExit(target_action=motion, on_exit=[
            EmitEvent(event=Shutdown(reason='Motion demo finished'))])),
        RegisterEventHandler(OnProcessExit(target_action=rviz, on_exit=[
            EmitEvent(event=Shutdown(reason='RViz closed'))])),
        publisher, motion, rviz,
    ])
