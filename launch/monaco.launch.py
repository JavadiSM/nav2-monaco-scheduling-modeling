"""One moving car with upstream Nav2; all edge-car models are static."""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project = Path(__file__).resolve().parents[1]
    scene = project / 'scenarios/monaco'
    scenario = json.loads((scene / 'scenario.json').read_text())
    pose = scenario['start']
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    tb3 = Path(get_package_share_directory('nav2_minimal_tb3_sim'))
    gui = LaunchConfiguration('gui')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('namespace', default_value=''),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', '-v', '2', str(scene / 'world.sdf')], output='screen'),
        ExecuteProcess(cmd=['gz', 'sim', '-g', '-v', '2', '--gui-config', str(project / 'config/monaco-gui.config')], condition=IfCondition(gui), output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(tb3 / 'launch/spawn_tb3.launch.py')),
            launch_arguments={'namespace': '', 'robot_name': 'racecar', 'robot_sdf': str(scene / 'racecar.sdf'), 'x_pose': str(pose['x']), 'y_pose': str(pose['y']), 'z_pose': '0.01', 'yaw': str(pose['yaw'])}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[{'robot_description': (scene / 'racecar.urdf').read_text(), 'use_sim_time': True}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2 / 'launch/bringup_launch.py')),
            launch_arguments={'namespace': '', 'use_namespace': 'False', 'map': str(scene / 'map.yaml'), 'params_file': str(scene / 'nav2_params.yaml'), 'use_sim_time': 'True', 'autostart': 'True', 'slam': 'False', 'use_composition': 'True'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2 / 'launch/rviz_launch.py')),
            launch_arguments={'namespace': '', 'use_sim_time': 'true', 'rviz_config': str(project / 'artifacts/monaco.rviz'), 'use_namespace': 'false'}.items(),
            condition=IfCondition(gui)),
    ])
