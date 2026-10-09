"""Frozen Monaco world with one red vehicle, or an optional four-car fleet."""
import json
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    project = Path(__file__).resolve().parents[1]
    scene = project / 'scenarios/monaco'
    scenario = json.loads((scene / 'scenario.json').read_text())
    pose = scenario['start']
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    tb3 = Path(get_package_share_directory('nav2_minimal_tb3_sim'))
    gui = LaunchConfiguration('gui')
    fleet_enabled = LaunchConfiguration('fleet')
    sys.path.insert(0, str(project))
    from tools.monaco_fleet import prepare_fleet
    fleet = prepare_fleet(project)
    assets = project/'artifacts/fleet/assets'
    import yaml
    from tools.monaco_start_grid import grid_pose
    grid = yaml.safe_load((project/'config/monaco_fleet.yaml').read_text())['start_grid']
    red_pose = grid_pose(scenario, 'red', grid)
    def select(fleet_value, baseline_value):
        return PythonExpression([repr(str(fleet_value)), " if '", fleet_enabled,
                                 "'.lower() in ('true','1') else ", repr(str(baseline_value))])
    world_file = select(assets/'world.sdf', scene/'world.sdf')
    map_file = select(assets/'map.yaml', scene/'map.yaml')
    root_params = select(assets/'red-nav2.yaml', scene/'nav2_params.yaml')
    fleet_actions = []
    for car in fleet:
        ns, p = car['namespace'], car['pose']
        fleet_actions.append(GroupAction(condition=IfCondition(fleet_enabled), actions=[
            Node(package='ros_gz_sim', executable='create', namespace=ns, output='screen',
                 arguments=['-world', 'monaco', '-name', car['model_name'], '-file', car['sdf'],
                            '-x', str(p['x']), '-y', str(p['y']), '-z', '0.01', '-Y', str(p['yaw'])]),
            Node(package='ros_gz_bridge', executable='parameter_bridge', namespace=ns, output='screen',
                 parameters=[{'config_file': car['bridge'], 'use_sim_time': True}]),
            Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=ns,
                 parameters=[{'robot_description': Path(car['urdf']).read_text(), 'use_sim_time': True}],
                 remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')], output='screen'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav2/'launch/bringup_launch.py')),
                launch_arguments={'namespace': ns, 'use_namespace': 'True', 'map': map_file,
                    'params_file': car['params'], 'use_sim_time': 'True', 'autostart': 'True',
                    'slam': 'False', 'use_composition': 'True'}.items()),
        ]))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('fleet', default_value='false'),
        DeclareLaunchArgument('namespace', default_value=''),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', '-v', '2', world_file], output='screen'),
        ExecuteProcess(cmd=['gz', 'sim', '-g', '-v', '2', '--gui-config', str(project / 'config/monaco-gui.config')], condition=IfCondition(gui), output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(tb3 / 'launch/spawn_tb3.launch.py')),
            launch_arguments={'namespace': '', 'robot_name': 'racecar', 'robot_sdf': str(scene / 'racecar.sdf'), 'x_pose': select(red_pose['x'], pose['x']), 'y_pose': select(red_pose['y'], pose['y']), 'z_pose': '0.01', 'yaw': select(red_pose['yaw'], pose['yaw'])}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[{'robot_description': (scene / 'racecar.urdf').read_text(), 'use_sim_time': True}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2 / 'launch/bringup_launch.py')),
            launch_arguments={'namespace': '', 'use_namespace': 'False', 'map': map_file, 'params_file': root_params, 'use_sim_time': 'True', 'autostart': 'True', 'slam': 'False', 'use_composition': 'True'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2 / 'launch/rviz_launch.py')),
            launch_arguments={'namespace': '', 'use_sim_time': 'true', 'rviz_config': str(project / 'artifacts/monaco.rviz'), 'use_namespace': 'false'}.items(),
            condition=IfCondition(gui)),
    ] + fleet_actions)
