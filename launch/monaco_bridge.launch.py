"""One modeled vehicle coupled to real Nav2 on the configured physics lattice."""
import json,sys,xml.etree.ElementTree as ET
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess,GroupAction,IncludeLaunchDescription,SetEnvironmentVariable,DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node,SetParameter

def generate_launch_description():
    root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root))
    import yaml
    config=json.loads((root/'config/live_bridge.json').read_text());scene=root/'scenarios/monaco';scenario=json.loads((scene/'scenario.json').read_text());runtime=root/'artifacts/live-bridge/assets';runtime.mkdir(parents=True,exist_ok=True)
    tree=ET.parse(scene/'world.sdf');tree.find('.//physics/max_step_size').text=str(config['step_ns']/1e9);tree.write(runtime/'world.sdf',encoding='unicode')
    params=yaml.safe_load((scene/'nav2_params.yaml').read_text());params['bt_navigator']['ros__parameters']['default_server_timeout']=config['action_ack_timeout_ms']
    for name in ('lifecycle_manager_navigation','lifecycle_manager_localization'):params.setdefault(name,{}).setdefault('ros__parameters',{})['bond_timeout']=0.0
    params_file=runtime/'nav2.yaml';params_file.write_text(yaml.safe_dump(params,sort_keys=False))
    sdf=ET.parse(scene/'racecar.sdf');urdf=ET.parse(scene/'racecar.urdf');color=config['vehicle_color']
    if color!='red':
        from tools.monaco_fleet import COLORS
        if color not in COLORS:raise ValueError('Unknown visual vehicle color')
        for visual in sdf.findall('.//visual'):
            if visual.get('name') in ('racing_body','nose'):
                for field in ('ambient','diffuse'):visual.find('material/'+field).text=COLORS[color]
        for material in urdf.findall('.//material'):
            if material.get('name')=='racer_red':material.find('color').set('rgba',COLORS[color])
    urdf_file=runtime/'racecar.urdf';urdf.write(urdf_file,encoding='unicode')
    sdf_file=runtime/'racecar.sdf';sdf.write(sdf_file,encoding='unicode')
    p=scenario['start'];nav2=Path(get_package_share_directory('nav2_bringup'));tb3=Path(get_package_share_directory('nav2_minimal_tb3_sim'))
    return LaunchDescription([DeclareLaunchArgument('gui',default_value='false'),DeclareLaunchArgument('socket'),DeclareLaunchArgument('clock'),
        ExecuteProcess(cmd=['gz','sim','-s','-v','2',str(runtime/'world.sdf')],output='screen'),
        ExecuteProcess(cmd=['gz','sim','-g','-v','2','--gui-config',str(root/'config/monaco-gui.config')],condition=IfCondition(LaunchConfiguration('gui')),output='screen'),
        Node(package='ros_gz_sim',executable='create',output='screen',arguments=['-world','monaco','-name','racecar','-file',str(sdf_file),'-x',str(p['x']),'-y',str(p['y']),'-z','0.01','-Y',str(p['yaw'])]),
        Node(package='ros_gz_bridge',executable='parameter_bridge',output='screen',parameters=[{'config_file':str(tb3/'configs/turtlebot3_waffle_bridge.yaml'),'use_sim_time':True}]),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':urdf_file.read_text(),'use_sim_time':True}],output='screen'),
        GroupAction(actions=[SetParameter(name='bond_timeout',value=0.0),SetEnvironmentVariable('LD_PRELOAD',str(root/'build/live-bridge/libnav2_live_bridge.so')),SetEnvironmentVariable('NAV2_LIVE_DEVICE','0'),SetEnvironmentVariable('NAV2_LIVE_SOCKET',LaunchConfiguration('socket')),SetEnvironmentVariable('NAV2_LIVE_CLOCK',LaunchConfiguration('clock')),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav2/'launch/bringup_launch.py')),launch_arguments={'namespace':'','use_namespace':'False','map':str(scene/'map.yaml'),'params_file':str(params_file),'use_sim_time':'True','autostart':'True','slam':'False','use_composition':'True'}.items())])])
