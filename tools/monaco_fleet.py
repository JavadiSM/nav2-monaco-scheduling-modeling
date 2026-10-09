"""Build isolated additional vehicle assets from the frozen single-car baseline."""
from copy import deepcopy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

COLORS = {'blue': '0.03 0.25 0.95 1', 'white': '0.96 0.97 1.0 1', 'green': '0.05 0.80 0.15 1'}
VEHICLES = ('red', 'blue', 'white', 'green')


def route_pose(scene, offset):
    """Interpolate a start pose along the existing centreline, in metres."""
    if not math.isfinite(offset) or offset < 0:
        raise ValueError('Start offset must be finite and nonnegative')
    if offset == 0:
        return dict(scene['start'])
    remaining = scene['start']['s'] + offset
    for a, b in zip(scene['centreline'], scene['centreline'][1:]):
        length = math.dist(a, b)
        if length and remaining <= length:
            t = remaining / length
            return dict(x=a[0] + t*(b[0]-a[0]), y=a[1] + t*(b[1]-a[1]),
                        yaw=math.atan2(b[1]-a[1], b[0]-a[0]), s=scene['start']['s']+offset)
        remaining -= length
    raise ValueError('Start offset lies outside the route')


def namespace_topics(value, namespace):
    if isinstance(value, dict):
        return {k: namespace_topics(v, namespace) for k, v in value.items()}
    if isinstance(value, list):
        return [namespace_topics(v, namespace) for v in value]
    if isinstance(value, str) and value.startswith('/'):
        return '/'+namespace+value
    return value


def prepare_fleet(project):
    project = Path(project)
    scene_dir = project/'scenarios/monaco'
    scene = json.loads((scene_dir/'scenario.json').read_text())
    out = project/'artifacts/fleet/assets'
    out.mkdir(parents=True, exist_ok=True)
    from tools.monaco_start_grid import prepare_start_grid, grid_pose
    config = yaml.safe_load((project/'config/monaco_fleet.yaml').read_text())
    prepare_start_grid(project, scene, out, config['start_grid'])
    base_params = yaml.safe_load((scene_dir/'nav2_params.yaml').read_text())
    base_params['bt_navigator']['ros__parameters']['default_server_timeout'] = config['nav2']['action_ack_timeout_ms']
    (out/'red-nav2.yaml').write_text(yaml.safe_dump(base_params, sort_keys=False))
    fleet = []
    messages = [
        ('joint_states', 'sensor_msgs/msg/JointState', 'gz.msgs.Model'),
        ('odom', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry'),
        ('tf', 'tf2_msgs/msg/TFMessage', 'gz.msgs.Pose_V'),
        ('imu', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU'),
        ('scan', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan'),
        ('cmd_vel', 'geometry_msgs/msg/Twist', 'gz.msgs.Twist'),
    ]
    original = ET.parse(scene_dir/'racecar.sdf')
    for color, rgba in COLORS.items():
        model_name = 'racecar_'+color
        sdf = deepcopy(original)
        sdf.getroot().find('model').set('name', model_name)
        for visual in sdf.findall('.//visual'):
            if visual.get('name') in ('racing_body', 'nose'):
                for field in ('ambient', 'diffuse'):
                    visual.find('material/'+field).text = rgba
        for tag in ('topic', 'odom_topic', 'tf_topic'):
            for element in sdf.findall('.//'+tag):
                if element.text and element.text.startswith('/'):
                    element.text = '/'+color+element.text
        sdf_file = out/(color+'.sdf')
        sdf.write(sdf_file, encoding='utf-8', xml_declaration=True)
        urdf = ET.parse(scene_dir/'racecar.urdf')
        for material in urdf.findall('.//material'):
            if material.get('name') == 'racer_red':
                material.find('color').set('rgba', rgba)
        urdf_file = out/(color+'.urdf')
        urdf.write(urdf_file, encoding='utf-8', xml_declaration=True)
        params = namespace_topics(base_params, color)
        params_file = out/(color+'-nav2.yaml')
        params_file.write_text(yaml.safe_dump(params, sort_keys=False))
        bridge = []
        for topic, ros_type, gz_type in messages:
            bridge.append(dict(ros_topic_name=f'/{color}/{topic}', gz_topic_name=f'/{color}/{topic}',
                ros_type_name=ros_type, gz_type_name=gz_type,
                direction='ROS_TO_GZ' if topic == 'cmd_vel' else 'GZ_TO_ROS'))
        bridge_file = out/(color+'-bridge.yaml')
        bridge_file.write_text(yaml.safe_dump(bridge, sort_keys=False))
        fleet.append(dict(color=color, namespace=color, model_name=model_name, pose=grid_pose(scene, color, config['start_grid']), sdf=str(sdf_file), urdf=str(urdf_file),
            params=str(params_file), bridge=str(bridge_file)))
    (out/'fleet.json').write_text(json.dumps(fleet, indent=2)+'\n')
    return fleet
