#!/usr/bin/env python3
"""Generate a hand-designed open Monaco-like corridor, map and car assets."""
import heapq
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, label
import yaml


def text(parent, name, value):
    elem = ET.SubElement(parent, name)
    elem.text = str(value)
    return elem


def box(link, name, pose, size, colour, collision=False):
    visual = ET.SubElement(link, 'visual', name=name)
    text(visual, 'pose', ' '.join(map(str, pose)))
    geometry = ET.SubElement(visual, 'geometry')
    text(ET.SubElement(geometry, 'box'), 'size', ' '.join(map(str, size)))
    material = ET.SubElement(visual, 'material')
    text(material, 'ambient', colour)
    text(material, 'diffuse', colour)
    if collision:
        body = ET.SubElement(link, 'collision', name=name + '_collision')
        text(body, 'pose', ' '.join(map(str, pose)))
        body.append(ET.fromstring(ET.tostring(geometry)))


def cylinder(link, name, pose, radius, length, colour):
    visual = ET.SubElement(link, 'visual', name=name)
    text(visual, 'pose', ' '.join(map(str, pose)))
    geom = ET.SubElement(ET.SubElement(visual, 'geometry'), 'cylinder')
    text(geom, 'radius', radius)
    text(geom, 'length', length)
    mat = ET.SubElement(visual, 'material')
    text(mat, 'ambient', colour)
    text(mat, 'diffuse', colour)


def static_model(world, name, x=0, y=0, yaw=0):
    model = ET.SubElement(world, 'model', name=name)
    text(model, 'static', 'true')
    text(model, 'pose', f'{x} {y} 0 0 0 {yaw}')
    return ET.SubElement(model, 'link', name='body')


def rounded(points, iterations=4):
    """Chaikin corner cutting preserves straight sections and rounds turns."""
    curve = np.array(points, dtype=float)
    for _ in range(iterations):
        q = 0.75 * curve[:-1] + 0.25 * curve[1:]
        r = 0.25 * curve[:-1] + 0.75 * curve[1:]
        pairs = np.empty((2 * len(q), 2))
        pairs[0::2], pairs[1::2] = q, r
        curve = np.vstack([points[0], pairs, points[-1]])
    sampled = [curve[0]]
    for a, b in zip(curve, curve[1:]):
        steps = max(1, int(math.ceil(np.linalg.norm(b-a) / 0.12)))
        sampled.extend(a+(b-a)*t for t in np.linspace(0,1,steps+1)[1:])
    return np.array(sampled)


def pose_at(route, distances, at):
    at = float(np.clip(at, 0, distances[-1] - 1e-5))
    i = max(0, min(len(route) - 2, np.searchsorted(distances, at, side='right') - 1))
    t = (at - distances[i]) / max(distances[i + 1] - distances[i], 1e-9)
    p = route[i] * (1 - t) + route[i + 1] * t
    delta = route[i + 1] - route[i]
    return {'x': float(p[0]), 'y': float(p[1]), 'yaw': math.atan2(delta[1], delta[0]), 's': at}


def shortest_grid_path(free, start, end, resolution):
    best = {start: 0.0}
    queue = [(0.0, start)]
    steps = [(a, b, math.hypot(a, b)) for a in [-1, 0, 1] for b in [-1, 0, 1] if a or b]
    while queue:
        distance, p = heapq.heappop(queue)
        if distance > best[p]:
            continue
        if p == end:
            return distance * resolution
        for dy, dx, cost in steps:
            q = (p[0] + dy, p[1] + dx)
            if not (0 <= q[0] < free.shape[0] and 0 <= q[1] < free.shape[1] and free[q]):
                continue
            candidate = distance + cost
            if candidate < best.get(q, math.inf):
                best[q] = candidate
                heapq.heappush(queue, (candidate, q))
    raise RuntimeError('Track is disconnected for the robot footprint.')


def main():
    project = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((project / 'config/monaco.yaml').read_text())
    output = project / 'scenarios/monaco'
    output.mkdir(parents=True, exist_ok=True)
    origin = np.array(config['pixel_origin'], dtype=float)
    scale = config['pixel_scale']
    def world_point(pixel):
        return (np.array(pixel, dtype=float) - origin) * [scale, -scale]
    points = np.array([world_point(p) for p in config['shape']])
    route = rounded(points, int(config.get('curve_smoothing_iterations', 4)))
    distances = np.r_[0, np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
    res, width = config['map_resolution'], config['road_width']
    minimum = np.floor((route.min(axis=0) - 3) / res) * res
    maximum = np.ceil((route.max(axis=0) + 3) / res) * res
    cols, rows = np.ceil((maximum - minimum) / res).astype(int)
    def pixel(p):
        return np.array([(p[0] - minimum[0]) / res, rows - 1 - (p[1] - minimum[1]) / res], dtype=int)
    mask = np.zeros((rows, cols), dtype=np.uint8)
    cv2.polylines(mask, [np.array([pixel(p) for p in route])], False, 255, thickness=round(width / res))
    for p in [route[0], route[-1]]:
        cv2.circle(mask, tuple(pixel(p)), round(width / (2 * res)), 255, -1)
    clearance = distance_transform_edt(mask > 0) * res
    drivable = clearance > 0.255
    a, b = tuple(pixel(route[0])[::-1]), tuple(pixel(route[-1])[::-1])
    shortest = shortest_grid_path(drivable, a, b, res)
    euclidean = float(np.linalg.norm(route[-1] - route[0]))
    if shortest < 0.78 * distances[-1] or shortest < 10 * euclidean:
        raise RuntimeError(f'Unintended shortcut: shortest={shortest:.2f}, centreline={distances[-1]:.2f}, direct={euclidean:.2f}')
    checkpoints, servers = [], []
    previous_s = -1
    for corner in config['corners']:
        centre = world_point(corner['point'])
        eligible = np.flatnonzero(distances > previous_s + 0.3)
        nearest = eligible[np.argmin(np.linalg.norm(route[eligible] - centre, axis=1))]
        s = distances[nearest]
        lead = float(corner.get('checkpoint_lead_m', 0.55))
        cp = pose_at(route, distances, max(0.5, s - lead))
        cp.update({'id': corner['id'], 'name': corner['name'], 'lead_m': lead})
        checkpoints.append(cp)
        previous_s = s
        p = np.array([cp['x'], cp['y']])
        normal = np.array([-math.sin(cp['yaw']), math.cos(cp['yaw'])])
        parked = None
        tangent = np.array([math.cos(cp['yaw']), math.sin(cp['yaw'])])
        for offset, along in [(off, along) for off in [1.25, 1.8, 2.5, 3.2, 4.0] for along in [0, -0.75, 0.75, -1.5, 1.5]]:
            for side in [1, -1]:
                candidate = p + side * offset * normal + along * tangent
                if np.min(np.linalg.norm(route - candidate, axis=1)) < width / 2 + 0.45:
                    continue
                if any(np.linalg.norm(candidate - [other['x'], other['y']]) < 1.0 for other in servers):
                    continue
                parked = candidate
                break
            if parked is not None:
                break
        if parked is None:
            raise RuntimeError(f'No clear parking bay for edge server {corner["id"]}')
        servers.append({'id': f'edge_{corner["id"]:02d}', 'checkpoint_id': corner['id'], 'x': float(parked[0]), 'y': float(parked[1]), 'yaw': cp['yaw'], 'static': True, 'worker_enabled': False})
    scenario = {'name': config['name'], 'road_width_m': width, 'centreline_length_m': float(distances[-1]), 'shortest_drivable_path_m': shortest, 'start_finish_direct_distance_m': euclidean, 'start': pose_at(route, distances, 0.05), 'finish': pose_at(route, distances, distances[-1] - 0.08), 'checkpoints': checkpoints, 'servers': servers, 'centreline': route.tolist(), 'communication_enabled': False}
    (output / 'scenario.json').write_text(json.dumps(scenario, indent=2) + '\n')
    # AMCL likelihood fields need physical wall surfaces, not solid-filled
    # outside space: filling it black gives off-road particles perfect scores.
    occupancy = np.where(mask > 0, 254, 205).astype(np.uint8)
    wall_contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(occupancy, wall_contours, -1, 0, thickness=2)
    Image.fromarray(occupancy).save(output / 'map.pgm')
    (output / 'map.yaml').write_text(yaml.safe_dump({'image': 'map.pgm', 'mode': 'trinary', 'resolution': res, 'origin': [float(minimum[0]), float(minimum[1]), 0.0], 'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}, sort_keys=False))

    root = ET.Element('sdf', version='1.9')
    world = ET.SubElement(root, 'world', name='monaco')
    for plugin_name, class_name in [('physics', 'Physics'), ('user-commands', 'UserCommands'), ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors'), ('imu', 'Imu')]:
        plugin = ET.SubElement(world, 'plugin', filename=f'gz-sim-{plugin_name}-system', name=f'gz::sim::systems::{class_name}')
        if class_name == 'Sensors':
            text(plugin, 'render_engine', 'ogre2')
    physics = ET.SubElement(world, 'physics', name='default', type='ode')
    text(physics, 'max_step_size', 0.003)
    text(physics, 'real_time_factor', 1)
    scene = ET.SubElement(world, 'scene')
    text(scene, 'ambient', '0.7 0.7 0.7 1')
    text(scene, 'background', '0.65 0.78 0.88 1')
    text(scene, 'shadows', 'false')
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    text(light, 'pose', '0 0 20 0 0 0')
    text(light, 'diffuse', '0.9 0.9 0.9 1')
    text(light, 'direction', '-0.5 0.3 -1')
    ground = static_model(world, 'ground')
    box(ground, 'ground', [0, 0, -0.08, 0, 0, 0], [100, 100, 0.15], '0.20 0.34 0.30 1', True)
    water = static_model(world, 'harbour', 4, -7)
    box(water, 'water', [0, 0, -0.002, 0, 0, 0], [29, 7, 0.004], '0.08 0.35 0.55 1')
    road = static_model(world, 'asphalt')
    visual_route = cv2.approxPolyDP(route.astype(np.float32), 0.025, False).reshape(-1, 2)
    for i, vertex in enumerate(visual_route):
        cylinder(road, f'road_join_{i}', [*vertex, 0.003, 0, 0, 0], width / 2, 0.008, '0.12 0.14 0.17 1')
    for i in range(len(visual_route) - 1):
        a0, b0 = visual_route[i], visual_route[i + 1]
        delta = b0 - a0
        midpoint = (a0 + b0) / 2
        box(road, f'road_{i}', [*midpoint, 0.003, 0, 0, math.atan2(delta[1], delta[0])], [float(np.linalg.norm(delta)) + 0.025, width, 0.008], '0.12 0.14 0.17 1')
    barriers = static_model(world, 'continuous_track_barriers')
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    wall_count = 0
    for contour in contours:
        vertices = cv2.approxPolyDP(contour, 0.8, True).reshape(-1, 2)
        vertices = np.array([[minimum[0] + v[0] * res, minimum[1] + (rows - 1 - v[1]) * res] for v in vertices])
        for p, q in zip(vertices, np.roll(vertices, -1, axis=0)):
            delta = q - p
            length = np.linalg.norm(delta)
            if length < 0.03:
                continue
            middle = (p + q) / 2
            colour = '0.92 0.93 0.94 1' if wall_count % 2 else '0.80 0.12 0.16 1'
            box(barriers, f'wall_{wall_count}', [*middle, config['barrier_height'] / 2, 0, 0, math.atan2(delta[1], delta[0])], [float(length) + 0.03, 0.055, config['barrier_height']], colour, True)
            wall_count += 1
    dashes = static_model(world, 'centreline_dashes')
    for i, s in enumerate(np.arange(0.8, distances[-1] - 0.5, 1.2)):
        p = pose_at(route, distances, s)
        box(dashes, f'dash_{i}', [p['x'], p['y'], 0.010, 0, 0, p['yaw']], [0.32, 0.025, 0.003], '0.82 0.83 0.84 1')
    for cp in checkpoints:
        checkpoint = static_model(world, f'checkpoint_{cp["id"]:02d}', cp['x'], cp['y'], cp['yaw'])
        box(checkpoint, 'stripe', [0, 0, 0.016, 0, 0, 0], [0.08, width - 0.13, 0.008], '1 0.78 0.15 1')
        for side in [-1, 1]:
            box(checkpoint, f'post_{side}', [0, side * (width / 2 + 0.08), 0.38, 0, 0, 0], [0.06, 0.06, 0.76], '1 0.72 0.06 1')
        box(checkpoint, 'overhead', [0, 0, 0.79, 0, 0, 0], [0.06, width + 0.22, 0.06], '1 0.72 0.06 1')
    for name, pose, colour in [('START', scenario['start'], '0.2 0.95 0.48 1'), ('FINISH', scenario['finish'], '1 0.27 0.31 1')]:
        gate = static_model(world, name.lower(), pose['x'], pose['y'], pose['yaw'])
        box(gate, 'stripe', [0, 0, 0.018, 0, 0, 0], [0.18, width - 0.1, 0.012], colour)
        for side in [-1, 1]:
            box(gate, f'post_{side}', [0, side * (width / 2 + 0.12), 0.7, 0, 0, 0], [0.10, 0.10, 1.4], colour)
        box(gate, 'header', [0, 0, 1.42, 0, 0, 0], [0.13, width + 0.35, 0.14], colour)
    for server in servers:
        car = static_model(world, server['id'], server['x'], server['y'], server['yaw'])
        box(car, 'parking_bay', [0, 0, 0.012, 0, 0, 0], [0.86, 0.60, 0.025], '0.1 0.65 0.74 1')
        box(car, 'chassis', [0, 0, 0.14, 0, 0, 0], [0.64, 0.32, 0.13], '0.05 0.35 0.9 1', True)
        box(car, 'cabin', [-0.04, 0, 0.25, 0, 0, 0], [0.26, 0.28, 0.14], '0.11 0.17 0.22 1')
        box(car, 'compute_module', [-0.23, 0, 0.23, 0, 0, 0], [0.12, 0.23, 0.08], '0.1 0.82 0.88 1')
        for front in [-1, 1]:
            for side in [-1, 1]:
                cylinder(car, f'wheel_{front}_{side}', [front * 0.20, side * 0.18, 0.10, math.pi / 2, 0, 0], 0.09, 0.065, '0.025 0.03 0.04 1')
        cylinder(car, 'antenna', [-0.23, 0, 0.48, 0, 0, 0], 0.013, 0.44, '0.12 0.93 0.95 1')
    ET.indent(root)
    ET.ElementTree(root).write(output / 'world.sdf', encoding='unicode', xml_declaration=True)

    from ament_index_python.packages import get_package_share_directory
    import xacro
    upstream = Path(get_package_share_directory('nav2_minimal_tb3_sim'))
    sdf = ET.fromstring(xacro.process_file(str(upstream / 'urdf/gz_waffle.sdf.xacro'), mappings={'namespace': ''}).toxml())
    sdf.insert(0, ET.Comment('Derived from nav2_minimal_tb3_sim/gz_waffle.sdf.xacro (Apache-2.0). Modified visual geometry only; see NOTICE.md and LICENSE.upstream.'))
    for link in sdf.findall('.//link'):
        for visual in list(link.findall('visual')):
            link.remove(visual)
    base = sdf.find('.//link[@name="base_link"]')
    box(base, 'racing_body', [-0.025, 0, 0.085, 0, 0, 0], [0.32, 0.19, 0.07], '0.92 0.07 0.13 1')
    box(base, 'nose', [0.14, 0, 0.075, 0, 0, 0], [0.15, 0.07, 0.04], '0.95 0.08 0.16 1')
    box(base, 'front_wing', [0.20, 0, 0.055, 0, 0, 0], [0.035, 0.27, 0.025], '0.06 0.07 0.09 1')
    box(base, 'rear_wing', [-0.20, 0, 0.11, 0, 0, 0], [0.045, 0.27, 0.02], '0.06 0.07 0.09 1')
    box(base, 'cockpit', [-0.035, 0, 0.107, 0, 0, 0], [0.095, 0.085, 0.04], '0.1 0.14 0.18 1')
    for fore in [-0.13, 0.10]:
        for side in [-1, 1]:
            cylinder(base, f'tyre_{fore}_{side}', [fore, side * 0.13, 0.035, math.pi / 2, 0, 0], 0.035, 0.045, '0.02 0.02 0.025 1')
    cylinder(sdf.find('.//link[@name="base_scan"]'), 'lidar', [-0.064, 0, 0.137, 0, 0, 0], 0.035, 0.022, '0.12 0.16 0.20 1')
    ET.indent(sdf)
    ET.ElementTree(sdf).write(output / 'racecar.sdf', encoding='unicode', xml_declaration=True)
    robot = ET.parse(upstream / 'urdf/turtlebot3_waffle.urdf').getroot()
    robot.insert(0, ET.Comment('Derived from nav2_minimal_tb3_sim/turtlebot3_waffle.urdf (Apache-2.0). Modified visual geometry only; see NOTICE.md and LICENSE.upstream.'))
    for link in robot.findall('link'):
        for visual in list(link.findall('visual')):
            link.remove(visual)
    urdf_base = robot.find('link[@name="base_link"]')
    visual = ET.SubElement(urdf_base, 'visual')
    ET.SubElement(visual, 'origin', xyz='-0.025 0 0.085', rpy='0 0 0')
    ET.SubElement(ET.SubElement(visual, 'geometry'), 'box', size='0.32 0.19 0.07')
    ET.SubElement(ET.SubElement(visual, 'material', name='racer_red'), 'color', rgba='0.92 0.07 0.13 1')
    ET.indent(robot)
    ET.ElementTree(robot).write(output / 'racecar.urdf', encoding='unicode', xml_declaration=True)
    params = yaml.safe_load((Path(get_package_share_directory('nav2_bringup')) / 'params/nav2_params.yaml').read_text())
    for name in ['local_costmap', 'global_costmap']:
        inflation = params[name][name]['ros__parameters']['inflation_layer']
        inflation['inflation_radius'] = 0.34
        inflation['cost_scaling_factor'] = 5.0
    localization = config['localization']
    amcl = params['amcl']['ros__parameters']
    for key in ['alpha1', 'alpha2', 'alpha3', 'alpha4', 'alpha5']:
        amcl[key] = localization['odometry_alpha']
    for key in ['max_beams', 'sigma_hit', 'update_min_d', 'update_min_a', 'z_hit', 'z_rand']:
        amcl[key] = localization[key]
    # Keep all upstream navigation / recovery nodes; only tune the mission rate.
    bt_source = Path(get_package_share_directory('nav2_bt_navigator')) / 'behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml'
    bt = ET.parse(bt_source)
    bt.getroot().insert(0, ET.Comment('Derived from nav2_bt_navigator NavigateThroughPoses BT (Apache-2.0); scenario replanning rate only.'))
    for rate in bt.findall('.//RateController'):
        rate.set('hz', str(config['mission_replan_hz']))
    ET.indent(bt)
    bt.write(output / 'navigate_through_poses.xml', encoding='unicode', xml_declaration=True)
    params['waypoint_follower']['ros__parameters']['stop_on_failure'] = True
    params['waypoint_follower']['ros__parameters']['wait_at_waypoint']['enabled'] = False
    params['planner_server']['ros__parameters']['GridBased']['allow_unknown'] = False
    (output / 'nav2_params.yaml').write_text('# Derived from nav2_bringup/params/nav2_params.yaml (Apache-2.0).\n# Scenario configuration changes are documented in docs/monaco-scenario.md.\n' + yaml.safe_dump(params, sort_keys=False))
    fig, ax = plt.subplots(figsize=(15, 7), facecolor='#edf2f6')
    ax.set_facecolor('#edf2f6')
    ax.plot(route[:, 0], route[:, 1], color='#172536', linewidth=20, solid_capstyle='round', zorder=2)
    ax.plot(route[:, 0], route[:, 1], color='#64778b', linewidth=14, solid_capstyle='round', zorder=3)
    ax.plot(route[:, 0], route[:, 1], color='#dbe5ed', linewidth=0.8, linestyle='--', zorder=4)
    for cp, server in zip(checkpoints, servers):
        ax.scatter(cp['x'], cp['y'], s=65, color='#f6bd3b', edgecolor='#172536', zorder=6)
        ax.text(cp['x'], cp['y'], str(cp['id']), fontsize=7, ha='center', va='center', zorder=7)
        ax.scatter(server['x'], server['y'], s=45, marker='s', color='#008dc0', zorder=5)
        ax.annotate(server['id'].replace('edge_', 'E'), (server['x'], server['y']), xytext=(4, 4), textcoords='offset points', fontsize=7, color='#005876')
    for name, colour, pose in [('START', '#16a068', scenario['start']), ('FINISH', '#db3b53', scenario['finish'])]:
        ax.scatter(pose['x'], pose['y'], s=100, color=colour, edgecolor='white', zorder=8)
        ax.annotate(name, (pose['x'], pose['y']), xytext=(-75, 15 if name == 'START' else -25), textcoords='offset points', fontsize=10, fontweight='bold', color=colour, arrowprops={'arrowstyle': '-', 'color': colour})
    ax.set_aspect('equal')
    ax.set_xlim(minimum[0], maximum[0])
    ax.set_ylim(minimum[1], maximum[1])
    ax.set_title('MONACO-INSPIRED OPEN CIRCUIT\n1 moving racer · 19 checkpoints · 19 parked edge-server cars', loc='left', fontsize=17, fontweight='bold', pad=20)
    ax.set_xlabel('Metres')
    ax.set_ylabel('Metres')
    ax.grid(alpha=0.15)
    fig.text(0.13, 0.035, f'Open-ended route: {distances[-1]:.1f} m | shortest drivable map path: {shortest:.1f} m | direct start–finish gap: {euclidean:.1f} m\nBlue cars are static visual edge locations. Communication and offloading are not enabled.', fontsize=10, color='#425268')
    fig.tight_layout(rect=[0, 0.08, 1, 1])
    fig.savefig(output / 'layout.png', dpi=150)
    plt.close(fig)
    print(json.dumps({'route_metres': float(distances[-1]), 'shortest_drivable_metres': shortest, 'direct_gap_metres': euclidean, 'checkpoints': len(checkpoints), 'static_servers': len(servers), 'barrier_segments': wall_count}, indent=2))


if __name__ == '__main__':
    if (Path(__file__).resolve().parents[1] / 'docs/evidence/frozen-scene-manifest.json').exists():
        raise SystemExit('The accepted scene is frozen. Use the existing assets; do not regenerate the circuit.')
    main()
