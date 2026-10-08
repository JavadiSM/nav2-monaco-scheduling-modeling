#!/usr/bin/env python3
"""Center the installed upstream RViz view without altering Nav2 algorithms."""
import argparse
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output')
    args = parser.parse_args()
    source = Path(get_package_share_directory('nav2_bringup')) / 'rviz/nav2_default_view.rviz'
    config = yaml.safe_load(source.read_text())
    view = config['Visualization Manager']['Views']['Current']
    view.update({'X': 0.0, 'Y': 0.0, 'Angle': 0.0, 'Scale': 95.0})
    for display in config['Visualization Manager']['Displays']:
        if display.get('Class') == 'rviz_default_plugins/RobotModel':
            display['Enabled'] = True
            display['Value'] = True
            display['Description Topic']['Durability Policy'] = 'Transient Local'
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(config, sort_keys=False))


if __name__ == '__main__':
    main()
