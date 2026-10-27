#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /home/pi/USTROV/ustrov_underlay/install/setup.bash
source /home/pi/USTROV/ustrov_ws/install/setup.bash
source /home/pi/ustrov-network/dds-wired-env.sh
set -o pipefail
ros2 launch ustrov_status_display status_display.launch.py 2>&1 | tee /home/pi/ustrov-network/display-terminal.log
