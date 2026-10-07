#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/ros_env.sh"
setup_ros_workspace
exec /usr/bin/python3 scripts/set_gain_alpha.py --gazebo "$@"
