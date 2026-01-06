#!/bin/bash
# Wrapper pour lancer GMapping depuis le workspace local
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PROJECT_ROOT="$DIR/../.."

source "$PROJECT_ROOT/deps/gmapping_ws/install/setup.bash"
exec ros2 run slam_gmapping slam_gmapping "$@"
