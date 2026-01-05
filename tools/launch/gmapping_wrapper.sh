#!/bin/bash
# Wrapper pour lancer GMapping depuis le workspace local
source /home/schneigu/Projects/slam_bench_orchestrator/deps/gmapping_ws/install/setup.bash
exec ros2 run slam_gmapping slam_gmapping "$@"
