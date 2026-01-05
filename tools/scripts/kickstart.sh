#!/bin/bash
echo "--------------------------------------------------------"
echo "[KICKSTART] WAITING 5s before sending command..."
sleep 5
echo "[KICKSTART] SENDING ROTATION COMMAND (FORCE)..."
# On envoie la commande pendant 5 secondes en boucle
timeout 5s ros2 topic pub -t 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.8}}'
echo "[KICKSTART] STOPPING ROBOT..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo "[KICKSTART] DONE."
echo "--------------------------------------------------------"
