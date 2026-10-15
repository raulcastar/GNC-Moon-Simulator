ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
ros2 topic pub --once /headlights_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /headlights_intensity std_msgs/msg/Float32 "{data: 0.3}"
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 300, y: 350, z: 0.0}}}"
cat /etc/resolv.conf | grep nameserver | awk '{print $2}'
docker logs -f --tail 50 {container_id}
New-NetFirewallRule -DisplayName "Allow WSL" -Direction Inbound -InterfaceAlias "vEthernet (WSL)" -Action Allow