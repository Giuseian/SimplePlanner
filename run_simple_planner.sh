#!/bin/bash
set -e

WS_DIR=~/Documents/GITHUB_PROJECTS/SimplePlanner
PARAM_FILE=$WS_DIR/config/params.yaml
MAP_FILE=$WS_DIR/maps/map.yaml
RVIZ_CONFIG=$WS_DIR/rviz/simple_planner_config.rviz

# === First terminal: RViz (software rendering forced) ===
gnome-terminal -- bash -c "
  source /opt/ros/jazzy/setup.bash;
  cd $WS_DIR;
  source install/setup.bash;
  echo '[INFO] Launching RViz with LIBGL_ALWAYS_SOFTWARE=1...';
  LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d $RVIZ_CONFIG;
  exec bash
"

# Dai tempo a RViz ad avviarsi
sleep 2

# === Second terminal: map_server ===
gnome-terminal -- bash -c "
  source /opt/ros/jazzy/setup.bash;
  cd $WS_DIR;
  source install/setup.bash;
  echo '[INFO] Launching map_server...';
  ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$MAP_FILE;
  exec bash
"

# Dai tempo a map_server ad avviarsi
sleep 4

# === Third terminal: lifecycle + planner ===
gnome-terminal -- bash -c "
  source /opt/ros/jazzy/setup.bash;
  cd $WS_DIR;
  source install/setup.bash;
  echo '[INFO] Configuring and activating map_server...';
  ros2 lifecycle set /map_server configure;
  ros2 lifecycle set /map_server activate;
  echo '[INFO] Launching simple_planner...';
  ros2 run simple_planner simple_planner_node --ros-args --params-file $PARAM_FILE;
  exec bash
"

echo '[INFO] Tutto avviato! Terminali separati:'
echo '  1) RViz (software rendering)'
echo '  2) map_server'
echo '  3) lifecycle + simple_planner'
