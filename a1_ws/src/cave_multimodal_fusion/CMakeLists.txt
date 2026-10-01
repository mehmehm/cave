cmake_minimum_required(VERSION 3.0.2)
project(cave_multimodal_fusion)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  message_filters
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    image_geometry
    message_filters
    rospy
    sensor_msgs
    std_msgs
    tf
    tf2_ros
    tf2_sensor_msgs
    visualization_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/rgb_lidar_projection.py
    scripts/rgb_thermal_lidar_sync.py
    scripts/multimodal_mode_selector.py
    scripts/multimodal_mode_selector_experiment.py
    scripts/multimodal_voxel_accumulator_clean.py
    scripts/experiment_metrics_logger.py
    scripts/fix_velodyne_time.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
