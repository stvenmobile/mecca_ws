cmake_minimum_required(VERSION 3.8)
project(mecca_description)

# Core build system
find_package(ament_cmake REQUIRED)

# If you ever want to preprocess XACRO → URDF at build time, you can
# uncomment these two lines (and install ros-jazzy-ament-xacro):
# find_package(ament_xacro REQUIRED)
# ament_export_dependencies(ament_xacro)

# Install only your description assets
install(
  DIRECTORY
    urdf
    meshes
    rviz
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
