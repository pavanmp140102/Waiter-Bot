cmake_minimum_required(VERSION 3.5)
project(waiter_bot_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install directories
install(
  DIRECTORY urdf meshes config launch navigation param rviz_config world map UI
  DESTINATION share/${PROJECT_NAME}
)

# Package declaration
ament_package()

