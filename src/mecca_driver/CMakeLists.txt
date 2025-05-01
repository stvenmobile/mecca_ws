cmake_minimum_required(VERSION 3.8)
project(mecca_driver)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
