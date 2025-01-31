cmake_minimum_required(VERSION 3.8)
project(ariac_gz_plugins)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  gz-plugin2
  gz-cmake3
  gz-sim8
  gz-sensors8
  rclcpp
  sensor_msgs
  ros_gz_bridge
  ariac_msgs
  aprs_interfaces
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(AriacCameraPlugin SHARED src/ariac_camera_plugin.cpp)
target_include_directories(AriacCameraPlugin PUBLIC include)
ament_target_dependencies(AriacCameraPlugin 
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(AriacCameraPlugin)
install(TARGETS AriacCameraPlugin
        DESTINATION lib/${PROJECT_NAME}
)


add_library(AriacLogicalCameraPlugin SHARED src/ariac_logical_camera_plugin.cpp)
target_include_directories(AriacLogicalCameraPlugin PUBLIC include)
ament_target_dependencies(AriacLogicalCameraPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(AriacLogicalCameraPlugin)
install(TARGETS AriacLogicalCameraPlugin
        DESTINATION lib/${PROJECT_NAME}
)


add_library(AriacRayPlugin SHARED src/ariac_ray_plugin.cpp)
target_include_directories(AriacRayPlugin PUBLIC include)
ament_target_dependencies(AriacRayPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(AriacRayPlugin)
install(TARGETS AriacRayPlugin
        DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY worlds models
        DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
