cmake_minimum_required(VERSION 3.8)
project(sonar_3d_demo)

find_package(ament_cmake REQUIRED)
find_package(CUDAToolkit QUIET)

if(CUDAToolkit_FOUND)
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
  install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
  install(DIRECTORY rviz   DESTINATION share/${PROJECT_NAME})
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

  install(
    PROGRAMS scripts/play_object_trajectory.py
    DESTINATION lib/${PROJECT_NAME}
  )

  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
else()
  message(STATUS "CUDA Toolkit not found: Skipping sonar_3d_demo targets")
endif()

ament_package()
