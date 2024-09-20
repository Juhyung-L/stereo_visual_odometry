cmake_minimum_required(VERSION 3.8)
project(visual_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Ceres REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  sensor_msgs
  visualization_msgs
  std_msgs
  cv_bridge
  tf2_ros
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

add_executable(main 
  src/main.cpp 
  src/frontend.cpp
  src/parse_KITTI.cpp
  src/visualizer.cpp
  src/sensor/map.cpp
  src/sensor/frame.cpp
  src/solve/matcher.cpp
  src/solve/detector.cpp
  src/solve/triangulator.cpp
  src/solve/estimator.cpp
  src/solve/optimizer.cpp
)
ament_target_dependencies(main ${dependencies})
target_link_libraries(main ${OpenCV_LIBS} ${Sophus_LIBRARIES} Eigen3::Eigen Ceres::ceres)

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME}
)

add_subdirectory(test)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
