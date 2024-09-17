add_executable(ba_test 
  ba_test.cpp
  ../src/sensor/map.cpp
  ../src/sensor/frame.cpp
)
target_link_libraries(ba_test ${OpenCV_LIBS} Eigen3::Eigen Ceres::ceres)
ament_target_dependencies(ba_test
  rclcpp
  visualization_msgs
  geometry_msgs
  std_msgs
)

add_executable(matcher_test matcher_test.cpp)
target_link_libraries(matcher_test ${OpenCV_LIBS} Eigen3::Eigen)

add_executable(sophus_test sophus_test.cpp)
target_link_libraries(sophus_test ${Sophus_LIBRARIES} Eigen3::Eigen)

add_executable(valgrind_main
  valgrind_main.cpp
  ../src/frontend.cpp
  ../src/parse_KITTI.cpp
  ../src/sensor/map.cpp
  ../src/sensor/frame.cpp
  ../src/solve/matcher.cpp
  ../src/solve/detector.cpp
  ../src/solve/triangulator.cpp
  ../src/solve/estimator.cpp
  ../src/solve/optimizer.cpp
)
target_link_libraries(valgrind_main ${OpenCV_LIBS} ${Sophus_LIBRARIES} Eigen3::Eigen Ceres::ceres)

add_executable(detector_test
  detector_test.cpp
  ../src/parse_KITTI.cpp
  ../src/sensor/frame.cpp
  ../src/solve/detector.cpp
)
target_link_libraries(detector_test ${OpenCV_LIBS} Eigen3::Eigen)

install(TARGETS 
  ba_test
  matcher_test
  sophus_test
  valgrind_main
  detector_test
  DESTINATION lib/${PROJECT_NAME} 
)