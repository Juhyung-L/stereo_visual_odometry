add_executable(ba_test ba_test.cpp ../src/sensor/map.cpp ../src/sensor/frame.cpp)
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

install(TARGETS 
    ba_test
    matcher_test
    sophus_test
    DESTINATION lib/${PROJECT_NAME}    
)