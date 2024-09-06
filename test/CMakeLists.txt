add_executable(ba_test ba_test.cpp ../src/sensor/map.cpp ../src/sensor/frame.cpp)
target_link_libraries(ba_test ${OpenCV_LIBS} Eigen3::Eigen Ceres::ceres)
ament_target_dependencies(ba_test
    rclcpp
    visualization_msgs
    geometry_msgs
    std_msgs
)

add_executable(test_stuff test.cpp)
target_link_libraries(test_stuff ${OpenCV_LIBS} Eigen3::Eigen)

install(TARGETS 
    ba_test
    test_stuff
    DESTINATION lib/${PROJECT_NAME}    
)