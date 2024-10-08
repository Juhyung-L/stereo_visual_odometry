#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "visual_odometry/parse_KITTI.hpp"
#include "visual_odometry/frontend.hpp"
#include "visual_odometry/sensor/map.hpp"
#include "visual_odometry/visualizer.hpp"

void setParams(const rclcpp::Node::SharedPtr& node, const std::shared_ptr<VO::Frontend>& frontend)
{
    node->declare_parameter("do_bundle_adjustment", false);
    node->declare_parameter("min_num_features", 500);
    node->declare_parameter("grid_cell_size", 10);
    node->declare_parameter("bundle_adjustment_window", 20);
    node->declare_parameter("max_delta_pose_norm", 50.0);
    node->declare_parameter("num_active_frames", 30);

    frontend->do_bundle_adjustment_ = node->get_parameter("do_bundle_adjustment").as_bool();
    frontend->min_num_features_ = static_cast<unsigned long>(node->get_parameter("min_num_features").as_int());
    frontend->grid_cell_size_ = node->get_parameter("grid_cell_size").as_int();
    frontend->bundle_adjustment_window_ = node->get_parameter("bundle_adjustment_window").as_int();
    frontend->max_delta_pose_norm_ = node->get_parameter("max_delta_pose_norm").as_double();
    frontend->num_active_frames_ = node->get_parameter("num_active_frames").as_int();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    VO::ParseKITTI pk;
    pk.loadFrames("/home/dev_ws/data/00/");
    pk.loadCameras("/home/dev_ws/data/00/calib.txt");
    pk.loadGroundTruth("/home/dev_ws/data/00/00.txt");

    rclcpp::Node::SharedPtr param_node = std::make_shared<rclcpp::Node>("param_node");

    std::shared_ptr<VO::Map> map = std::make_shared<VO::Map>();
    std::shared_ptr<VO::Visualizer> visualizer_ = std::make_shared<VO::Visualizer>();
    // push P0 and P1 since we are using grey scale cameras
    std::shared_ptr<VO::Frontend> frontend = std::make_shared<VO::Frontend>(pk.cameras_[0], pk.cameras_[1], map);
    setParams(param_node, frontend);
    frontend->initialize();
    
    visualizer_->visualizeGroundTruth(pk.ground_truth_poses_);
    for (auto& frame : pk.frames_)
    {
        cv::Mat img_left = cv::imread(frame.left_frame_, cv::IMREAD_GRAYSCALE);
        cv::Mat img_right = cv::imread(frame.right_frame_, cv::IMREAD_GRAYSCALE);

        frontend->context_.insertImages(img_left, img_right);
        frontend->visualOdometryPipeline();
        visualizer_->visualize(frontend->context_, frontend->poses_);
        rclcpp::spin_some(visualizer_);
        
        if (!rclcpp::ok()) {break;}
    }
    rclcpp::shutdown();
    return 0;
}