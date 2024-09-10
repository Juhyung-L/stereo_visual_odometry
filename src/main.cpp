#include <iostream>
#include <memory>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "visual_odometry/parse_KITTI.hpp"
#include "visual_odometry/frontend.hpp"
#include "visual_odometry/sensor/map.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    VO::ParseKITTI pk;
    pk.loadFrames("/home/dev_ws/data/00/");
    pk.loadCameras("/home/dev_ws/data/00/calib.txt");
    pk.loadGroundTruth("/home/dev_ws/data/00/00.txt");

    std::shared_ptr<VO::Map> map = std::make_shared<VO::Map>();

    // push P0 and P1 since we are using grey scale cameras
    VO::Frontend frontend(pk.cameras_[0], pk.cameras_[1], map);
    frontend.enableBundleAdjustment();
    
    frontend.visualizer_->visualizeGroundTruth(pk.ground_truth_poses_);
    for (auto& frame : pk.frames_)
    {
        cv::Mat left_img = cv::imread(frame.left_frame_, cv::IMREAD_GRAYSCALE);
        cv::Mat right_img = cv::imread(frame.right_frame_, cv::IMREAD_GRAYSCALE);

        frontend.insertImages(left_img, right_img);
        rclcpp::spin_some(frontend.visualizer_);
        
        if (!rclcpp::ok())
        {
            break;
        }
        usleep(100000);
    }
    rclcpp::shutdown();
    return 0;
}