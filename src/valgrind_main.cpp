#include <iostream>
#include <memory>
#include <atomic>
#include <csignal>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "visual_odometry/parse_KITTI.hpp"
#include "visual_odometry/frontend.hpp"
#include "visual_odometry/sensor/map.hpp"

std::atomic<bool> run{true};

void sigint_handler(int signal)
{
    run = false;
}

int main(int argc, char** argv)
{
    VO::ParseKITTI pk;
    pk.loadFrames("/home/dev_ws/data/00/");
    pk.loadCameras("/home/dev_ws/data/00/calib.txt");
    pk.loadGroundTruth("/home/dev_ws/data/00/00.txt");

    std::shared_ptr<VO::Map> map = std::make_shared<VO::Map>();
    // push P0 and P1 since we are using grey scale cameras
    VO::Frontend frontend(pk.cameras_[0], pk.cameras_[1], map);
    frontend.initialize();
    
    for (auto& frame : pk.frames_)
    {
        cv::Mat img_left = cv::imread(frame.left_frame_, cv::IMREAD_GRAYSCALE);
        cv::Mat img_right = cv::imread(frame.right_frame_, cv::IMREAD_GRAYSCALE);

        frontend.context_.insertImages(img_left, img_right);
        frontend.visualOdometryPipeline();
        if (!run) {break;}
    }
    return 0;
}