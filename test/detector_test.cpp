#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "visual_odometry/solve/detector.hpp"
#include "visual_odometry/sensor/context.hpp"
#include "visual_odometry/parse_KITTI.hpp"

int main(int argc, char** argv)
{
    VO::ParseKITTI pk;
    pk.loadFrames("/home/dev_ws/data/00/");

    VO::Context context;

    VO::Detector detector;
    detector.grid_cell_size_ = 5;
    unsigned int iterations{0};
    for (auto& frame : pk.frames_)
    {
        std::cout << "Iteration: " << iterations << std::endl;
        ++iterations;
        cv::Mat img_left = cv::imread(frame.left_frame_, cv::IMREAD_GRAYSCALE);
        cv::Mat img_right = cv::imread(frame.right_frame_, cv::IMREAD_GRAYSCALE);

        context.insertImages(img_left, img_right);
        if (!context.frame_prev_) {continue;}
        detector.detectFeatures(context);
        context.frame_prev_->features_left_.clear();
    }
    return 0;
}