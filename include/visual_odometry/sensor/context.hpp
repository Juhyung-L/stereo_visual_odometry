#ifndef CONTEXT_HPP_
#define CONTEXT_HPP_

#include <memory>
#include <vector>

#include <opencv2/core.hpp>

#include "visual_odometry/sensor/frame.hpp"

namespace VO
{
class Context
{
public:
    std::shared_ptr<Frame> frame_prev_{nullptr};
    std::shared_ptr<Frame> frame_curr_{nullptr};

    std::vector<cv::Point2f> points_left_;
    std::vector<cv::Point2f> points_right_;

    unsigned long frame_id_{0};
    void insertImages(cv::Mat img_left, cv::Mat img_right)
    {
        std::shared_ptr<Frame> frame = std::make_shared<Frame>();
        frame->img_left_ = img_left;
        frame->img_right_ = img_right;
        frame->id_ = frame_id_;
        frame_prev_ = frame_curr_;
        frame_curr_ = frame;
        ++frame_id_;
    }
};
}

#endif