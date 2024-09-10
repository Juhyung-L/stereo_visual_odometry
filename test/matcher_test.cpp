#include <memory>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "visual_odometry/sensor/frame.hpp"

int main(void)
{
    std::string path_to_img_left = "/home/dev_ws/data/00/image_0/000000.png";
    std::string path_to_img_right = "/home/dev_ws/data/00/image_1/000000.png";
    std::shared_ptr<VO::Frame> frame = std::make_shared<VO::Frame>();
    frame->img_left_ = cv::imread(path_to_img_left);
    frame->img_right_ = cv::imread(path_to_img_right);
    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(20, true);
    std::vector<cv::KeyPoint> kps;

    // detect
    detector->detect(frame->img_left_, kps);
    std::vector<cv::Point2f> pts_left;
    pts_left.reserve(kps.size());
    for (const cv::KeyPoint& kp : kps)
    {
        pts_left.push_back(kp.pt);
    }

    // match
    std::vector<cv::Point2f> pts_right;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::TermCriteria term = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
    cv::calcOpticalFlowPyrLK(frame->img_left_, frame->img_right_, pts_left, pts_right, status, err, cv::Size(21, 21), 3, term, 0, 0.001);

    std::vector<cv::Point2f> pts_left_filt, pts_right_filt;
    for (size_t i=0; i<status.size(); ++i)
    {
        if (status[i])
        {
            if (pts_left[i].x < 0 || pts_left[i].y < 0 ||
                pts_right[i].x < 0 || pts_right[i].y < 0)
            {
                std::cout << "Point at index " << i << " out of bounds\n";
            }
            pts_left_filt.push_back(pts_left[i]);
            pts_right_filt.push_back(pts_right[i]);
        }
    }

    // visualize matching
    const int height = frame->img_left_.rows;
    cv::Mat two_imgs;
    cv::vconcat(frame->img_left_, frame->img_right_, two_imgs);

    // draw lines
    for (size_t i=0; i<pts_left_filt.size(); ++i)
    {
        pts_left_filt[i].y += height;
        cv::line(two_imgs, pts_left_filt[i], pts_right_filt[i], cv::Scalar(255,0,0));
    }
    cv::imshow("Optical Flow", two_imgs);
    cv::waitKey(0);

    return 0;
}