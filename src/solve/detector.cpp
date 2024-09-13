#include <iostream>

#include "visual_odometry/sensor/context.hpp"
#include "visual_odometry/sensor/feature.hpp"
#include "visual_odometry/solve/detector.hpp"

/**
 * - Finds new features from the left image.
 * - Divides the image into square cells and only take features inside an unoccupied cell
 * - Can be used when the left features list is not empty, 
 *   in which case new features are added to the same list (replenishes the features list)
 */

namespace VO
{
static const auto keypoint_comparator = [](const cv::KeyPoint& kp1, const cv::KeyPoint& kp2)
{
    return std::abs(kp1.response) > std::abs(kp2.response);
};

Detector::Detector()
: detector_(cv::FastFeatureDetector::create(20, 100))
{}

Detector::Detector(int grid_cell_size)
: grid_cell_size_(grid_cell_size)
{
    Detector();
}

void Detector::detectFeatures(Context& context)
{
    // divide the frame into square cells of size grid_cell_size_ by grid_cell_size_
    const int NUM_GRID_CELL_COLS = std::ceil(static_cast<double>(context.frame_prev_->img_left_.cols )/ grid_cell_size_);
    const int NUM_GRID_CELL_ROWS = std::ceil(static_cast<double>(context.frame_prev_->img_right_.rows )/ grid_cell_size_);
    std::vector<bool> occ_grid(NUM_GRID_CELL_COLS * NUM_GRID_CELL_ROWS, false);

    // mark location of existing features
    std::vector<std::shared_ptr<Feature>>& features = context.frame_prev_->features_left_;
    for (const std::shared_ptr<Feature>& feature : features)
    {
        int x = feature->pixel.x;
        int y = feature->pixel.y;
        int idx = static_cast<int>(y / grid_cell_size_) * NUM_GRID_CELL_COLS + static_cast<int>(x / grid_cell_size_);
        occ_grid[idx] = true;
    }

    // detect new features
    std::vector<cv::KeyPoint> keypoints;
    detector_->detect(context.frame_prev_->img_left_, keypoints);
    std::sort(keypoints.begin(), keypoints.end(), keypoint_comparator);

    // add new features only if they are in unoccupied cells
    int new_feature_cnt = 0;
    for (const cv::KeyPoint& kp : keypoints)
    {
        int x = kp.pt.x;
        int y = kp.pt.y;
        int idx = static_cast<int>(y / grid_cell_size_) * NUM_GRID_CELL_COLS + static_cast<int>(x / grid_cell_size_);
        if (!occ_grid[idx])
        {
            context.frame_prev_->pushFeatureLeft(kp.pt);
            occ_grid[idx] = true;
            ++new_feature_cnt;
        }
    }

    std::cout << new_feature_cnt << " new features added\n";
}
}