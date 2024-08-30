#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

#include "visual_odometry/solve/matcher.hpp"

namespace VO
{
/**
 * - Only called during initialization.
 * - When called, only features of the left image are found, 
 *   so it uses optical flow to find the corresponding features in the right image.
 */
void Matcher::matchStereo(Context& context)
{
    // at this point, only the left features are populated
    std::vector<cv::Point2f> points_left = context.frame_curr_->getPointsLeft2D();
    std::vector<cv::Point2f> points_right;
    std::vector<float> err;
    std::vector<uchar> status;
    cv::TermCriteria term(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
    cv::Size window_size(30, 30);
    
    // calculate the optical flow from left features to right features
    cv::calcOpticalFlowPyrLK(
        context.frame_curr_->img_left_,context.frame_curr_->img_right_,
        points_left, points_right,
        status, err, window_size, 3, term, 0, 0.001);

    std::vector<std::shared_ptr<Feature>> features_left = context.frame_curr_->features_left_;
    context.frame_curr_->features_left_.clear();
    context.frame_curr_->features_right_.clear();
    for (std::size_t i=0; i<status.size(); ++i)
    {
        if (points_left[i].x < 0 || points_left[i].y < 0 ||
            points_right[i].x < 0 || points_right[i].y < 0 ||
            !status[i])
        {}
        else
        {
            context.frame_curr_->features_left_.push_back(features_left[i]);
            context.frame_curr_->features_right_.push_back(points_right[i]);
        }
    }
    context.frame_curr_->features_left_.shrink_to_fit();
    context.frame_curr_->features_right_.shrink_to_fit();
}

void Matcher::matchCircular(Context& context)
{
    // already populated
    std::vector<cv::Point2f> points_left_prev = context.frame_prev_->getPointsLeft2D();
    std::vector<cv::Point2f>& points_right_prev = context.frame_prev_->features_right_;

    // empty
    std::vector<cv::Point2f> points_left_curr;
    std::vector<cv::Point2f> points_right_curr;

    std::vector<float> err;
    std::vector<uchar> status_0;
    std::vector<uchar> status_1;
    std::vector<uchar> status_2;
    cv::TermCriteria term(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
    cv::Size window_size(21, 21);

    // circular match
    // curr left and right are matched with prev left and right to populate them
    // prev left & curr left
    cv::calcOpticalFlowPyrLK(
        context.frame_prev_->img_left_, context.frame_curr_->img_left_, 
        points_left_prev, points_left_curr,
        status_0, err, window_size, 3, term, 0, 0.001);
    // prev right & curr right
    cv::calcOpticalFlowPyrLK(
        context.frame_prev_->img_right_, context.frame_curr_->img_right_, 
        points_right_prev, points_right_curr,
        status_1, err, window_size, 3, term, 0, 0.001);

    // then the curr left and rights are matched for circular matching
    // curr left & curr right
    cv::calcOpticalFlowPyrLK(
        context.frame_curr_->img_left_, context.frame_curr_->img_right_, 
        points_left_curr, points_right_curr,
        status_2, err, window_size, 3, term, cv::OPTFLOW_USE_INITIAL_FLOW, 0.001);

    /**
     * Important note:
     * Bad features are deleted from the left features list, but the right features list is 
     * repopulated from a new list. This is because the features in the left list have landmarks
     * associated with them, which is cannot be deleted.
     */
    std::vector<std::shared_ptr<Feature>> features_left_prev = context.frame_prev_->features_left_;
    context.frame_prev_->features_left_.clear();
    context.frame_prev_->features_right_.clear();
    for (std::size_t i=0; i<status_2.size(); ++i)
    {
        if (!status_0[i] || !status_1[i] || !status_2[i] ||
            points_left_prev[i].x < 0 || points_left_prev[i].y < 0 ||
            points_right_prev[i].x < 0 || points_right_prev[i].y < 0 ||
            points_left_curr[i].x < 0 || points_left_curr[i].y < 0 ||
            points_right_curr[i].x < 0 || points_right_curr[i].y < 0)
        {}
        else
        {
            // repopulate previous features list with good features
            context.frame_prev_->features_left_.push_back(features_left_prev[i]);
            context.frame_prev_->features_right_.push_back(points_right_prev[i]);
            // populate the features list of the current image
            context.frame_curr_->pushFeatureLeft(points_left_curr[i]);
            context.frame_curr_->features_right_.push_back(points_right_curr[i]);
        }
    }
    context.frame_prev_->features_left_.shrink_to_fit();
    context.frame_prev_->features_right_.shrink_to_fit();

    std::cout << context.frame_prev_->features_left_.size() << " Features matched\n";
}
}