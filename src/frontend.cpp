#include <iostream>

#include "visual_odometry/frontend.hpp"

namespace VO
{
Frontend::Frontend(const Camera& camera_left, const Camera& camera_right, const std::shared_ptr<Map> map)
: camera_left_(camera_left)
, camera_right_(camera_right)
, map_(map)
{}

void Frontend::initialize()
{
    detector_.grid_cell_size_ = grid_cell_size_;
    optimizer_.loss_function_scale_ = loss_function_scale_;
    triangulator_.camera_left_ = camera_left_;
    triangulator_.camera_right_ = camera_right_;
    triangulator_.map_ = map_;
    estimator_.max_allowed_translation_sq_ = max_allowed_translation_sq_;
}

void Frontend::visualOdometryPipeline()
{
    std::cout << "------------------------------------------------------------'\n";
    if (!context_.frame_prev_) {return;}
    std::cout << "Iteration: " << iterations_ << std::endl;
    /**
     * This do-while loop iterates only if the estimation has failed for the first time.
     * It throws an exception if the estimation fails twice in a row.
     */
    do
    {
        // if number of features is below the threshold, add new features
        if (context_.frame_prev_->features_left_.size() < min_num_features_)
        {
            detector_.detectFeatures(context_);
            matcher_.matchStereo(context_);
        }

        // triangulate features in the previous frame
        triangulator_.triangulate(context_);

        // get optical flow of previous features to current image (copy over the landmarks)
        matcher_.matchCircular(context_);

        // estimate pose of current frame using triangulated features of previous frame
        estimation_status_ = estimator_.estimate(context_, camera_left_.K_);
        if (!estimation_status_ && retried_)
        {
            throw std::runtime_error("Estimation failed twice.");
        }
        else if (!estimation_status_ && !retried_)
        {
            // reset map and context and try again
            resetContext();
            resetMap();
            iterations_ = 1;
            retried_ = true;
            std::cout << "Estimation failed. Retrying." << std::endl;
        }
    }
    while (!estimation_status_);
    retried_ = false;

    // insert current frame into map
    map_->insertFrame(context_.frame_curr_);
    // this function deletes landmarks in the map that have no features pointing to it
    map_->cleanMap();
    
    // bundle adjustment
    if (do_bundle_adjustment_ && iterations_ % bundle_adjustment_window_ == 0)
    {
        optimizer_.optimize(map_, camera_left_.K_, loss_function_scale_);
    }
    std::cout << "------------------------------------------------------------'\n";
    
    poses_.push_back(context_.frame_prev_->pose_);
    ++iterations_;
}

void Frontend::resetContext()
{
    context_.frame_prev_->features_left_.clear();
    context_.frame_prev_->features_right_.clear();
    context_.frame_curr_->features_left_.clear();
    context_.frame_curr_->features_right_.clear();
}

void Frontend::resetMap()
{
    map_ = std::make_shared<Map>();
    triangulator_.map_ = map_;
}
}