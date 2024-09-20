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
    triangulator_.camera_left_ = camera_left_;
    triangulator_.camera_right_ = camera_right_;
    triangulator_.map_ = map_;
    estimator_.max_delta_pose_norm_ = max_delta_pose_norm_;
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
        // erase the poses that will get optimized
        if (poses_.size() < map_->frames_.size()) {poses_.clear();}
        else {poses_.erase(poses_.end()-map_->frames_.size(), poses_.end());}
        for (const std::shared_ptr<Frame>& frame : map_->frames_) {poses_.push_back(frame->pose_);}
    }
    else
    {
        poses_.push_back(context_.frame_prev_->pose_);
    }
    std::cout << "------------------------------------------------------------'\n";
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