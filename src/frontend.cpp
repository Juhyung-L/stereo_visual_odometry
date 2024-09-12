#include <iostream>

#include "visual_odometry/frontend.hpp"

namespace VO
{
Frontend::Frontend(const Camera& camera_left, const Camera& camera_right, const std::shared_ptr<Map> map)
: camera_left_(camera_left)
, camera_right_(camera_right)
, map_(map)
{
    detector_ = Detector(grid_cell_size_);
    triangulator_ = Triangulator(camera_left_, camera_right_, map);
    visualizer_ = std::make_shared<Visualizer>();
}

void Frontend::visualOdometryPipeline()
{
    ++iterations_;
    std::cout << "------------------------------------------------------------'\n";
    if (!context_.frame_prev_) {return;}

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
    estimator_.estimate(context_, camera_left_.K_);

    // insert current frame into map
    map_->insertFrame(context_.frame_curr_);
    // this function deletes landmarks in the map that have no features pointing to it
    map_->cleanMap();
    
    // bundle adjustment
    if (do_bundle_adjustment_ && iterations_ % bundle_adjustment_window_ == 0)
    {
        optimizer_.optimize(map_, camera_left_.K_, loss_function_scale_);
    }

    poses_.push_back(context_.frame_prev_->pose_);
    visualizer_->visualize(context_, poses_);
    std::cout << "------------------------------------------------------------'\n";
}
}