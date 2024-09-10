#include <iostream>

#include "visual_odometry/frontend.hpp"

namespace VO
{
Frontend::Frontend(const Camera& camera_left, const Camera& camera_right, const std::shared_ptr<Map> map)
: camera_left_(camera_left)
, camera_right_(camera_right)
, map_(map)
{
    detector_ = Detector(bucket_cell_size_);
    triangulator_ = Triangulator(camera_left_, camera_right_, map);
    visualizer_ = std::make_shared<Visualizer>();
}

void Frontend::insertImages(cv::Mat img_left, cv::Mat img_right)
{
    context_.insertImages(img_left, img_right);
    switch (status_)
    {
        case INITIALIZING:
            initialize();
            break;
        case TRACKING:
            track();
            break;
        default:
            throw std::runtime_error("Unknown status");
    }
    ++iterations_;
}

void Frontend::initialize()
{
    detector_.detectFeatures(context_);
    matcher_.matchStereo(context_);
    status_ = TRACKING;
}

void Frontend::track()
{
    std::cout << "------------------------------------------------------------'\n";
    matcher_.matchCircular(context_);
    triangulator_.triangulate(context_);
    estimator_.estimate(context_, camera_left_.K_);

    visualizer_->visualize(context_);

    // insert current frame into map
    map_->insertKeyFrame(context_.frame_curr_);
    // this function deletes landmarks in the map that have no features pointing to it
    map_->cleanMap();
    
    // bundle adjustment
    if (do_bundle_adjustment_ && iterations_ % 15 == 0)
    {
        optimizer_.optimize(map_, camera_left_.K_);
    }

    // if number of features is below the threshold, add new features
    if (context_.frame_curr_->features_left_.size() < min_feature_size_)
    {
        detector_.detectFeatures(context_);
        matcher_.matchStereo(context_);
    }
    std::cout << "------------------------------------------------------------'\n";
}
}