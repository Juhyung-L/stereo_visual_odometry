#ifndef FRONTEND_HPP_
#define FRONTEND_HPP_

#include <memory>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "visual_odometry/sensor/context.hpp"
#include "visual_odometry/sensor/map.hpp"
#include "visual_odometry/sensor/camera.hpp"
#include "visual_odometry/solve/detector.hpp"
#include "visual_odometry/solve/matcher.hpp"
#include "visual_odometry/solve/triangulator.hpp"
#include "visual_odometry/solve/estimator.hpp"
#include "visual_odometry/solve/optimizer.hpp"

namespace VO
{
class Frontend
{
public:    
    Frontend() = default;
    Frontend(const Camera& camera_left, const Camera& camera_right, const std::shared_ptr<Map> map);
    void initialize();
    void visualOdometryPipeline();

    Context context_;
    std::vector<Sophus::SE3d> poses_;

    // parameters
    bool do_bundle_adjustment_{false};
    unsigned long min_num_features_{500};
    int grid_cell_size_{10};
    int bundle_adjustment_window_{20};
    double max_delta_pose_norm_{10.0};
    int num_active_frames_{30};

private:
    
    Detector detector_;
    Matcher matcher_;
    Triangulator triangulator_;
    Estimator estimator_;
    Optimizer optimizer_;

    Camera camera_left_;
    Camera camera_right_;

    std::shared_ptr<Map> map_;
    unsigned long iterations_{1};

    bool estimation_status_;
    bool retried_{false};

    void resetContext();
    void resetMap();
};
}

#endif