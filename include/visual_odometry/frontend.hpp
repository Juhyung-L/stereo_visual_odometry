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
#include "visual_odometry/visualizer.hpp"

namespace VO
{
class Frontend
{
public:    
    Frontend() = default;
    Frontend(const Camera& camera_left, const Camera& camera_right, const std::shared_ptr<Map> map);
    void visualOdometryPipeline();

    Context context_;

    std::shared_ptr<Visualizer> visualizer_;
    bool do_bundle_adjustment_{false};
    unsigned long min_num_features_{500};
    int grid_cell_size_{5};
    double loss_function_scale_{1.0};
    int bundle_adjustment_window_{20};

private:
    
    Detector detector_;
    Matcher matcher_;
    Triangulator triangulator_;
    Estimator estimator_;
    Optimizer optimizer_;

    Camera camera_left_;
    Camera camera_right_;

    std::shared_ptr<Map> map_;
    unsigned long iterations_{0};
    std::vector<Sophus::SE3d> poses_;
};
}

#endif