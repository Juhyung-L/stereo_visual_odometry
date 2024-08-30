#ifndef FRONTEND_HPP_
#define FRONTEND_HPP_

#include <memory>

#include <opencv2/core.hpp>

#include "visual_odometry/sensor/context.hpp"
#include "visual_odometry/sensor/map.hpp"
#include "visual_odometry/sensor/camera.hpp"
#include "visual_odometry/solve/detector.hpp"
#include "visual_odometry/solve/matcher.hpp"
#include "visual_odometry/solve/triangulator.hpp"
#include "visual_odometry/solve/estimator.hpp"
#include "visual_odometry/visualizer.hpp"

namespace VO
{
class Frontend
{
public:
    enum Status {INITIALIZING, TRACKING};
    
    Frontend() = default;
    Frontend(const Camera& camera_left, const Camera& camera_right, const std::shared_ptr<Map> map);
    void insertImages(cv::Mat img_left, cv::Mat img_right);

    std::shared_ptr<Visualizer> visualizer_;

private:
    void initialize();
    void process();
    
    Context context_;
    Status status_{INITIALIZING};

    Detector detector_;
    Matcher matcher_;
    Triangulator triangulator_;
    Estimator estimator_;

    Camera camera_left_;
    Camera camera_right_;

    std::shared_ptr<Map> map_;
    const int bucket_cell_size_{15};
    const unsigned long min_feature_size_{2000};
};
}

#endif