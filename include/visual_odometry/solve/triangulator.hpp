#ifndef TRIANGULATOR_HPP_
#define TRIANGULATOR_HPP_

#include <memory>

#include "visual_odometry/sensor/camera.hpp"
#include "visual_odometry/sensor/context.hpp"
#include "visual_odometry/sensor/map.hpp"

namespace VO
{
class Triangulator
{
public:
    Triangulator() = default;
    Triangulator(const Camera& camera_left, const Camera& camera_right, const std::shared_ptr<Map> map);
    void triangulate(Context& context);

    std::shared_ptr<Map> map_;
    Camera camera_left_;
    Camera camera_right_;
    
private:
    std::size_t firstFeatureWithoutLandmark(const std::vector<std::shared_ptr<Feature>>& feature);

};
}

#endif