#ifndef MATCHER_HPP_
#define MATCHER_HPP_

#include "visual_odometry/sensor/context.hpp"

namespace VO
{
class Matcher
{
public:
    Matcher() = default;
    void matchStereo(Context& context);
    void matchCircular(Context& context);
};
}

#endif
