#pragma once

#include <string>

#include <geometry_common/Circle.h>
#include <geometry_common/Point2D.h>

namespace cabin {

class MovingCircle : public kelo::geometry_common::Circle
{
    public:
        kelo::geometry_common::Vector2D vel;

        MovingCircle(const Circle& circle, float vel_x = 0.0f, float vel_y = 0.0f):
            Circle(circle),
            vel(vel_x, vel_y) {}

        friend std::ostream& operator << (std::ostream& out, const MovingCircle& mc)
        {
            out << "<x: " << mc.x
                << ", y: " << mc.y
                << ", r: " << mc.r
                << ", vx: " << mc.vel.x
                << ", vy: " << mc.vel.y
                << ">";
            return out;
        }
};

} // namespace cabin
