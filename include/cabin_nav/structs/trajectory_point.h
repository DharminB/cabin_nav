#ifndef CABIN_trajectory_point_H
#define CABIN_trajectory_point_H

#include <geometry_common/Pose2D.h>
#include <geometry_common/XYTheta.h>
#include <geometry_common/TransformMatrix2D.h>

namespace cabin {

class TrajectoryPoint; // forward declaration

using Trajectory = std::vector<TrajectoryPoint>;

class TrajectoryPoint
{
    public:
        kelo::geometry_common::Pose2D pos;
        kelo::geometry_common::Velocity2D vel;
        float t{0.0f};

        using Ptr = std::shared_ptr<TrajectoryPoint>;
        using ConstPtr = std::shared_ptr<const TrajectoryPoint>;

        TrajectoryPoint(float x = 0.0f, float y = 0.0f, float theta = 0.0f,
                   float vel_x = 0.0f, float vel_y = 0.0f, float vel_theta = 0.0f,
                   float _t = 0.0f):
            pos(x, y, theta),
            vel(vel_x, vel_y, vel_theta),
            t(_t) {}

        TrajectoryPoint(const TrajectoryPoint& state):
            pos(state.pos),
            vel(state.vel),
            t(state.t) {}

        TrajectoryPoint(const kelo::geometry_common::Pose2D& _pos,
                   const kelo::geometry_common::Velocity2D& _vel =
                            kelo::geometry_common::Velocity2D(0.0f, 0.0f, 0.0f),
                   float _t = 0.0f):
            pos(_pos),
            vel(_vel),
            t(_t) {}

        virtual ~TrajectoryPoint() {}

        bool operator == (const TrajectoryPoint& other);

        TrajectoryPoint operator - (const TrajectoryPoint& other) const;

        TrajectoryPoint operator * (
                const kelo::geometry_common::TransformMatrix2D& tf) const;

        friend TrajectoryPoint operator * (
                const kelo::geometry_common::TransformMatrix2D& tf,
                const TrajectoryPoint& state);

        friend Trajectory operator * (
                const Trajectory& traj,
                const kelo::geometry_common::TransformMatrix2D& tf);

        friend Trajectory operator * (
                const kelo::geometry_common::TransformMatrix2D& tf,
                const Trajectory& traj);

        friend std::ostream& operator << (std::ostream& out, const TrajectoryPoint& state);

        friend std::ostream& operator << (std::ostream& out, const Trajectory& traj);

};

} // namespace cabin
#endif // CABIN_trajectory_point_H
