#include <cabin_nav/mpc/model.h>

#include <geometry_common/Pose2D.h>
#include <geometry_common/TransformMatrix2D.h>

using kelo::geometry_common::Pose2D;
using kelo::geometry_common::Acceleration2D;
using kelo::geometry_common::TransformMatrix2D;

namespace cabin {

Trajectory Model::calcTrajectory(
        const TrajectoryPoint& current_state,
        const std::vector<float>& u,
        const std::vector<float>& sample_times,
        bool is_unicycle)
{
    Trajectory trajectory;
    TrajectoryPoint current(current_state);

    if ( is_unicycle )
    {
        current.vel.y = 0.0f;
    }

    trajectory.reserve(sample_times.size());
    for ( size_t i = 0; i < sample_times.size(); i++ )
    {
        TrajectoryPoint next_state = ( is_unicycle )
            ? Model::predictNextTrajectoryPoint(
                    current, sample_times[i], Acceleration2D(u[i*2], 0.0f, u[(i*2)+1]))
            : Model::predictNextTrajectoryPoint(
                    current, sample_times[i], Acceleration2D(u[i*3], u[(i*3)+1], u[(i*3)+2]));
        trajectory.push_back(next_state);
        current = next_state;
    }
    return trajectory;
}

TrajectoryPoint Model::predictNextTrajectoryPoint(
        const TrajectoryPoint& current,
        float sample_time,
        const Acceleration2D& acc,
        float max_delta_t)
{
    TrajectoryPoint next_state(current.pos, current.vel, current.t + sample_time);

    float delta_t = max_delta_t;

    TransformMatrix2D pos_mat(current.pos);
    TransformMatrix2D vel_mat;

    for (float remaining_time = sample_time; remaining_time > 0.0f; remaining_time-=delta_t)
    {
        delta_t = std::min(remaining_time, max_delta_t);

        /* v_final = v_init + (acc * delta_t) */
        next_state.vel = next_state.vel + (acc * delta_t);
        vel_mat.update(next_state.vel * delta_t);

        /* x_final = x_init + (v_final * delta_t) */
        pos_mat *= vel_mat;
    }
    next_state.pos = pos_mat.asPose2D();
    return next_state;
}

} // namespace cabin
