#ifndef CABIN_MODEL_H
#define CABIN_MODEL_H

#include <cabin_nav/structs/trajectory_point.h>
#include <geometry_common/XYTheta.h>

namespace cabin {

class Model
{
    public:
        /**
         * @brief 
         * 
         * ASSUMPTIONS
         * if is_unicycle then
         *      u.size() % 2 == 0 && sample_times.size() * 2 == u.size()
         *      current_state.vel_y == 0.0f
         * else
         *      u.size() % 3 == 0 && sample_times.size() * 3 == u.size()
         *
         * @param current_state current state of the robot
         * @param u commanded accelerations represented in a list. \n
         * [acc_x_t0, acc_y_t0, acc_theta_t0, acc_x_t1, acc_y_t1, acc_theta_t1, ...]
         * @param sample_times different in times between two steps \n
         * [t1-t0, t2-t1, ...]
         * @param is_unicycle if the commanded acceleration (`u`) should be
         * considered for unicycle kinematics.
         *
         * @return
         */
        static Trajectory calcTrajectory(
                const TrajectoryPoint& current_state,
                const std::vector<float>& u,
                const std::vector<float>& sample_times,
                bool is_unicycle = false);

        /**
         * @brief Predict the next state of the robot when given acceleration is
         * applied. Uses Euler forward integration.
         * 
         * ASSUMPTIONS
         * accelerations are constant over sample_time and can be applied
         * instantaneously (i.e. no jerk limits)
         *
         * @param current current state of the robot
         * @param sample_time amount of time for which to apply constant acc
         * @param acc commanded acceleration
         * @param max_delta_t largest amount of time for a single euler forward
         * integration step. Smaller value gives better accuracy but is
         * computationally expensive.
         *
         * @return predicted state after applying given acceleration
         */
        static TrajectoryPoint predictNextTrajectoryPoint(
                const TrajectoryPoint& current,
                float sample_time,
                const kelo::geometry_common::Acceleration2D& acc,
                float max_delta_t = 0.1f);

};

} // namespace cabin

#endif // CABIN_MODEL_H
