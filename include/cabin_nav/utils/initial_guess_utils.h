#ifndef CABIN_INITIAL_GUESS_UTILS_H
#define CABIN_INITIAL_GUESS_UTILS_H

#include <yaml-cpp/yaml.h>

#include <geometry_common/XYTheta.h>

#include <cabin_nav/structs/trajectory_point.h>
#include <cabin_nav/mpc/optimiser.h>

namespace cabin {

class InitialGuessUtils
{
    public:

        InitialGuessUtils() = default;

        virtual ~InitialGuessUtils() = default;

        void configure(
                const YAML::Node& params,
                const kelo::geometry_common::Velocity2D& max_vel,
                const kelo::geometry_common::Velocity2D& min_vel,
                const kelo::geometry_common::Acceleration2D& max_acc,
                const std::vector<float>& behavior_sample_times,
                bool is_unicycle = false);

        void configure(
                const std::vector<kelo::geometry_common::Velocity2D>& velocities,
                const kelo::geometry_common::Acceleration2D& max_acc, 
                const std::vector<float>& behavior_sample_times,
                size_t num_of_controls,
                float sample_time,
                size_t states_per_control,
                size_t counter_threshold,
                bool is_unicycle);

        void reset();

        void setCostFunction(CostFunction calcCost);

        void calcInitialGuessIntermittently(
                const TrajectoryPoint& current,
                std::vector<float>& u,
                Trajectory& trajectory);

        static void interpolateU(
                const std::vector<float>& behavior_sample_times,
                std::vector<float>& u,
                bool is_unicycle = false);

        void calcInitialGuess(
                const TrajectoryPoint& current,
                std::vector<float>& best_u,
                Trajectory& trajectory) const;

        static void trajectoryToU(
                const TrajectoryPoint& current,
                const std::vector<float>& behavior_sample_times,
                const Trajectory& trajectory,
                std::vector<float>& u,
                const kelo::geometry_common::Acceleration2D& max_acc,
                bool is_unicycle);

        std::vector<float> getSampleTimes() const;

        void printVelocities() const;

    private:

        std::vector<Trajectory> trajectories_;
        float sample_time_{1.0f};
        size_t states_per_control_{1};

        /* for intermittent calculations */
        std::vector<float> behavior_sample_times_;
        CostFunction calcCost_;
        size_t counter_{0};
        size_t counter_threshold_{1};
        bool is_unicycle_{false};

        kelo::geometry_common::Acceleration2D max_acc_;

        std::vector<kelo::geometry_common::Velocity2D> velocities_;

        void generateTrajectories(
                const std::vector<kelo::geometry_common::Velocity2D>& velocities,
                size_t num_of_controls);

        static std::vector<kelo::geometry_common::Velocity2D> generateVelocities(
                float max_vel_x,
                float min_vel_x,
                float max_vel_y,
                float max_vel_theta,
                size_t num_of_vel_x_samples,
                size_t num_of_vel_y_samples,
                size_t num_of_vel_theta_samples,
                bool quadratic_x_samples = false,
                bool quadratic_y_samples = false,
                bool quadratic_theta_samples = false,
                bool apply_elliptical_filter = true);

};

} // namespace cabin

#endif // CABIN_INITIAL_GUESS_UTILS_H
