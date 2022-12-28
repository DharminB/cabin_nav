#include <iostream>

#include <geometry_common/Pose2D.h>
#include <geometry_common/XYTheta.h>
#include <geometry_common/Utils.h>
#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/mpc/model.h>
#include <cabin_nav/utils/utils.h>

#include <cabin_nav/utils/initial_guess_utils.h>

using kelo::geometry_common::Pose2D;
using kelo::geometry_common::Acceleration2D;
using kelo::geometry_common::Velocity2D;
using GCUtils = kelo::geometry_common::Utils;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

void InitialGuessUtils::configure(
        const YAML::Node& config,
        const Velocity2D& max_vel,
        const Velocity2D& min_vel,
        const Acceleration2D& max_acc,
        const std::vector<float>& behavior_sample_times,
        bool is_unicycle)
{
    size_t num_of_vel_x_samples = Parser::get<size_t>(config, "num_of_vel_x_samples", 5);
    size_t num_of_vel_y_samples = Parser::get<size_t>(config, "num_of_vel_y_samples", 5);
    size_t num_of_vel_theta_samples = Parser::get<size_t>(config, "num_of_vel_theta_samples", 5);

    float max_vel_y = max_vel.y;
    if ( is_unicycle )
    {
        num_of_vel_y_samples = 1;
        max_vel_y = 0.0f;
    }

    bool quadratic_x_samples = Parser::get<bool>(config, "quadratic_x_samples", false);
    bool quadratic_y_samples = Parser::get<bool>(config, "quadratic_y_samples", false);
    bool quadratic_theta_samples = Parser::get<bool>(config, "quadratic_theta_samples", false);

    bool apply_elliptical_filter = Parser::get<bool>(
            config, "apply_elliptical_filter", true);
    size_t num_of_controls = Parser::get<size_t>(config, "num_of_controls", 2);

    float sample_time = Parser::get<float>(config, "sample_time", 1.0f);
    size_t states_per_control = Parser::get<size_t>(config, "states_per_control", 1);

    size_t counter_threshold = Parser::get<size_t>(config, "frequency", 1);

    std::vector<Velocity2D> velocities = InitialGuessUtils::generateVelocities(
            max_vel.x, min_vel.x, max_vel_y, max_vel.theta,
            num_of_vel_x_samples, num_of_vel_y_samples, num_of_vel_theta_samples,
            quadratic_x_samples, quadratic_y_samples, quadratic_theta_samples,
            apply_elliptical_filter);
    configure(velocities, max_acc, behavior_sample_times, num_of_controls,
              sample_time, states_per_control, counter_threshold, is_unicycle);
}

void InitialGuessUtils::configure(
        const std::vector<Velocity2D>& velocities,
        const Acceleration2D& max_acc, 
        const std::vector<float>& behavior_sample_times,
        size_t num_of_controls,
        float sample_time,
        size_t states_per_control,
        size_t counter_threshold,
        bool is_unicycle)
{
    max_acc_ = max_acc;
    sample_time_ = sample_time;
    states_per_control_ = states_per_control;
    behavior_sample_times_ = behavior_sample_times;
    counter_threshold_ = counter_threshold;
    is_unicycle_ = is_unicycle;

    generateTrajectories(velocities, num_of_controls);
}

void InitialGuessUtils::reset()
{
    counter_ = 0;
}

void InitialGuessUtils::setCostFunction(
        CostFunction calcCost)
{
    calcCost_ = calcCost;
}

void InitialGuessUtils::calcInitialGuessIntermittently(
        const TrajectoryPoint& current,
        std::vector<float>& u,
        Trajectory& trajectory)
{
    if ( counter_ == 0 )
    {
        trajectory.clear();
        calcInitialGuess(current, u, trajectory);
        trajectory.insert(trajectory.begin(), TrajectoryPoint());
    }
    else
    {
        InitialGuessUtils::interpolateU(behavior_sample_times_, u, is_unicycle_);
    }

    counter_ ++;
    if ( counter_ >= counter_threshold_ )
    {
        counter_ = 0;
    }
}

void InitialGuessUtils::interpolateU(
        const std::vector<float>& behavior_sample_times,
        std::vector<float>& u,
        bool is_unicycle)
{
    /* calculate non-relative time offset when a control is applied */
    std::vector<float> control_times;
    float accumulated_time = 0.0f;
    control_times.push_back(accumulated_time);
    for ( size_t i = 0; i < behavior_sample_times.size(); i++ )
    {
        accumulated_time += behavior_sample_times[i];
        control_times.push_back(accumulated_time);
    }

    /* make a copy of u */
    std::vector<float> prev_u(u);
    if ( is_unicycle )
    {
        prev_u.insert(prev_u.end(), 2, 0.0f);
    }
    else
    {
        prev_u.insert(prev_u.end(), 3, 0.0f);
    }

    /* interpolate u based on prev_u */
    size_t i = 0;
    for ( size_t j = 0; j+1 < control_times.size(); j++ )
    {
        float lookup_time = control_times[j] + behavior_sample_times[0];
        bool interpolated = false;
        for ( ; i+1 < control_times.size(); i++ )
        {
            if ( lookup_time < control_times[i+1] )
            {
                float t = (lookup_time        - control_times[i])
                        / (control_times[i+1] - control_times[i]);
                if ( is_unicycle )
                {
                    u[ j*2   ] = prev_u[ i*2   ] + (t * (prev_u[ (i+1)*2   ] - prev_u[ i*2   ]));
                    u[(j*2)+1] = prev_u[(i*2)+1] + (t * (prev_u[((i+1)*2)+1] - prev_u[(i*2)+1]));
                }
                else
                {
                    u[ j*3   ] = prev_u[ i*3   ] + (t * (prev_u[ (i+1)*3   ] - prev_u[ i*3   ]));
                    u[(j*3)+1] = prev_u[(i*3)+1] + (t * (prev_u[((i+1)*3)+1] - prev_u[(i*3)+1]));
                    u[(j*3)+2] = prev_u[(i*3)+2] + (t * (prev_u[((i+1)*3)+2] - prev_u[(i*3)+2]));
                }
                interpolated = true;
                break;
            }
        }
        if ( !interpolated )
        {
            if ( is_unicycle )
            {
                u[ j*2   ] = 0.0f;
                u[(j*2)+1] = 0.0f;
            }
            else
            {
                u[ j*3   ] = 0.0f;
                u[(j*3)+1] = 0.0f;
                u[(j*3)+2] = 0.0f;
            }
        }
    }
}

void InitialGuessUtils::calcInitialGuess(
        const TrajectoryPoint& current,
        std::vector<float>& best_u,
        Trajectory& trajectory) const
{
    float min_cost = 10000.0f;
    size_t min_cost_index = 0;
    std::vector<float> u(best_u.size(), 0.0f);
    for ( size_t i = 0; i < trajectories_.size(); i++ )
    {
        trajectoryToU(current, behavior_sample_times_, trajectories_[i],
                      u, max_acc_, is_unicycle_);
        float cost = calcCost_(u);
        if ( cost < min_cost )
        {
            min_cost = cost;
            min_cost_index = i;
            best_u = u;
        }
    }
    trajectory = trajectories_[min_cost_index];
}

void InitialGuessUtils::trajectoryToU(
        const TrajectoryPoint& current,
        const std::vector<float>& behavior_sample_times,
        const Trajectory& trajectory,
        std::vector<float>& u,
        const Acceleration2D& max_acc,
        bool is_unicycle)
{
    float accumulated_time = 0.0f;
    size_t trajectory_index = 0;
    float prev_trajectory_time = 0.0f;
    Acceleration2D req_acc;
    Velocity2D curr_vel = current.vel;
    if ( is_unicycle )
    {
        curr_vel.y = 0.0f;
    }

    for ( size_t i = 0; i < behavior_sample_times.size(); i++ )
    {
        if ( trajectory_index >= trajectory.size() )
        {
            size_t start_index = ( is_unicycle ) ? i*2 : i*3;
            std::fill(u.begin()+start_index, u.end(), 0.0f);
            break;
        }
        /* calculate required acceleration */
        req_acc = (trajectory[trajectory_index].vel - curr_vel) /
                  (trajectory[trajectory_index].t - prev_trajectory_time);

        /* clip to allowable acceleration */
        if ( is_unicycle )
        {
            u[ i*2   ] = GCUtils::clip(req_acc.x,     max_acc.x,     -max_acc.x);
            u[(i*2)+1] = GCUtils::clip(req_acc.theta, max_acc.theta, -max_acc.theta);
        }
        else
        {
            u[ i*3   ] = GCUtils::clip(req_acc.x,     max_acc.x,     -max_acc.x);
            u[(i*3)+1] = GCUtils::clip(req_acc.y,     max_acc.y,     -max_acc.y);
            u[(i*3)+2] = GCUtils::clip(req_acc.theta, max_acc.theta, -max_acc.theta);
        }

        /* update velocity for next iteration */
        if ( is_unicycle )
        {
            curr_vel.x     += u[ i*2   ] * behavior_sample_times[i];
            curr_vel.theta += u[(i*2)+1] * behavior_sample_times[i];
        }
        else
        {
            curr_vel.x     += u[ i*3   ] * behavior_sample_times[i];
            curr_vel.y     += u[(i*3)+1] * behavior_sample_times[i];
            curr_vel.theta += u[(i*3)+2] * behavior_sample_times[i];
        }

        accumulated_time += behavior_sample_times[i];
        while ( trajectory_index < trajectory.size() &&
                accumulated_time >= trajectory[trajectory_index].t )
        {
            prev_trajectory_time = trajectory[trajectory_index].t;
            trajectory_index++;
        }
    }
}

void InitialGuessUtils::generateTrajectories(
        const std::vector<Velocity2D>& velocities,
        size_t num_of_controls)
{
    std::vector<std::vector<int> > indexes = Utils::generateBFIndexes(
            velocities.size()-1, 0, num_of_controls);
    trajectories_.clear();
    Acceleration2D zero_acc;
    for ( size_t i = 0; i < indexes.size(); i++ )
    {
        Trajectory trajectory;
        TrajectoryPoint curr;
        for ( size_t j = 0; j < indexes[i].size(); j++ )
        {
            curr.vel = velocities[indexes[i][j]];
            TrajectoryPoint next_state;
            for ( size_t k = 0; k < states_per_control_; k++ )
            {
                next_state = Model::predictNextTrajectoryPoint(curr,
                        sample_time_/states_per_control_, zero_acc);
                trajectory.push_back(next_state);
                curr = next_state;
            }
        }
        trajectories_.push_back(trajectory);
    }
    // std::cout << trajectories_.size() << std::endl;
}

std::vector<Velocity2D> InitialGuessUtils::generateVelocities(
        float max_vel_x,
        float min_vel_x,
        float max_vel_y,
        float max_vel_theta,
        size_t num_of_vel_x_samples,
        size_t num_of_vel_y_samples,
        size_t num_of_vel_theta_samples,
        bool quadratic_x_samples,
        bool quadratic_y_samples,
        bool quadratic_theta_samples,
        bool apply_elliptical_filter)
{
    /* ASSUMPTION: min_vel_y == -max_vel_y && min_vel_theta == -max_vel_theta */
    std::vector<Velocity2D> velocities;

    std::vector<int> max_limits({((int)num_of_vel_x_samples)-1,
                                 ((int)num_of_vel_y_samples)-1,
                                 ((int)num_of_vel_theta_samples)-1});
    std::vector<int> min_limits(3, 0);
    std::vector<std::vector<int> > indexes = Utils::generateBFIndexes(max_limits, min_limits);

    std::array<float, 3> max_vel({max_vel_x, max_vel_y, max_vel_theta});
    std::array<float, 3> min_vel({min_vel_x, -max_vel_y, -max_vel_theta});
    std::array<bool, 3> quadratic_samples({quadratic_x_samples,
                                           quadratic_y_samples,
                                           quadratic_theta_samples});

    velocities.reserve(indexes.size());
    for ( size_t i = 0; i < indexes.size(); i++ )
    {
        std::vector<float> velocity;
        velocity.reserve(indexes[i].size());
        for ( size_t j = 0; j < indexes[i].size(); j++ )
        {
            float vel = 0.0f;
            if ( max_limits[j] != 0 )
            {
                if ( quadratic_samples[j] )
                {
                    float half_limit = ((float)max_limits[j]) / 2;
                    float sign = ( indexes[i][j] >= half_limit ) ? 1.0f : -1.0f;
                    float mid_vel = (max_vel[j] + min_vel[j])/2;
                    /* y = a * x^2 */
                    float a = (max_vel[j]-mid_vel)/(std::pow(half_limit, 2));
                    float norm_index = ((float)indexes[i][j]) - half_limit;
                    vel = mid_vel + (a * sign * std::pow(norm_index, 2));
                    vel = GCUtils::roundFloat(vel, 3);
                }
                else
                {
                    float delta_vel = (max_vel[j] - min_vel[j]) / max_limits[j];
                    vel = min_vel[j] + (indexes[i][j] * delta_vel);
                }

            }
            velocity.push_back(vel);
        }
        if ( !apply_elliptical_filter )
        {
            velocities.push_back(Velocity2D(velocity[0], velocity[1], velocity[2]));
        }
        else
        {
            float x_term = 0.0f;
            if ( velocity[0] > 0 && max_vel[0] != 0 )
            {
                 x_term = std::pow(velocity[0]/max_vel[0], 2);
            }
            else if ( velocity[0] < 0 && min_vel[0] != 0 )
            {
                 x_term = std::pow(velocity[0]/min_vel[0], 2);
            }
            float y_term = ( max_vel[1] != 0 )
                           ? std::pow(velocity[1]/max_vel[1], 2)
                           : 0.0f;
            float theta_term = ( max_vel[2] != 0 )
                               ? std::pow(velocity[2]/max_vel[2], 2)
                               : 0.0f;
            float elliptical_linear_vel_constraint = x_term + y_term;
            float elliptical_angular_vel_constraint = x_term + theta_term;
            if ( elliptical_linear_vel_constraint - 1.0f < 1e-3f &&
                 elliptical_angular_vel_constraint - 1.0f < 1e-3f )
            {
                velocities.push_back(Velocity2D(velocity[0], velocity[1], velocity[2]));
            }
        }
    }

    return velocities;
}

std::vector<float> InitialGuessUtils::getSampleTimes() const
{
    float control_time = trajectories_[0][0].t;
    std::vector<float> sample_times(trajectories_[0].size(), control_time);
    return sample_times;
}

void InitialGuessUtils::printVelocities() const
{
    for ( size_t i = 0; i < velocities_.size(); i++ )
    {
        std::cout << i << " " << velocities_[i] << std::endl;
    }
    std::cout << std::endl;
}

} // namespace cabin
