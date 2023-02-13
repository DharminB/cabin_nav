#include <geometry_msgs/Twist.h>

#include <geometry_common/Utils.h>
#include <geometry_common/Point2D.h>
#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/utils/utils.h>
#include <cabin_nav/mpc/model.h>

#include <cabin_nav/output/cmd_vel_output.h>
#include <cabin_nav/output/cmd_vel_output_data.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Polygon2D;
using kelo::geometry_common::XYTheta;
using kelo::geometry_common::Velocity2D;
using kelo::geometry_common::Acceleration2D;
using GCUtils = kelo::geometry_common::Utils;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool CmdVelOutput::configure(const YAML::Node& config)
{
    if ( !Parser::read<std::string>(config, "topic", topic_) ||
         !Parser::read<float>(config, "rate", rate_) )
    {
        std::cerr << Print::Err << Print::Time() << "[CmdVelOutput] "
                  << "topic and/or rate not provided"
                  << Print::End << std::endl;
        return false;
    }
    sample_time_ = 1.0f/rate_;
    queue_size_ = Parser::get<size_t>(config, "queue_size", 1);
    ideal_loop_duration_ = std::chrono::duration<float>(1.0f / rate_);
    robot_frame_ = Parser::get<std::string>(config, "robot_frame", "base_link");

    if ( !Parser::read<XYTheta>(config, "max_vel", max_vel_) ||
         !Parser::read<XYTheta>(config, "min_vel", min_vel_) ||
         !Parser::read<XYTheta>(config, "max_acc", max_acc_) ||
         !Parser::read<XYTheta>(config, "braking_acc", max_braking_acc_) ||
         !parseFootprint(config) )
    {
        return false;
    }
    min_acc_ = max_acc_ * -1.0f;
    min_braking_acc_ = max_braking_acc_ * -1.0f;

    nh_.param<bool>("dummy_mode", dummy_mode_, false);

    return true;
}

void CmdVelOutput::initializeOutputData(
        OutputData::Ptr& output_data) const
{
    output_data = std::make_shared<CmdVelOutputData>();
}

bool CmdVelOutput::setData(
        const OutputData::Ptr& output_data,
        const InputData::Map& input_data_map)
{
    std::lock_guard<std::mutex> guard(loop_thread_mutex_);

    if ( output_data->getType() != getType() )
    {
        std::cout << Print::Err << Print::Time() << "[CmdVelOutput] "
                  << "output_data's type is not \"" << getType() << "\"."
                  << Print::End << std::endl;
        return false;
    }

    CmdVelOutputData::ConstPtr cmd_vel_output_data =
        std::static_pointer_cast<const CmdVelOutputData>(output_data);

    if ( cmd_vel_output_data->trajectory.size() > 0 )
    {
        target_vel_ = cmd_vel_output_data->trajectory.front().vel;
        applySafetyConstraints(input_data_map);
        int sample_time_milli = 1000 * cmd_vel_output_data->trajectory.front().t;
        target_vel_time_ = std::chrono::steady_clock::now() +
                           std::chrono::milliseconds(sample_time_milli);
        traj_pub_.publish(Utils::convertToROSPath(
                cmd_vel_output_data->trajectory, robot_frame_));
    }
    else
    {
        target_vel_ = Velocity2D();
        target_vel_time_ = std::chrono::steady_clock::now();
        return false;
    }
    return true;
}

void CmdVelOutput::start()
{
    if ( is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[CmdVelOutput] "
                  << "Already active. Ignoring start() call."
                  << Print::End << std::endl;
        return;
    }

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(topic_, queue_size_);
    debug_target_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("debug_target_vel", 1);
    traj_pub_ = nh_.advertise<nav_msgs::Path>("trajectory", 1);
    footprint_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("footprint", 1);
    target_vel_time_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    is_active_ = true;
    loop_thread_ = std::thread(&CmdVelOutput::mainLoop, this);
}

void CmdVelOutput::stop()
{
    if ( !is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[CmdVelOutput] "
                  << "Not active yet. Ignoring stop() call."
                  << Print::End << std::endl;
        return;
    }

    is_active_ = false;
    if ( loop_thread_.joinable() )
    {
        loop_thread_.join();
    }
}

void CmdVelOutput::step()
{
    /* publish target velocities for debugging */
    geometry_msgs::Twist target_vel_msg;
    target_vel_msg.linear.x = target_vel_.x;
    target_vel_msg.linear.y = target_vel_.y;
    target_vel_msg.angular.z = target_vel_.theta;
    debug_target_vel_pub_.publish(target_vel_msg);

    footprint_msg_.header.stamp = ros::Time::now();
    footprint_pub_.publish(footprint_msg_);

    if ( dummy_mode_ )
    {
        return;
    }

    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

    float remaining_time = ( now > target_vel_time_ )
                           ? sample_time_
                           : std::chrono::duration<float>(target_vel_time_ - now).count();

    float delta_time = std::min(sample_time_, remaining_time);

    if ( now > target_vel_time_ + std::chrono::seconds(1) )
    {
        std::cerr << Print::Warn << Print::Time() << "[CmdVelOutput] "
                  << "Did not get target vel within expected duration."
                  << " Commanding zero vel." << Print::End << std::endl;
        target_vel_.x = 0.0f;
        target_vel_.y = 0.0f;
        target_vel_.theta = 0.0f;
    }

    /* calculate acc needed to reach target velocity */
    Acceleration2D target_acc = (target_vel_ - vel_) / remaining_time;

    /* apply acceleration limits */
    target_acc = ( use_braking_acc_ )
                 ? GCUtils::clip(target_acc, max_braking_acc_, min_braking_acc_)
                 : GCUtils::clip(target_acc, max_acc_, min_acc_);

    /* apply appropriate acceleration */
    vel_ = vel_ + (target_acc * delta_time);

    /* apply velocity limits */
    vel_ = GCUtils::applyVelLimits(vel_, max_vel_, min_vel_);

    /* avoid very small velocities */
    if ( std::fabs(vel_.x) < 1e-3f )
    {
        vel_.x = 0.0f;
    }
    if ( std::fabs(vel_.y) < 1e-3f )
    {
        vel_.y = 0.0f;
    }
    if ( std::fabs(vel_.theta) < 1e-3f )
    {
        vel_.theta = 0.0f;
    }

    // std::cout << vel_ << std::endl;

    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = vel_.x;
    cmd_vel_msg.linear.y = vel_.y;
    cmd_vel_msg.angular.z = vel_.theta;
    cmd_vel_pub_.publish(cmd_vel_msg);
}

void CmdVelOutput::applySafetyConstraints(const InputData::Map& input_data_map)
{
    if ( std::isnan(target_vel_.x) ||
         std::isnan(target_vel_.y) ||
         std::isnan(target_vel_.theta) )
    {
        std::cerr << Print::Warn << Print::Time() << "[CmdVelOutput] "
                  << "Target velocity has nan " << target_vel_
                  << ". Commanding zero vel."
                  << Print::End << std::endl;
        target_vel_ = Velocity2D();
        return;
    }

    Velocity2D safe_vel = GCUtils::applyVelLimits(target_vel_, max_vel_, min_vel_);
    if ( safe_vel != target_vel_ )
    {
        std::cerr << Print::Warn << Print::Time() << "[CmdVelOutput] "
                  << "Velocity reduced from " << target_vel_ << " to " << safe_vel 
                  << " to comply with velocity limits."
                  << Print::End << std::endl;
        target_vel_ = safe_vel;
    }

    /* calculate braking time */
    XYTheta braking_time;
    braking_time.x = std::fabs(safe_vel.x) / max_acc_.x;
    braking_time.y = std::fabs(safe_vel.y) / max_acc_.y;
    braking_time.theta = std::fabs(safe_vel.theta) / max_acc_.theta;
    float safe_braking_time = std::max(
            std::max(braking_time.x, braking_time.y), braking_time.theta);
    safe_braking_time *= 1.1f; // multiplying with 1.1 to add 10% safety buffer to combat with noise

    /* calculate trajectory with constant velocity */
    const float sample_time = 0.1f; // 100 ms
    TrajectoryPoint current;
    current.vel = safe_vel;
    size_t num_of_states = std::ceil(safe_braking_time / sample_time);
    std::vector<float> sample_times(num_of_states, sample_time);
    std::vector<float> u(num_of_states * 3, 0.0f);
    Trajectory trajectory = Model::calcTrajectory(
            current, u, sample_times);

    // /* check collision */
    // float time_to_collision = -1.0f;
    // int collision_index = Utils::calcCollisionIndex(
    //         trajectory, footprint_, input_data->laser_pts);
    // if ( collision_index >= 0 ) // unsafe
    // {
    //     time_to_collision = trajectory[collision_index].t;
    //     float conservative_ttc = std::max(time_to_collision - sample_time, 0.0f);
    //     float vel_scaling_factor = conservative_ttc / safe_braking_time;
    //     vel_scaling_factor = std::min(1.0f, vel_scaling_factor); // just to be extra safe
    //     safe_vel = safe_vel * vel_scaling_factor;
    // }

    if ( safe_vel != target_vel_ )
    {
        std::cerr << Print::Warn << Print::Time() << "[CmdVelOutput] "
                  << "Velocity reduced from " << target_vel_ << " to " << safe_vel
                  << " to avoid potential collision."
                  << Print::End << std::endl;
        target_vel_ = safe_vel;
        use_braking_acc_ = true;
    }
    else
    {
        use_braking_acc_ = false;
    }
}

bool CmdVelOutput::parseFootprint(const YAML::Node& config)
{
    Polygon2D raw_footprint;
    if ( !Parser::read<Polygon2D>(config, "footprint", raw_footprint) )
    {
        std::cerr << Print::Warn << Print::Time() << "[CmdVelOutput] "
                  << "Could not parse footprint"
                  << Print::End << std::endl;
        return false;
    }

    if ( raw_footprint.size() < 3 )
    {
        std::cerr << Print::Warn << Print::Time() << "[CmdVelOutput] "
                  << "Footprint does not form a polygon. Need atleast 3 points." << std::endl
                  << Print::End << std::endl;
        return false;
    }

    float footprint_padding = Parser::get<float>(config, "footprint_padding", 0.01f);
    footprint_ = raw_footprint.calcInflatedPolygon(footprint_padding);
    footprint_msg_ = footprint_.asPolygonStamped(robot_frame_);

    return true;
}

std::ostream& CmdVelOutput::write(std::ostream& out) const
{
    out << "<Output type: " << getType() << ">";
    return out;
}

} // namespace cabin
