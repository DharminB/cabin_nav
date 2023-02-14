#include <geometry_common/Circle.h>
#include <geometry_common/TransformMatrix2D.h>
#include <geometry_common/Utils.h>
#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/mpc/model.h>
#include <cabin_nav/utils/utils.h>
#include <cabin_nav/output/visualization_marker_output_data.h>
#include <cabin_nav/behavior/behavior.h>

using kelo::geometry_common::Box2D;
using kelo::geometry_common::Pose2D;
using kelo::geometry_common::Velocity2D;
using kelo::geometry_common::Acceleration2D;
using kelo::geometry_common::XYTheta;
using kelo::geometry_common::Point2D;
using kelo::geometry_common::Vector2D;
using kelo::geometry_common::PointCloud2D;
using kelo::geometry_common::TransformMatrix2D;
using kelo::geometry_common::Circle;
using GCUtils = kelo::geometry_common::Utils;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

Behavior::Behavior(const std::string& type):
    type_(type)
{
    behavior_name_marker_.header.frame_id = "robot";
    behavior_name_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    behavior_name_marker_.color.a = 1.0f;
    behavior_name_marker_.scale.z = 0.2;
    behavior_name_marker_.pose.position.x = -0.5;
    behavior_name_marker_.pose.orientation.w = 1.0;
}

const std::string& Behavior::getType() const
{
    return type_;
}

void Behavior::setName(const std::string& name)
{
    name_ = name;
    behavior_name_marker_.text = name_;
}

const std::string& Behavior::getName() const
{
    return name_;
}

float Behavior::getIdealLoopDuration() const
{
    return ( sample_times_.empty() ) ? 1.0f : sample_times_.front();
}

void Behavior::updateAction(std::shared_ptr<Action>& action)
{
}

const std::vector<std::string>& Behavior::getRequiredInputs() const
{
    return required_inputs_;
}

std::string Behavior::getRecoveryBehaviorName(const std::string& failure_code) const
{
    if ( failure_to_recovery_map_.find(failure_code) == failure_to_recovery_map_.end() )
    {
        std::cerr << Print::Err << Print::Time() << "[Behavior] "
                  << "failure_code " << failure_code
                  << " does not exist in failure_to_recovery_map."
                  << Print::End << std::endl;
        return "";
    }
    return failure_to_recovery_map_.at(failure_code);
}

void Behavior::setAccLimits(const Acceleration2D& max_acc)
{
    max_acc_ = max_acc;
}

void Behavior::setVelLimits(const Velocity2D& max_vel, const Velocity2D& min_vel)
{
    max_vel_ = max_vel;
    min_vel_ = min_vel;
}

bool Behavior::parseAccLimits(const YAML::Node& config)
{
    return ( Parser::hasKey(config, "control") &&
             Parser::read<XYTheta>(config["control"], "max_acc", max_acc_) );
}

bool Behavior::parseVelLimits(const YAML::Node& config)
{
    return ( Parser::hasKey(config, "control") &&
             Parser::read<XYTheta>(config["control"], "max_vel", max_vel_) &&
             Parser::read<XYTheta>(config["control"], "min_vel", min_vel_) );
}

bool Behavior::parseControlHorizon(const YAML::Node& config)
{
    if ( !Parser::hasKey(config, "control") ||
         !Parser::hasKey(config["control"], "control_horizon") )
    {
        return false;
    }

    if ( !config["control"]["control_horizon"].IsSequence() )
    {
        std::cerr << Print::Err << Print::Time() << "[Behavior] "
                  << "control_horizon_params is not a sequence"
                  << Print::End << std::endl;
        sample_times_.clear();
        sample_times_.push_back(1.0f);
        return false;
    }

    sample_times_.clear();
    float section_duration;
    size_t section_num_of_controls;
    for ( const YAML::Node& horizon_section_yaml : config["control"]["control_horizon"] )
    {
        if ( !Parser::read<float>(horizon_section_yaml, "duration", section_duration) ||
             !Parser::read<size_t>(horizon_section_yaml, "num_of_controls", section_num_of_controls) )
        {
            std::cerr << Print::Err << Print::Time() << "[Behavior] "
                      << "control_horizon_params contains a section with incorrect format."
                      << Print::End << std::endl;
            sample_times_.clear();
            sample_times_.push_back(1.0f);
            return false;
        }

        if ( section_duration < 0.01f )
        {
            std::cerr << Print::Err << Print::Time() << "[Behavior] "
                      << "control_horizon contains a section with too small "
                      << "duration " << section_duration
                      << Print::End << std::endl;
            sample_times_.clear();
            sample_times_.push_back(1.0f);
            return false;
        }
        if ( section_num_of_controls == 0 )
        {
            std::cerr << Print::Err << Print::Time() << "[Behavior] "
                      << "control_horizon contains a section with zero num_of_controls"
                      << Print::End << std::endl;
            sample_times_.clear();
            sample_times_.push_back(1.0f);
            return false;
        }

        for ( size_t i = 0; i < section_num_of_controls; i++ )
        {
            sample_times_.push_back(section_duration/section_num_of_controls);
        }
    }
    return true;
}

bool Behavior::parseAccLimitWeights(const YAML::Node& config)
{
    return ( Parser::hasKey(config, "control") &&
             Parser::hasKey(config["control"], "weights") &&
             Parser::read<XYTheta>(config["control"]["weights"], "acc_limit",
                                   acc_limit_weights_) );
}

bool Behavior::parseVelLimitWeights(const YAML::Node& config)
{
    return ( Parser::hasKey(config, "control") &&
             Parser::hasKey(config["control"], "weights") &&
             Parser::read<XYTheta>(config["control"]["weights"], "vel_limit",
                                   vel_limit_weights_) );
}

bool Behavior::parseGoalStateWeights(const YAML::Node& config)
{
    return ( Parser::hasKey(config, "control") &&
             Parser::hasKey(config["control"], "weights") &&
             Parser::hasKey(config["control"]["weights"], "goal_state") &&
             Parser::read<XYTheta>(config["control"]["weights"]["goal_state"],
                                   "pos", goal_state_pos_weights_) &&
             Parser::read<XYTheta>(config["control"]["weights"]["goal_state"],
                                   "vel", goal_state_vel_weights_) );
}

bool Behavior::parseFootprint(const YAML::Node& config)
{
    if ( !Parser::hasKey(config, "control") ||
         !Parser::hasKey(config["control"], "footprint") )
    {
        std::cerr << Print::Err << Print::Time() << "[Behavior] "
                  << "control does not contain \"footprint\""
                  << Print::End << std::endl;
        return false;
    }

    inflation_dist_ = Parser::get<float>(config["control"], "inflation_dist", 0.5f);
    float inflation_dist = inflation_dist_;
    if ( Parser::read<float>(config["control"], "min_inflation_dist",
                             min_inflation_dist_, false) &&
         Parser::read<float>(config["control"], "max_inflation_dist",
                             max_inflation_dist_, false) )
    {
        inflation_dist = min_inflation_dist_;
    }

    if ( Parser::read<Box2D>(
                config["control"], "footprint", box_footprint_, false) )
    {
        is_footprint_box_ = true;
        footprint_marker_ = box_footprint_.asPolygon2D().calcInflatedPolygon(
                inflation_dist).asMarker("robot", 0.66f, 0.33f, 1.0f, 1.0f, 0.01f, false);
    }
    else if ( Parser::read<Circle>(
                config["control"], "footprint", circle_footprint_, false) )
    {
        is_footprint_box_ = false;
        footprint_marker_ = circle_footprint_.asPolygon2D().calcInflatedPolygon(
                inflation_dist).asMarker("robot", 0.66f, 0.33f, 1.0f, 1.0f, 0.01f, false);
    }
    else
    {
        std::cerr << Print::Err << Print::Time() << "[Behavior] "
                  << "\"footprint\" is neither a circle nor a box."
                  << Print::End << std::endl;
        return false;
    }
    return true;
}

bool Behavior::parseFailureToRecoveryMap(const YAML::Node& config)
{
    if ( Parser::hasKey(config, "recovery", false) &&
         Parser::hasKey(config["recovery"], "failure_to_recovery_map", false) )
    {
        return ( Parser::read<std::map<std::string, std::string>>(
                    config["recovery"], "failure_to_recovery_map", failure_to_recovery_map_) );
    }
    return true; // if a behavior does not have recovery; don't fail
}

bool Behavior::parseRequiredInputs(const YAML::Node& config)
{
    std::map<std::string, std::string> required_inputs_map;
    if ( !Parser::read<std::map<std::string, std::string>>(
                 config, "required_inputs", required_inputs_map) )
    {
        return false;
    }
    required_inputs_map_ = Utils::convertMapToUnorderedMap(required_inputs_map);

    /* initialize required_inputs_ */
    required_inputs_.reserve(required_inputs_map.size());
    for ( auto itr = required_inputs_map.begin(); itr != required_inputs_map.end(); itr ++ )
    {
        required_inputs_.push_back(itr->second);
    }
    return true;
}

bool Behavior::parseOutputs(const YAML::Node& config)
{
    std::map<std::string, std::string> outputs_map;
    if ( !Parser::read<std::map<std::string, std::string>>(
                 config, "outputs", outputs_map) )
    {
        return false;
    }
    outputs_map_ = Utils::convertMapToUnorderedMap(outputs_map);
    return true;
}

float Behavior::calcAccLimitCost(const std::vector<float>& u, bool is_unicycle) const
{
    size_t dimensions = ( is_unicycle ) ? 2 : 3;
    if ( sample_times_.size() * dimensions != u.size() )
    {
        return 1e6f;
    }

    float cost = 0.0f;
    XYTheta dist;
    for ( size_t i = 0; i < sample_times_.size(); i++ )
    {
        if ( is_unicycle )
        {
            dist.x     = u[ i*2   ] - GCUtils::clip(u[ i*2   ], max_acc_.x,     -max_acc_.x);
            dist.theta = u[(i*2)+1] - GCUtils::clip(u[(i*2)+1], max_acc_.theta, -max_acc_.theta);
        }
        else
        {
            dist.x     = u[ i*3   ] - GCUtils::clip(u[ i*3   ], max_acc_.x,     -max_acc_.x);
            dist.y     = u[(i*3)+1] - GCUtils::clip(u[(i*3)+1], max_acc_.y,     -max_acc_.y);
            dist.theta = u[(i*3)+2] - GCUtils::clip(u[(i*3)+2], max_acc_.theta, -max_acc_.theta);
        }
        cost += acc_limit_weights_.x * dist.x * dist.x;
        cost += acc_limit_weights_.y * dist.y * dist.y;
        cost += acc_limit_weights_.theta * dist.theta * dist.theta;
    }
    return cost;
}

float Behavior::calcVelLimitCost(const Velocity2D& vel) const
{
    float cost = 0.0f;
    XYTheta diff = vel - GCUtils::clip(vel, max_vel_, min_vel_);
    cost += vel_limit_weights_.x * diff.x * diff.x;
    cost += vel_limit_weights_.y * diff.y * diff.y;
    cost += vel_limit_weights_.theta * diff.theta * diff.theta;
    return cost;
}

float Behavior::calcLaserPtsCostBoxFootprint(
        const PointCloud2D& laser_pts,
        const Pose2D& pose,
        float inflation_dist) const
{
    TransformMatrix2D tf(pose);
    tf.invert();

    if ( inflation_dist < 1e-3f )
    {
        inflation_dist = 1e-3f;
    }

    float cost = 0.0f;
    float clipped_dist = 0.0f;
    Vector2D dist;
    const float multiplier = 1.0f / std::pow(inflation_dist, 2);
    Point2D pt;
    for ( size_t i = 0; i < laser_pts.size(); i++ )
    {
        pt = tf * laser_pts[i];
        dist.x = std::max(pt.x - box_footprint_.max_x, box_footprint_.min_x - pt.x);
        dist.y = std::max(pt.y - box_footprint_.max_y, box_footprint_.min_y - pt.y);
        clipped_dist = inflation_dist - std::min(std::max(dist.x, dist.y), inflation_dist);
        cost += multiplier * clipped_dist * clipped_dist;
    }
    return cost;
}

float Behavior::calcLaserPtsCostCircleFootprint(
        const PointCloud2D& laser_pts,
        const Pose2D& pose,
        float inflation_dist) const
{
    Circle transformed_circle = TransformMatrix2D(pose) * circle_footprint_;

    if ( inflation_dist < 1e-3f )
    {
        inflation_dist = 1e-3f;
    }

    float cost = 0.0f;
    float dist = 0.0f;
    float clipped_dist = 0.0f;
    float inflated_radius = transformed_circle.r + inflation_dist;
    float multiplier = 1.0f / std::pow(inflation_dist, 2);
    for ( size_t i = 0; i < laser_pts.size(); i++ )
    {
        dist = transformed_circle.distTo(laser_pts[i]);
        clipped_dist = inflated_radius - std::min(dist, inflated_radius);
        cost += multiplier * clipped_dist * clipped_dist;
    }
    return cost;
}

float Behavior::calcLaserPtsCost(
        const PointCloud2D& laser_pts,
        const Pose2D& pose,
        float inflation_dist) const
{
    return ( is_footprint_box_ )
           ? calcLaserPtsCostBoxFootprint(laser_pts, pose, inflation_dist)
           : calcLaserPtsCostCircleFootprint(laser_pts, pose, inflation_dist);
}

float Behavior::calcLaserPtsCostVelBased(
        const PointCloud2D& laser_pts,
        const TrajectoryPoint& traj_pt,
        float min_inflation_dist,
        float max_inflation_dist) const
{
    float linear_vel_sq = std::pow(traj_pt.vel.x, 2) + std::pow(traj_pt.vel.y, 2);
    float inflation_dist = GCUtils::clip(linear_vel_sq/(max_acc_.x * 2), // braking dist
                                         max_inflation_dist,
                                         min_inflation_dist);
    return calcLaserPtsCost(laser_pts, traj_pt.pos, inflation_dist);
}

float Behavior::calcCirclesCostBoxFootprint(
        const std::vector<Circle>& obstacles,
        const Pose2D& pose,
        float inflation_dist) const
{
    TransformMatrix2D tf(pose);
    tf.invert();

    if ( inflation_dist < 1e-3f )
    {
        inflation_dist = 1e-3f;
    }

    float cost = 0.0f;
    float clipped_dist = 0.0f;
    Vector2D dist;
    const float multiplier = 1.0f / std::pow(inflation_dist, 2);
    Circle c;
    for ( size_t i = 0; i < obstacles.size(); i++ )
    {
        c = tf * obstacles[i];
        dist.x = std::max(c.x - box_footprint_.max_x, box_footprint_.min_x - c.x) - c.r;
        dist.y = std::max(c.y - box_footprint_.max_y, box_footprint_.min_y - c.y) - c.r;
        clipped_dist = inflation_dist - std::min(std::max(dist.x, dist.y), inflation_dist);
        cost += multiplier * clipped_dist * clipped_dist;
    }
    return cost;
}

float Behavior::calcCirclesCostCircleFootprint(
        const std::vector<Circle>& obstacles,
        const Pose2D& pose,
        float inflation_dist) const
{
    Circle transformed_circle = TransformMatrix2D(pose) * circle_footprint_;

    if ( inflation_dist < 1e-3f )
    {
        inflation_dist = 1e-3f;
    }

    float cost = 0.0f;
    float dist = 0.0f;
    float clipped_dist = 0.0f;
    float inflated_radius = transformed_circle.r + inflation_dist;
    float multiplier = 1.0f / std::pow(inflation_dist, 2);
    for ( size_t i = 0; i < obstacles.size(); i++ )
    {
        dist = transformed_circle.distTo(obstacles[i]) - obstacles[i].r;
        clipped_dist = inflated_radius - std::min(dist, inflated_radius);
        cost += multiplier * clipped_dist * clipped_dist;
    }
    return cost;
}

float Behavior::calcCirclesCost(
        const std::vector<Circle>& obstacles,
        const Pose2D& pose,
        float inflation_dist) const
{
    return ( is_footprint_box_ )
           ? calcCirclesCostBoxFootprint(obstacles, pose, inflation_dist)
           : calcCirclesCostCircleFootprint(obstacles, pose, inflation_dist);
}

float Behavior::calcCirclesCostVelBased(
        const std::vector<Circle>& obstacles,
        const TrajectoryPoint& traj_pt,
        float min_inflation_dist,
        float max_inflation_dist) const
{
    float linear_vel_sq = std::pow(traj_pt.vel.x, 2) + std::pow(traj_pt.vel.y, 2);
    float inflation_dist = GCUtils::clip(linear_vel_sq/(max_acc_.x * 2), // braking dist
                                         max_inflation_dist,
                                         min_inflation_dist);
    return calcCirclesCost(obstacles, traj_pt.pos, inflation_dist);
}

float Behavior::calcTrajectoryPointCost(
        const TrajectoryPoint& s1,
        const TrajectoryPoint& s2,
        const XYTheta& pos_weights,
        const XYTheta& vel_weights) const
{
    float cost = 0.0f;
    const TrajectoryPoint diff = s1 - s2;
    cost += pos_weights.x     * std::pow(diff.pos.x, 2);
    cost += pos_weights.y     * std::pow(diff.pos.y, 2);
    cost += pos_weights.theta * std::pow(diff.pos.theta, 2);
    cost += vel_weights.x     * std::pow(diff.vel.x, 2);
    cost += vel_weights.y     * std::pow(diff.vel.y, 2);
    cost += vel_weights.theta * std::pow(diff.vel.theta, 2);
    return cost;
}

float Behavior::calcGoalCost(
        const TrajectoryPoint& traj_pt,
        const TrajectoryPoint& goal) const
{
    return calcTrajectoryPointCost(
            traj_pt, goal, goal_state_pos_weights_, goal_state_vel_weights_);
}

Trajectory Behavior::calcRampedTrajectory(
        const Velocity2D& curr_vel, const Velocity2D& target_vel) const
{
    TrajectoryPoint current(Pose2D(), curr_vel, 0.0f);
    return calcRampedTrajectory(current, target_vel);
}

Trajectory Behavior::calcRampedTrajectory(
        const TrajectoryPoint& current,
        const Velocity2D& target_vel) const
{
    Velocity2D vel(current.vel);

    Acceleration2D req_acc, applied_acc;
    Acceleration2D min_acc = max_acc_ * -1.0f;
    std::vector<float> u(sample_times_.size() * 3);
    for ( size_t i = 0; i < sample_times_.size(); i++ )
    {
        /* calculate required acceleration */
        req_acc = (target_vel - vel) / sample_times_[i];

        /* clip to allowable acceleration */
        applied_acc = GCUtils::clip(req_acc, max_acc_, min_acc); 
        u[(i*3)  ] = applied_acc.x;
        u[(i*3)+1] = applied_acc.y;
        u[(i*3)+2] = applied_acc.theta;

        /* update velocity for next iteration */
        vel = vel + (applied_acc * sample_times_[i]);
    }
    return Model::calcTrajectory(current, u, sample_times_);
}

void Behavior::addInitialGuessTrajectoryMarkers(
        const Trajectory& traj,
        OutputData::Map& output_data_map) const
{
    VisualizationMarkerOutputData::addMarkers(output_data_map,
            outputs_map_.at("markers"), Utils::convertTrajectoryToMarkers(
            traj, "robot",
            0.0f, 0.5f, 0.5f, 1.0f, 0.01f, // line
            1.0f, 0.0f, 0.0f, 1.0f, 0.05f, 0.01f)); // arrow
}

} // namespace cabin
