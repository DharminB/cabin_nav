#include <chrono>

#include <geometry_common/Utils.h>
#include <yaml_common/Parser2.h>

#include <cabin_nav/mpc/optimiser.h>
#include <cabin_nav/mpc/model.h>
#include <cabin_nav/input/velocity_input_data.h>
#include <cabin_nav/input/localisation_input_data.h>
#include <cabin_nav/input/laser_input_data.h>
#include <cabin_nav/output/cmd_vel_output_data.h>
#include <cabin_nav/output/visualization_marker_output_data.h>
#include <cabin_nav/structs/context_data.h>
#include <cabin_nav/action/goto_action.h>
#include <cabin_nav/behavior/ptp_behavior.h>

using kelo::geometry_common::Path;
using kelo::geometry_common::Circle;
using kelo::geometry_common::Pose2D;
using kelo::geometry_common::PointCloud2D;
using kelo::geometry_common::XYTheta;
using kelo::geometry_common::Velocity2D;
using kelo::geometry_common::Acceleration2D;
using kelo::geometry_common::TransformMatrix2D;
using GCUtils = kelo::geometry_common::Utils;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool PTPBehavior::configure(const YAML::Node& config)
{
    if ( !parseControlHorizon(config) ||
         !parseAccLimits(config) ||
         !parseVelLimits(config) ||
         !parseAccLimitWeights(config) ||
         !parseVelLimitWeights(config) ||
         !parseGoalStateWeights(config) ||
         !parseNormalStateWeights(config) ||
         !parseFootprint(config) ||
         !parseRequiredInputs(config) ||
         !parseOutputs(config) )
    {
        return false;
    }
    const YAML::Node& control_config = config["control"];
    use_dynamic_obstacles_ = Parser::get<bool>(
            control_config, "use_dynamic_obstacles", false);
    is_unicycle_ = Parser::get<bool>(
            control_config, "is_unicycle", false);
    weight_laser_pts_repulsion_ = Parser::get<float>(
            control_config["weights"], "laser_pts_repulsion", 0.0f);
    weight_goal_state_intercept_angle_ = Parser::get<float>(
            control_config["weights"], "goal_state_intercept_angle", 0.0f);
    shorten_control_horizon_ = Parser::get<bool>(
            control_config, "shorten_control_horizon", true);

    if ( Parser::hasKey(control_config, "geometric_planner", false) )
    {
        geometric_planner_.configure(control_config["geometric_planner"]);
        geometric_planner_.setFootprint(box_footprint_, circle_footprint_, is_footprint_box_);
        geometric_planner_goal_tolerance_linear_ = Parser::get<float>(
                control_config["geometric_planner"], "goal_tolerance_linear", 0.2f);
        geometric_planner_goal_tolerance_angular_ = Parser::get<float>(
                control_config["geometric_planner"], "goal_tolerance_angular", 1.0f);
        use_geometric_planner_ = true;
    }
    else
    {
        use_geometric_planner_ = false;
    }
    min_inflation_dist_ = Parser::get<float>(
            control_config, "min_inflation_dist", 0.1f);
    max_inflation_dist_ = Parser::get<float>(
            control_config, "max_inflation_dist", 10.0f);
    reverse_motion_goal_threshold_linear_ = Parser::get<float>(
            control_config, "reverse_motion_goal_threshold_linear", 0.2f);
    reverse_motion_goal_threshold_heading_ = Parser::get<float>(
            control_config, "reverse_motion_goal_threshold_heading", M_PI/2);

    parseFailureToRecoveryMap(config);

    if ( is_unicycle_ )
    {
        max_vel_.y = 0.0f;
        min_vel_.y = 0.0f;
    }

    cf_ = std::bind(&PTPBehavior::calcCost, this, std::placeholders::_1);

    if ( Parser::hasKey(control_config, "initial_guess", false) )
    {
        initial_guess_utils_.configure(control_config["initial_guess"],
                max_vel_, min_vel_, max_acc_, sample_times_, is_unicycle_);
        initial_guess_utils_.setCostFunction(cf_);
        use_initial_guess_utils_ = true;
        initial_guess_sample_times_ = initial_guess_utils_.getSampleTimes();
    }
    else
    {
        use_initial_guess_utils_ = false;
    }

    u_ = ( is_unicycle_ )
         ? std::vector<float>(2 * sample_times_.size(), 0.0f)
         : std::vector<float>(3 * sample_times_.size(), 0.0f);

    if ( use_dynamic_obstacles_ )
    {
        obstacle_tracker_.configure(sample_times_[0]);
    }

    float control_horizon_time = 0.0f;
    for ( float sample_time : sample_times_ )
    {
        control_horizon_time += sample_time;
    }
    robot_to_goal_dist_ = max_vel_.x * control_horizon_time;
    return true;
}

void PTPBehavior::reset()
{
    goal_ = TrajectoryPoint();
    current_.vel = Velocity2D();
    shortened_control_horizon_size_ = 1;
    std::fill(u_.begin(), u_.end(), 0.0f);
    initial_guess_utils_.reset();
    initial_guess_trajectory_.clear();
    laser_pts_.clear();

    /* moving obstacle */
    future_obstacles_.clear();
    future_initial_guess_obstacles_.clear();
    obstacle_tracker_.reset();

    geometric_planner_.reset();
}

void PTPBehavior::runOnce(const ContextData& context_data, BehaviorFeedback& fb)
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    VisualizationMarkerOutputData::addMarker(fb.output_data_map,
            outputs_map_.at("markers"), behavior_name_marker_);
    VisualizationMarkerOutputData::addMarkers(fb.output_data_map,
            outputs_map_.at("markers"), context_data.getPlanMarkers());
    VisualizationMarkerOutputData::addMarker(fb.output_data_map,
            outputs_map_.at("markers"), footprint_marker_);

    context_data_ = &context_data;

    if ( !VelocityInputData::getVelocity(context_data.input_data_map,
             required_inputs_map_.at("velocity"), current_.vel) ||
         !LocalisationInputData::getLocalisationTF(context_data.input_data_map,
             required_inputs_map_.at("localisation"), localisation_tf_) ||
         !LaserInputData::getLaserPts(context_data.input_data_map,
             required_inputs_map_.at("laser"), laser_pts_) )
    {
        std::cout << Print::Err << Print::Time() << "[PTPBehavior] "
                  << "Could not get velocity and/or localization tf."
                  << Print::End << std::endl;
        CmdVelOutputData::setTrajectory(
                fb.output_data_map, calcRampedTrajectory(current_.vel));
        fb.success = false;
        return;
    }

    if ( use_dynamic_obstacles_ )
    {
        std::vector<visualization_msgs::Marker> markers;
        std::vector<MovingCircle> moving_obstacles;
        obstacle_tracker_.calcObstacles(
                localisation_tf_.asPose2D(), laser_pts_,
                markers, moving_obstacles, static_pts_);
        obstacle_tracker_.calcFutureObstacles(
                initial_guess_sample_times_, moving_obstacles,
                future_initial_guess_obstacles_);
        obstacle_tracker_.calcFutureObstacles(
                sample_times_, moving_obstacles, future_obstacles_);
        VisualizationMarkerOutputData::addMarkers(fb.output_data_map,
                outputs_map_.at("markers"), markers);
    }

    goal_ = calcGoal(context_data, fb);

    std::vector<visualization_msgs::Marker> markers;
    planGeometricPath(markers);
    VisualizationMarkerOutputData::addMarkers(fb.output_data_map,
            outputs_map_.at("markers"), markers);
    calcInitialU(u_, fb);

    std::chrono::duration<float> init_u_time_duration = std::chrono::steady_clock::now() - start_time;
    float remaining_time = sample_times_.front() - init_u_time_duration.count();

    float time_threshold = remaining_time * 0.8f;

    Optimiser::conjugateGradientDescent(time_threshold, cf_, u_);
    Trajectory traj = Model::calcTrajectory(current_, u_, sample_times_, is_unicycle_);
    CmdVelOutputData::setTrajectory(fb.output_data_map, traj);

    std::chrono::duration<float> time_taken = std::chrono::steady_clock::now() - start_time;
    // std::cout << time_taken.count() * 1000.0f << " ms" << std::endl;

    VisualizationMarkerOutputData::addMarker(fb.output_data_map,
            outputs_map_.at("markers"), goal_.pos.asMarker("robot", 0.0f, 1.0f, 0.0f));

    fb.success = true;
}

void PTPBehavior::calcInitialU(std::vector<float>& u, BehaviorFeedback& fb)
{
    if ( use_initial_guess_utils_ )
    {
        initial_guess_utils_.calcInitialGuessIntermittently(
                current_, u, initial_guess_trajectory_);
        addInitialGuessTrajectoryMarkers(
                initial_guess_trajectory_, fb.output_data_map);
    }
    else
    {
        /* apply no control */
        std::fill(u.begin(), u.end(), 0.0f);
    }

    if ( !shorten_control_horizon_ )
    {
        return;
    }

    /* calc hueristic time required to reach goal state */
    float linear_dist = goal_.pos.position().magnitude();
    float linear_dist_est_time  = linear_dist / max_vel_.x;
    float angular_dist_est_time  = std::fabs(goal_.pos.theta) / max_vel_.theta;
    float max_est_time = std::max(linear_dist_est_time, angular_dist_est_time);
    float control_horizon_time = 0.0f;
    size_t i = 0;
    for ( i = 0; i < sample_times_.size(); i++ )
    {
        control_horizon_time += sample_times_[i];
        if ( control_horizon_time > max_est_time )
        {
            break;
        }
    }
    shortened_control_horizon_size_ = sample_times_.size() - i + 1;
}

float PTPBehavior::calcCost(const std::vector<float>& u)
{
    // generate trajectory using model
    Trajectory trajectory = Model::calcTrajectory(
            current_, u, sample_times_, is_unicycle_);

    float cost = 0.0f;

    /* acc limit soft constraints */
    cost += calcAccLimitCost(u, is_unicycle_);

    for ( size_t i = 0; i < trajectory.size(); i++ )
    {
        /* CONSTRAINTS */
        if ( use_dynamic_obstacles_ )
        {
            cost += weight_laser_pts_repulsion_ *
                    calcCirclesCostVelBased(
                            future_obstacles_[i], trajectory[i],
                            min_inflation_dist_, max_inflation_dist_);
            cost += weight_laser_pts_repulsion_ *
                    calcLaserPtsCostVelBased(
                            static_pts_, trajectory[i],
                            min_inflation_dist_, max_inflation_dist_);
        }
        else
        {
            cost += weight_laser_pts_repulsion_ *
                    calcLaserPtsCostVelBased(
                            laser_pts_, trajectory[i],
                            min_inflation_dist_, max_inflation_dist_);
        }

        /* vel limit soft constraints */
        cost += calcVelLimitCost(trajectory[i].vel);

        cost += calcTrajectoryPointCost(trajectory[i], goal_,
                normal_state_pos_weights_, normal_state_vel_weights_);
    }

    /* GOAL */
    for ( size_t i = trajectory.size()-shortened_control_horizon_size_;
          i < trajectory.size();
          i++ )
    {
        cost += calcGoalCost(trajectory[i], goal_);
        float angle_to_goal = (goal_.pos.position() - trajectory[i].pos.position()).angle();
        float angular_dist = GCUtils::calcShortestAngle(
                angle_to_goal, goal_.pos.theta);
        cost += weight_goal_state_intercept_angle_ * std::pow(angular_dist, 2);
    }

    return cost;
}

bool PTPBehavior::preConditionSatisfied(
        const ContextData& context_data, Action::Ptr action) const
{
    if ( context_data.isPlanValid() &&
         context_data.plan[context_data.plan_index]->getType() == "goto" )
    {
        const GoToAction::Ptr goto_action = std::static_pointer_cast<GoToAction>(action);
        if ( goto_action->isDuringTransition() )
        {
            return true;
        }
    }
    return false;
}

bool PTPBehavior::postConditionSatisfied(const ContextData& context_data) const
{
    return true;
}

TrajectoryPoint PTPBehavior::calcGoal(
        const ContextData& context_data, BehaviorFeedback& fb)
{
    const GoToAction::Ptr goto_action = std::dynamic_pointer_cast<GoToAction>(
             context_data.plan[context_data.plan_index]);
    if ( goto_action == nullptr )
    {
        return TrajectoryPoint();
    }

    /* calculate goal in local frame */
    TransformMatrix2D tf = localisation_tf_.calcInverse();
    return tf * goto_action->getIntermediateGoal();
}

void PTPBehavior::planGeometricPath(std::vector<visualization_msgs::Marker>& markers)
{
    if ( !use_geometric_planner_ )
    {
        return;
    }
    TrajectoryPoint start;

    /* strategy for not moving backwards (since the laser cannot see behind) */
    float dist_to_goal = start.pos.distTo(goal_.pos);
    float angle_to_goal = goal_.pos.position().angle();
    float angular_dist = GCUtils::calcShortestAngle(angle_to_goal, goal_.pos.theta);
    if ( dist_to_goal > reverse_motion_goal_threshold_linear_ &&
         std::fabs(angular_dist) > reverse_motion_goal_threshold_heading_ )
    {
        /* rotate in place */
        // float angular_offset = (( angular_dist > 0 ) ? -1 : 1) * reverse_motion_goal_threshold_heading_;
        // goal_.theta = angle_to_goal + angular_offset;

        /* ackermann drive */
        TrajectoryPoint goal_state = goal_;
        float turning_radius = reverse_motion_goal_threshold_linear_;
        goal_state.vel.theta = ( angular_dist < 0 ) ? min_vel_.theta/2 : max_vel_.theta/2;
        goal_state.vel.x = -turning_radius * std::fabs(goal_state.vel.theta);
        TrajectoryPoint ns = Model::predictNextTrajectoryPoint(goal_state,
                std::fabs(angular_dist/goal_state.vel.theta), Acceleration2D());
        goal_ = ns;
        // goal_.vel_x *= -1;
        goal_.vel.x = 0.0f;
    }

    if ( dist_to_goal < geometric_planner_goal_tolerance_linear_ &&
         std::fabs(goal_.pos.theta) < geometric_planner_goal_tolerance_angular_ )
    {
        return;
    }

    Path geometric_path;
    if ( geometric_planner_.plan(start.pos, goal_.pos,
                laser_pts_, geometric_path, markers) )
    {
        markers.push_back(GCUtils::convertGeometricPathToMarker(
                    geometric_path, "robot", 0.0f, 0.33f, 0.0f, 1.0f, 0.02f));

        int i = GeometricPlanner::calcIndexAtDist(geometric_path, robot_to_goal_dist_);
        if ( i < 0 )
        {
            return;
        }

        if ( geometric_planner_use_goal_theta_ && !is_unicycle_ )
        {
            goal_.pos.x = geometric_path[i].x;
            goal_.pos.y = geometric_path[i].y;
        }
        else
        {
            goal_.pos = geometric_path[i];
        }
    }
}

bool PTPBehavior::parseNormalStateWeights(const YAML::Node& config)
{
    if ( Parser::hasKey(config, "control") &&
         Parser::hasKey(config["control"], "weights") &&
         Parser::hasKey(config["control"]["weights"], "normal_state", false) )
    {
        return ( Parser::read<XYTheta>(config["control"]["weights"]["normal_state"],
                    "pos", normal_state_pos_weights_) &&
                 Parser::read<XYTheta>(config["control"]["weights"]["normal_state"],
                     "vel", normal_state_vel_weights_) );
    }
    return true; // if it doesn't have normal state weights, don't fail
}

} // namespace cabin
