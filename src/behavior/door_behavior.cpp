#include <cmath>
#include <chrono>

#include <geometry_common/Utils.h>
#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/structs/context_data.h>
#include <cabin_nav/mpc/optimiser.h>
#include <cabin_nav/mpc/model.h>
#include <cabin_nav/input/velocity_input_data.h>
#include <cabin_nav/input/localisation_input_data.h>
#include <cabin_nav/input/laser_input_data.h>
#include <cabin_nav/output/cmd_vel_output_data.h>
#include <cabin_nav/output/visualization_marker_output_data.h>
#include <cabin_nav/action/goto_action.h>
#include <cabin_nav/behavior/door_behavior.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Vector2D;
using kelo::geometry_common::PointVec2D;
using kelo::geometry_common::PointCloud2D;
using kelo::geometry_common::Polygon2D;
using kelo::geometry_common::Pose2D;
using kelo::geometry_common::Path;
using kelo::geometry_common::Velocity2D;
using kelo::geometry_common::LineSegment2D;
using kelo::geometry_common::TransformMatrix2D;
using GCUtils = kelo::geometry_common::Utils;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool DoorBehavior::configure(const YAML::Node& config)
{
    if ( !parseControlHorizon(config) ||
         !parseAccLimits(config) ||
         !parseVelLimits(config) ||
         !parseAccLimitWeights(config) ||
         !parseVelLimitWeights(config) ||
         !parseGoalStateWeights(config) ||
         !parseFootprint(config) ||
         !parseRequiredInputs(config) ||
         !parseOutputs(config) )
    {
        return false;
    }
    const YAML::Node& control_config = config["control"];
    weight_ideal_path_perp_dist_ = Parser::get<float>(
            control_config["weights"], "ideal_path_perp_dist", 0.0f);
    weight_laser_pts_repulsion_ = Parser::get<float>(
            control_config["weights"], "laser_pts_repulsion", 0.0f);
    inflation_dist_ = Parser::get<float>(
            control_config, "inflation_dist", 0.05f);

    if ( Parser::hasKey(config, "perception") )
    {
        ideal_path_length_after_door_ = Parser::get<float>(
                config["perception"], "ideal_path_length_after_door", 2.0f);
        avg_pts_dist_threshold_ = Parser::get<float>(
                config["perception"], "avg_pt_dist_threshold", 0.1f);
        buffer_size_ = Parser::get<size_t>(
                config["perception"], "door_frame_buffer_size", 5);
    }

    if ( Parser::hasKey(config, "transition_condition") )
    {
        if ( Parser::hasKey(config["transition_condition"], "pre") )
        {
            pre_door_goal_dist_ = Parser::get<float>(
                    config["transition_condition"]["pre"],
                    "door_goal_dist", 1.5f);
            pre_door_min_dist_ = Parser::get<float>(
                    config["transition_condition"]["pre"],
                    "door_min_dist", 1.0f);
            pre_door_max_dist_ = Parser::get<float>(
                    config["transition_condition"]["pre"],
                    "door_max_dist", 2.5f);
            goal_tolerance_heading_ = Parser::get<float>(
                    config["transition_condition"]["pre"],
                    "goal_tolerance_heading", 1.0f);
            goal_tolerance_angular_ = Parser::get<float>(
                    config["transition_condition"]["pre"],
                    "goal_tolerance_angular", 0.8f);
        }
        if ( Parser::hasKey(config["transition_condition"], "post") )
        {
            post_door_min_dist_x_ = Parser::get<float>(
                    config["transition_condition"]["post"],
                    "door_min_dist_x", 0.4f);
        }
    }

    if ( Parser::hasKey(config, "recovery") )
    {
        door_mismatch_threshold_ = Parser::get<float>(
                config["recovery"], "door_mismatch_threshold", 0.3f);
        retry_threshold_ = Parser::get<size_t>(config["recovery"], "retry_threshold", 0);
        parseFailureToRecoveryMap(config);
    }

    cf_ = std::bind(&DoorBehavior::calcCost, this, std::placeholders::_1);

    if ( Parser::hasKey(control_config, "initial_guess") )
    {
        initial_guess_utils_.configure(control_config["initial_guess"],
                max_vel_, min_vel_, max_acc_, sample_times_);
        initial_guess_utils_.setCostFunction(cf_);
        use_initial_guess_utils_ = true;
    }
    else
    {
        use_initial_guess_utils_ = false;
    }

    goal_.vel.x = max_vel_.x;
    u_ = std::vector<float>(3 * sample_times_.size(), 0.0f);
    return true;
}

void DoorBehavior::reset()
{
    door_buffer_.clear();
    std::fill(u_.begin(), u_.end(), 0.0f);
    door_ = LineSegment2D();
    ideal_path_ = LineSegment2D();
    initial_guess_trajectory_.clear();
    initial_guess_utils_.reset();
    retry_counter_ = 0;
    current_.vel = Velocity2D();
    laser_pts_.clear();
}

void DoorBehavior::runOnce(const ContextData& context_data, BehaviorFeedback& fb)
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
        std::cout << Print::Err << Print::Time() << "[DoorBehavior] "
                  << "Could not get velocity, localization tf and/or laser."
                  << Print::End << std::endl;
        CmdVelOutputData::setTrajectory(
                fb.output_data_map, calcRampedTrajectory(current_.vel));
        fb.success = false;
        return;
    }

    if ( !findDoorFrameMap() )
    {
        std::cerr << Print::Err << Print::Time() << "[DoorBehavior] "
                  << "Could not find door frame from semantic map"
                  << Print::End << std::endl;
        fb.success = false;
        return;
    }

    if ( !updateDoorFrame() )
    {
        if ( std::fabs(door_.start.angle()) < M_PI/2 ||
             std::fabs(door_.end.angle()) < M_PI/2 )
        {
            std::cerr << Print::Warn << Print::Time() << "[DoorBehavior] "
                      << "Could not update door frame based on sensor data"
                      << Print::End << std::endl;
            failWithRetry(fb);
            return;
        }
    }

    calcAveragedDoorFrame();

    /* calculate ideal path from door frame */
    float door_m = door_.slope();
    float door_c = door_.constant();
    Point2D door_center = door_.center();
    float ideal_path_m, ideal_path_c;
    GCUtils::findPerpendicularLineAt(door_m, door_c, door_center, ideal_path_m, ideal_path_c);
    ideal_path_.start.x = 0.0f;
    ideal_path_.start.y = (ideal_path_m * ideal_path_.start.x) + ideal_path_c;
    ideal_path_.end.x = door_center.x + ideal_path_length_after_door_;
    ideal_path_.end.y = (ideal_path_m * ideal_path_.end.x) + ideal_path_c;
    goal_.pos.theta = ideal_path_.angle();

    calcInitialU(u_, fb);

    std::chrono::duration<float> init_u_time_duration = std::chrono::steady_clock::now() - start_time;
    float remaining_time = sample_times_.front() - init_u_time_duration.count();

    float time_threshold = remaining_time * 0.8f;

    Optimiser::conjugateGradientDescent(time_threshold, cf_, u_);
    Trajectory traj = Model::calcTrajectory(current_, u_, sample_times_);
    CmdVelOutputData::setTrajectory(fb.output_data_map, traj);

    std::chrono::duration<float> time_taken = std::chrono::steady_clock::now() - start_time;
    // std::cout << time_taken.count() * 1000.0f << " ms" << std::endl;

    /* visualize features using markers */
    VisualizationMarkerOutputData::addMarker(fb.output_data_map,
            outputs_map_.at("markers"), door_.asMarker(
                "robot", 0.0f, 0.0f, 1.0f, 1.0f, 0.07f));
    VisualizationMarkerOutputData::addMarker(fb.output_data_map,
            outputs_map_.at("markers"), ideal_path_.asMarker(
                "robot", 0.0f, 0.33f, 0.0f, 1.0f, 0.05f));

    retry_counter_ = 0;
    fb.success = true;
}

void DoorBehavior::calcInitialU(std::vector<float>& u, BehaviorFeedback& fb)
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
}

float DoorBehavior::calcCost(const std::vector<float>& u)
{
    // generate trajectory using model
    Trajectory trajectory = Model::calcTrajectory(current_, u, sample_times_);

    float cost = 0.0f;

    /* acc limit soft constraints */
    cost += calcAccLimitCost(u);

    for ( const TrajectoryPoint& tp : trajectory )
    {
        /* CONSTRAINTS */
        cost += weight_laser_pts_repulsion_ *
                calcLaserPtsCost(laser_pts_, tp.pos, inflation_dist_);

        /* vel limit soft constraints */
        cost += calcVelLimitCost(tp.vel);

        cost += weight_ideal_path_perp_dist_ * ideal_path_.squaredMinDistTo(tp.pos.position());
        cost += calcGoalCost(tp, goal_);
    }

    return cost;
}

bool DoorBehavior::preConditionSatisfied(
        const ContextData& context_data, std::shared_ptr<Action> action) const
{
    if ( !context_data.isPlanValid() ||
         context_data.plan[context_data.plan_index]->getType() != "goto" )
    {
        return false;
    }

    TransformMatrix2D loc_tf;
    if ( !LocalisationInputData::getLocalisationTF(context_data.input_data_map,
                required_inputs_map_.at("localisation"), loc_tf) )
    {
        std::cout << Print::Err << Print::Time() << "[DoorBehavior] "
                  << "Could not get localisation tf"
                  << Print::End << std::endl;
        return false;
    }

    GoToAction::Ptr goto_action = std::static_pointer_cast<GoToAction>(action);
    const std::vector<Area::ConstPtr>& goto_plan = goto_action->getGoToPlan();
    size_t goto_plan_index = goto_action->getGoToPlanIndex();
    const Area::ConstPtr& door_area = goto_plan[goto_plan_index];
    const Area::ConstPtr& prev_area = goto_plan[goto_plan_index-1];
    const Pose2D robot_pose = loc_tf.asPose2D();
    Point2D robot_pt = robot_pose.position();

    // if door is first area; use robot_pt otherwise use prev area center
    Point2D prev_pt = ( goto_plan_index == 0 ) ? robot_pt : prev_area->getCenter();

    const Polygon2D& door_polygon = door_area->getPolygon();
    LineSegment2D door(door_polygon[0], door_polygon[1]);
    Pose2D pre_door_goal = Utils::calcIntermediateDoorGoal(
            door, prev_pt, true, pre_door_goal_dist_);
    Pose2D door_entry_pose(door_area->getCenter(), pre_door_goal.theta);

    float dist_to_door = door_entry_pose.distTo(robot_pt);
    float angle_diff = GCUtils::calcShortestAngle(door_entry_pose.theta, robot_pose.theta);
    float angle_to_goal = (door_entry_pose.position() - robot_pt).angle();
    float angle_to_goal_diff = GCUtils::calcShortestAngle(door_entry_pose.theta,
                                                         angle_to_goal);
    float goal_tolerance_heading_at_dist =
        goal_tolerance_heading_ * (dist_to_door / pre_door_max_dist_);

    if ( std::fabs(angle_diff) < goal_tolerance_angular_ &&
         std::fabs(angle_to_goal_diff) < goal_tolerance_heading_at_dist &&
         dist_to_door < pre_door_max_dist_ &&
         dist_to_door > pre_door_min_dist_ )
    {
        return true;
    }
    else
    {
        goto_action->setIntermediateGoal(TrajectoryPoint(pre_door_goal));
        goto_action->enableIsDuringTransition();
        return false;
    }
}

bool DoorBehavior::postConditionSatisfied(const ContextData& context_data) const
{
    if ( !context_data.isPlanValid() ||
         context_data.plan[context_data.plan_index]->getType() != "goto" )
    {
        return false;
    }

    TransformMatrix2D loc_tf;
    if ( !LocalisationInputData::getLocalisationTF(context_data.input_data_map,
                required_inputs_map_.at("localisation"), loc_tf) )
    {
        std::cout << Print::Err << Print::Time() << "[DoorBehavior] "
                  << "Could not get localisation tf"
                  << Print::End << std::endl;
        return false;
    }
    const GoToAction::Ptr goto_action = std::static_pointer_cast<GoToAction>(
             context_data.plan[context_data.plan_index]);
    const std::vector<Area::ConstPtr>& goto_plan = goto_action->getGoToPlan();
    size_t goto_plan_index = goto_action->getGoToPlanIndex();
    const Area::ConstPtr& door_area = goto_plan[goto_plan_index];
    const Polygon2D& door_polygon = door_area->getPolygon();
    LineSegment2D door(door_polygon[0], door_polygon[1]);

    TransformMatrix2D robot_to_global_tf = loc_tf.calcInverse();
    LineSegment2D door_local = robot_to_global_tf * door;

    return ( door_local.start.x < 0 &&
             door_local.end.x < 0 &&
             door_local.center().x < -post_door_min_dist_x_ );
}

bool DoorBehavior::findDoorFrameMap()
{
    const GoToAction::Ptr goto_action = std::static_pointer_cast<GoToAction>(
             context_data_->plan[context_data_->plan_index]);
    TransformMatrix2D robot_to_global_tf = localisation_tf_.calcInverse();

    const std::vector<Area::ConstPtr>& goto_plan = goto_action->getGoToPlan();
    size_t goto_plan_index = goto_action->getGoToPlanIndex();
    const Area::ConstPtr& door_area = goto_plan[goto_plan_index];

    if ( goto_plan.empty() || door_area->getType() != "door" )
    {
        return false;
    }
    const Polygon2D& door_polygon = door_area->getPolygon();
    LineSegment2D door(door_polygon[0], door_polygon[1]);
    door_ = robot_to_global_tf * door;
    return true;
}

bool DoorBehavior::updateDoorFrame()
{
    float half_door_size = door_.length() / 2;
    Point2D door_left = ( door_.start.angle() > door_.end.angle() ) ? door_.start : door_.end;
    Point2D door_right = ( door_.start.angle() > door_.end.angle() ) ? door_.end : door_.start;

    PointCloud2D left_pts, right_pts;
    for ( const Point2D& pt : laser_pts_ )
    {
        if ( pt.distTo(door_left) < half_door_size )
        {
            left_pts.push_back(pt);
        }
        if ( pt.distTo(door_right) < half_door_size )
        {
            right_pts.push_back(pt);
        }
    }
    if ( left_pts.empty() || right_pts.empty() )
    {
        return false;
    }

    /* update door line segment to the smallest line segment in their vicinity */
    float min_dist = std::numeric_limits<float>::max();
    for ( const Point2D& left_pt : left_pts )
    {
        for ( const Point2D& right_pt : right_pts )
        {
            float dist = left_pt.distTo(right_pt);
            if ( dist < min_dist )
            {
                min_dist = dist;
                door_left = left_pt;
                door_right = right_pt;
            }
        }
    }

    /* average points in vicinity for reduce noise */
    PointCloud2D left_avg_pts, right_avg_pts;
    left_avg_pts.reserve(left_pts.size());
    right_avg_pts.reserve(right_pts.size());
    for ( const Point2D& pt : left_pts )
    {
        if ( pt.distTo(door_left) < avg_pts_dist_threshold_ )
        {
            left_avg_pts.push_back(pt);
        }
    }
    for ( const Point2D& pt : right_pts )
    {
        if ( pt.distTo(door_right) < avg_pts_dist_threshold_ )
        {
            right_avg_pts.push_back(pt);
        }
    }

    if ( !left_avg_pts.empty() )
    {
        door_left = GCUtils::calcMeanPoint(left_avg_pts);
    }
    if ( !right_avg_pts.empty() )
    {
        door_right = GCUtils::calcMeanPoint(right_avg_pts);
    }
    LineSegment2D updated_door(door_left, door_right);

    if ( std::fabs(updated_door.length()/door_.length() - 1.0f) < door_mismatch_threshold_ )
    {
        door_ = updated_door;
        return true;
    }
    else
    {
        return false;
    }
}

void DoorBehavior::calcAveragedDoorFrame()
{
    /* add current door frame to buffer */
    door_buffer_.push_back(door_);
    if ( door_buffer_.size() > buffer_size_ )
    {
        door_buffer_.pop_front();
    }

    /* average start and end points of doors in door buffer */
    LineSegment2D avg_door;
    for ( const LineSegment2D& door : door_buffer_ )
    {
        avg_door.start = avg_door.start + door.start;
        avg_door.end = avg_door.end + door.end;
    }
    float buffer_size_inv = 1.0f / door_buffer_.size();
    avg_door.start = avg_door.start * buffer_size_inv;
    avg_door.end = avg_door.end * buffer_size_inv;
    door_ = avg_door;
}

void DoorBehavior::failWithRetry(BehaviorFeedback& fb, const std::string& failure_code)
{
    CmdVelOutputData::setTrajectory(
            fb.output_data_map, calcRampedTrajectory(current_.vel));
    retry_counter_ ++;
    if ( retry_counter_ > retry_threshold_ )
    {
        fb.success = false;
        fb.failure_code = failure_code;
    }
    else
    {
        fb.success = true;
    }
}

} // namespace cabin
