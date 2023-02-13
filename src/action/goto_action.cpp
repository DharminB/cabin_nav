#include <set>

#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/structs/context_data.h>
#include <cabin_nav/input/localisation_input_data.h>
#include <cabin_nav/input/semantic_map_input_data.h>
#include <cabin_nav/action/goto_action.h>

using kelo::geometry_common::Pose2D;
using kelo::geometry_common::TransformMatrix2D;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool GoToAction::configure(
        const YAML::Node& config,
        const YAML::Node& params,
        const ContextData& context_data)
{
    if ( !parseInputs(config) )
    {
        return false;
    }

    /* read tolerance from config */
    goal_tolerance_linear_ = Parser::get<float>(config, "goal_tolerance_linear", 0.2f);
    goal_tolerance_angular_ = Parser::get<float>(config, "goal_tolerance_angular", 0.2f);

    /* overwrite tolerances from the ones in params (if available) */
    goal_tolerance_linear_ = Parser::get<float>(
            params, "goal_tolerance_linear", goal_tolerance_linear_);
    goal_tolerance_angular_ = Parser::get<float>(
            params, "goal_tolerance_angular", goal_tolerance_angular_);

    if ( !Parser::read(params, "goal", goal_) )
    {
        std::cout << Print::Err << Print::Time() << "[GoToAction] "
                  << "Could not parse \"goal\""
                  << Print::End << std::endl;
        return false;
    }

    std::string global_frame = Parser::get<std::string>(config, "global_frame", "map");
    std::string robot_frame = Parser::get<std::string>(config, "robot_frame", "base_link");
    std::string frame = Parser::get<std::string>(params["goal"], "frame", "map");
    if ( frame != global_frame && frame != robot_frame )
    {
        std::cout << Print::Err << Print::Time() << "[GoToAction] "
                  << "Frame of goal is \"" << frame << "\" but only \""
                  << global_frame << "\" and \"" << robot_frame << "\" is supported."
                  << Print::End << std::endl;
        return false;
    }

    TransformMatrix2D tf;
    if ( !LocalisationInputData::getLocalisationTF(context_data.input_data_map,
                inputs_map_.at("localisation"), tf) )
    {
        std::cout << Print::Err << Print::Time() << "[GoToAction] "
                  << "Could not get localisation tf"
                  << Print::End << std::endl;
        return false;
    }

    if ( frame == robot_frame )
    {
        tf.transform(goal_);
    }

    /* parse/infer start pose */
    Pose2D start;
    if ( !Parser::read(params, "start", start, false) )
    {
        std::cout << "[GoToAction] \"start\" not provided. Using robot's "
                  << "current location." << std::endl;
        start = tf.asPose2D();
    }

    plan(context_data, start);

    if ( !fillRequiredBehaviorNames(config, params) )
    {
        return false;
    }

    return true;
}

Status GoToAction::recommendNextBehavior(
        ContextData& context_data,
        const Behavior::Map& behavior_map,
        const std::string& current_behavior,
        const BehaviorFeedback& fb,
        std::vector<std::string>& required_inputs,
        std::string& next_behavior)
{
    Status status = recursiveRecommendNextBehavior(
            context_data, behavior_map, current_behavior, fb, next_behavior);

    if ( status != Status::RUNNING )
    {
        return status;
    }

    if ( behavior_map.find(next_behavior) == behavior_map.end() )
    {
        std::cerr << Print::Err << Print::Time() << "[GoToAction] "
                  << next_behavior << " behavior is not available"
                  << Print::End << std::endl;
        return Status::FAILURE;
    }

    next_behavior = getBehaviorNameBasedOnRequiredInputs(
            next_behavior, context_data, behavior_map, required_inputs);

    return Status::RUNNING;
}

Status GoToAction::recursiveRecommendNextBehavior(
        const ContextData& context_data,
        const Behavior::Map& behavior_map,
        const std::string& current_behavior,
        const BehaviorFeedback& fb,
        std::string& next_behavior)
{
    /* when goto plan is empty or robot is almost at goal */
    if ( goto_plan_.size() - goto_plan_index_ == 0 )
    {
        if ( reachedGoal(context_data) )
        {
            goto_plan_index_ ++;
            return Status::SUCCESS;
        }
        else
        {
            setIntermediateGoal(TrajectoryPoint(goal_));
            enableIsDuringTransition();
        }
        SemanticMap::ConstPtr semantic_map{nullptr};
        if ( (!SemanticMapInputData::getSemanticMap(context_data.input_data_map,
                    inputs_map_.at("semantic_map"), semantic_map) ||
             !semantic_map->isValid()) &&
             behavior_map.find("ptp_occ_grid") != behavior_map.end() )
        {
            next_behavior = "ptp_occ_grid";
            return Status::RUNNING;
        }
        next_behavior = "ptp";
        return Status::RUNNING;
    }

    std::string current_area_type = goto_plan_[goto_plan_index_]->getType();
    std::string behavior_name = getBehaviorNameFromAreaType(current_area_type);
    // std::cout << behavior_name << " " << current_area_type << std::endl;

    if ( behavior_map.find(behavior_name) == behavior_map.end() )
    {
        // FIXME remove when room has its own behavior
        if ( behavior_name == "room" )
        {
            goto_plan_index_ ++;
            is_during_transition_ = true;
            return recursiveRecommendNextBehavior(
                    context_data, behavior_map, current_behavior, fb, next_behavior);
        }
        std::cerr << Print::Err << Print::Time() << "[GoToAction] "
                  << behavior_name << " behavior is not available"
                  << Print::End << std::endl;
        return Status::FAILURE;
    }

    if ( is_during_recovery_ )
    {
        if ( recovery_behavior_stack_.empty() )
        {
            is_during_recovery_ = false;
            return recursiveRecommendNextBehavior(
                    context_data, behavior_map, current_behavior, fb, next_behavior);
        }
        behavior_name = recovery_behavior_stack_.back();
    }

    else if ( is_during_transition_ )
    {
        if ( behavior_map.at(behavior_name)->preConditionSatisfied(
                    context_data, context_data.plan[context_data.plan_index]) )
        {
            is_during_transition_ = false;
            return recursiveRecommendNextBehavior(
                    context_data, behavior_map, current_behavior, fb, next_behavior);
        }
        else
        {
            std::string previous_behavior = ( goto_plan_index_ == 0 )
                                             ? "standstill"
                                             : goto_plan_[goto_plan_index_-1]->getType();
            std::string next_intended_behavior =
                ( goto_plan_.size() - goto_plan_index_ == 0 )
                ? "standstill"
                : goto_plan_[goto_plan_index_]->getType();
            std::string transition_behavior_name =
                previous_behavior + "_to_" + next_intended_behavior + "_transition";
            if ( behavior_map.find(transition_behavior_name) == behavior_map.end() )
            {
                next_behavior = "ptp";
                return Status::RUNNING;
            }
            next_behavior = transition_behavior_name;
            return Status::RUNNING;
        }
    }

    if ( !fb.success )
    {
        if ( fb.failure_code == "" )
        {
            std::cerr << Print::Err << Print::Time() << "[GoToAction] "
                      << behavior_name << " behavior failed without recovery."
                      << Print::End << std::endl;
            return Status::FAILURE;
        }
        else
        {
            std::string recovery_behavior_name = behavior_map.at(
                    behavior_name)->getRecoveryBehaviorName(fb.failure_code);
            if ( recovery_behavior_name.empty() )
            {
                return Status::FAILURE;
            }
            if ( behavior_map.find(recovery_behavior_name) == behavior_map.end() )
            {
                std::cerr << Print::Err << Print::Time() << "[GoToAction] "
                          << recovery_behavior_name << " behavior is not available"
                          << Print::End << std::endl;
                return Status::FAILURE;
            }
            is_during_recovery_ = true;
            recovery_behavior_stack_.push_back(recovery_behavior_name);
            next_behavior = recovery_behavior_name;
            return Status::RUNNING;
        }
    }

    if ( behavior_map.at(behavior_name)->postConditionSatisfied(context_data) )
    {
        if ( is_during_recovery_ )
        {
            recovery_behavior_stack_.pop_back();
            return recursiveRecommendNextBehavior(
                    context_data, behavior_map, current_behavior, fb, next_behavior);
        }
        else
        {
            goto_plan_index_ ++;
            is_during_transition_ = true;
            return recursiveRecommendNextBehavior(
                    context_data, behavior_map, current_behavior, fb, next_behavior);
        }
    }
    else
    {
        next_behavior = behavior_name;
        return Status::RUNNING;
    }

    return Status::FAILURE; // should never reach this point
}

std::string GoToAction::getBehaviorNameFromAreaType(const std::string& area_type)
{
    return std::string(area_type);
}

std::string GoToAction::getBehaviorNameBasedOnRequiredInputs(
        const std::string& preferred_behavior_name,
        const ContextData& context_data,
        const Behavior::Map& behavior_map,
        std::vector<std::string>& required_inputs)
{
    required_inputs = behavior_map.at(preferred_behavior_name)->getRequiredInputs();
    for ( size_t i = 0; i < required_inputs.size(); i++ )
    {
        if ( !context_data.active_inputs.at(required_inputs[i]) )
        {
            std::cerr << Print::Warn << Print::Time() << "[GoToAction] "
                      << preferred_behavior_name
                      << " behavior requires an inactive input " << required_inputs[i]
                      << Print::End << std::endl;
            return "standstill";
        }
    }
    return preferred_behavior_name;
}

void GoToAction::plan(const ContextData& context_data, const Pose2D& start)
{
    SemanticMap::ConstPtr semantic_map{nullptr};
    if ( !SemanticMapInputData::getSemanticMap(context_data.input_data_map,
                inputs_map_.at("semantic_map"), semantic_map) ||
         !semantic_map->isValid() )
    {
        return;
    }

    goto_plan_ = semantic_map->plan(
            semantic_map->getAreaContainingPoint(start.position()),
            semantic_map->getAreaContainingPoint(goal_.position()));
}

bool GoToAction::reachedGoal(const ContextData& context_data) const
{
    TransformMatrix2D tf;
    if ( !LocalisationInputData::getLocalisationTF(context_data.input_data_map,
                inputs_map_.at("localisation"), tf) )
    {
        std::cout << Print::Err << Print::Time() << "[GoToAction] "
                  << "Could not get localisation tf"
                  << Print::End << std::endl;
        return true;
    }
    const Pose2D robot_pose = tf.asPose2D();
    return Utils::isWithinTolerance(
            robot_pose, goal_, goal_tolerance_linear_, goal_tolerance_angular_);
}

bool GoToAction::fillRequiredBehaviorNames(
        const YAML::Node& config,
        const YAML::Node& params)
{
    /* parse available behaviors */
    std::vector<std::string> available_behaviors;
    std::set<std::string> available_behaviors_set;
    if ( !Parser::read<std::vector<std::string>>(
                config, "available_behaviors", available_behaviors) ||
         available_behaviors.empty() )
    {
        std::cout << Print::Err << Print::Time() << "[GoToAction] "
                  << "Available behaviors are not provided or it is an empty list."
                  << Print::End << std::endl;
        return false;
    }
    for ( size_t i = 0; i < available_behaviors.size(); i++ )
    {
        available_behaviors_set.insert(available_behaviors[i]);
    }

    std::set<std::string> required_behavior_names_set;
    required_behavior_names_set.insert("standstill");
    GoToAction::addBehaviorName("ptp", available_behaviors_set,
            required_behavior_names_set);
    GoToAction::addBehaviorName("ptp_occ_grid", available_behaviors_set,
            required_behavior_names_set);
    required_behavior_names_set.insert("joypad");

    for ( size_t i = 0; i < goto_plan_.size(); i++ )
    {
        std::string curr_behavior_name = getBehaviorNameFromAreaType(goto_plan_[i]->getType());
        std::string prev_behavior_name = ( i > 0 )
            ? getBehaviorNameFromAreaType(goto_plan_[i-1]->getType())
            : "standstill";
        std::string next_behavior_name = ( i+1 < goto_plan_.size() )
            ? getBehaviorNameFromAreaType(goto_plan_[i+1]->getType())
            : "standstill";

        std::string prev_transition_behavior_name =
             prev_behavior_name + "_to_" + curr_behavior_name + "_transition";
        std::string next_transition_behavior_name =
             curr_behavior_name + "_to_" + next_behavior_name + "_transition";
        GoToAction::addBehaviorName(curr_behavior_name,
                available_behaviors_set, required_behavior_names_set);
        GoToAction::addBehaviorName(prev_transition_behavior_name,
                available_behaviors_set, required_behavior_names_set);
        GoToAction::addBehaviorName(next_transition_behavior_name,
                available_behaviors_set, required_behavior_names_set);
    }

    required_behavior_names_.reserve(required_behavior_names_set.size());
    for ( auto itr = required_behavior_names_set.begin();
          itr != required_behavior_names_set.end();
          itr ++ )
    {
        required_behavior_names_.push_back(*itr);
    }
    return true;
}

void GoToAction::addBehaviorName(
        const std::string& behavior_name,
        const std::set<std::string>& available_behaviors,
        std::set<std::string>& required_behaviors)
{
    if ( available_behaviors.find(behavior_name) != available_behaviors.end() )
    {
        required_behaviors.insert(behavior_name);
    }
}

std::ostream& GoToAction::write(std::ostream& out) const
{
    out << "type: " << getType() << std::endl;
    out << "goal: " << goal_ << std::endl;
    out << "intermediate_goal: " << intermediate_goal_ << std::endl;
    if ( is_during_transition_ )
    {
        out << "During transition." << std::endl;
    }
    out << "goto_plan_index: " << goto_plan_index_ << std::endl;
    out << "goto_plan size: " << goto_plan_.size() << std::endl;

    out << getPlanAsString() << std::endl;

    if ( is_during_recovery_ )
    {
        out << "During recovery." << std::endl;
        out << "recovery_behavior_stack: [";
        for ( std::string recovery_behavior_name : recovery_behavior_stack_ )
        {
            out << recovery_behavior_name << ", ";
        }
        out << "]" << std::endl;
    }
    return out;
}

std::string GoToAction::getPlanAsString() const
{
    std::string plan_string;
    plan_string += "goto_plan: ROBOT -> ";
    for ( size_t i = 0; i < goto_plan_.size(); i++ )
    {
        if ( goto_plan_index_ == i )
        {
            plan_string += "*";
        }
        plan_string += goto_plan_[i]->getName() + " -> ";
    }
    plan_string += "GOAL";
    return plan_string;
}

std::vector<visualization_msgs::Marker> GoToAction::asMarkers() const
{
    std::vector<visualization_msgs::Marker> markers;
    for ( size_t i = goto_plan_index_; i < goto_plan_.size(); i++ )
    {
        markers.push_back(goto_plan_[i]->asMarker("global"));
    }
    markers.push_back(goal_.asMarker(
                "global", 1.0f, 0.0f, 0.0f, 1.0f, 0.3f, 0.05f, 0.05f));
    return markers;
}

void GoToAction::setGoal(const Pose2D& goal)
{
    goal_ = goal;
}

Pose2D GoToAction::getGoal() const
{
    return goal_;
}

TrajectoryPoint GoToAction::getIntermediateGoal() const
{
    return intermediate_goal_;
}

const std::vector<Area::ConstPtr>& GoToAction::getGoToPlan() const
{
    return goto_plan_;
}

size_t GoToAction::getGoToPlanIndex() const
{
    return goto_plan_index_;
}

bool GoToAction::isDuringTransition() const
{
    return is_during_transition_;
}

bool GoToAction::isDuringRecovery() const
{
    return is_during_recovery_;
}

void GoToAction::setIntermediateGoal(const TrajectoryPoint& intermediate_goal)
{
    intermediate_goal_ = intermediate_goal;
}

void GoToAction::enableIsDuringTransition()
{
    is_during_transition_ = true;
}

} // namespace cabin
