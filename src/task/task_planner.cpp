#include <iostream>
#include <regex>

#include <geometry_common/Point2D.h>
#include <geometry_common/Pose2D.h>
#include <geometry_common/Polygon2D.h>
#include <geometry_common/LineSegment2D.h>
#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>

#include <cabin_nav/task/task_planner.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Vector2D;
using kelo::geometry_common::Pose2D;
using kelo::geometry_common::PoseVec2D;
using kelo::geometry_common::Polygon2D;
using kelo::geometry_common::LineSegment2D;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool TaskPlanner::plan(
        const YAML::Node& task_request,
        const ContextData& context_data,
        std::vector<YAML::Node>& actions_params)
{
    std::string task_type;
    if ( !Parser::read<std::string>(task_request, "task_type", task_type) )
    {
        std::cerr << Print::Err << Print::Time() << "[TaskPlanner] "
                  << "Task request does not have a \"task_type\""
                  << Print::End << std::endl;
        return false;
    }

    if ( task_type == "goto" )
    {
        return TaskPlanner::planGoToTask(task_request, context_data, actions_params);
    }
    else
    {
        std::cerr << Print::Err << Print::Time() << "[TaskPlanner] "
                  << "Task of type \"" << task_type << "\" is not supported."
                  << Print::End << std::endl;
        return false;
    }

    return false;
}

bool TaskPlanner::planGoToTask(
        const YAML::Node& task_request,
        const ContextData& context_data,
        std::vector<YAML::Node>& actions_params)
{
    YAML::Node goto_action_params = YAML::Clone(task_request);
    goto_action_params["type"] = "goto";
    actions_params.push_back(goto_action_params);
    return true;
}

} // namespace cabin
