#include <yaml_common/Parser2.h>
#include <geometry_common/Pose2D.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/cabin_navigator_ros.h>

using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool CABINNavigatorROS::initialise()
{
    /* read configuration files */
    std::string config_dir;
    nh_.param<std::string>("config_dir", config_dir, "");
    if ( config_dir.empty() )
    {
        ROS_FATAL("config_dir is empty string");
        return false;
    }
    std::string task_config_file = config_dir + "tasks.yaml";
    std::string action_config_file = config_dir + "actions.yaml";
    std::string behavior_config_file = config_dir + "behaviors.yaml";
    std::string input_config_file = config_dir + "inputs.yaml";
    std::string output_config_file = config_dir + "outputs.yaml";
    YAML::Node task_config, action_config, behavior_config, input_config, output_config;
    if ( !Parser::loadFile(task_config_file, task_config) ||
         !Parser::loadFile(action_config_file, action_config) ||
         !Parser::loadFile(behavior_config_file, behavior_config) ||
         !Parser::loadFile(input_config_file, input_config) ||
         !Parser::loadFile(output_config_file, output_config) )
    {
        ROS_FATAL("Could not read config files");
        return false;
    }

    if ( !CABINNavigator::configure(
                task_config,
                action_config,
                behavior_config,
                input_config,
                output_config) )
    {
        ROS_FATAL("Could not configure CABINNavigator");
        return false;
    }

    /* subscribers */
    goal_sub_ = nh_.subscribe("goal", 1, &CABINNavigatorROS::goalCb, this);
    task_request_sub_ = nh_.subscribe("task_request", 1, &CABINNavigatorROS::taskRequestCb, this);
    cancel_sub_ = nh_.subscribe("cancel", 1, &CABINNavigatorROS::cancelCb, this);

    return true;
}

void CABINNavigatorROS::goalCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    kelo::geometry_common::Pose2D new_goal(*msg);
    YAML::Node goal_yaml;
    goal_yaml["x"] = new_goal.x;
    goal_yaml["y"] = new_goal.y;
    goal_yaml["theta"] = new_goal.theta;
    goal_yaml["frame"] = msg->header.frame_id;
    YAML::Node task_request_yaml;
    task_request_yaml["task_type"] = "goto";
    task_request_yaml["goal"] = goal_yaml;
    setNewTaskRequest(task_request_yaml);
}

void CABINNavigatorROS::taskRequestCb(const std_msgs::String::ConstPtr& msg)
{
    try
    {
        YAML::Node task_request_yaml = YAML::Load(msg->data);
        setNewTaskRequest(task_request_yaml);
    }
    catch (const YAML::Exception&) {
        ROS_WARN("Received an invalid task_request. Please make sure it is in yaml format");
    }
}

void CABINNavigatorROS::cancelCb(const std_msgs::Empty::ConstPtr& msg)
{
    cancel();
}

} // namespace cabin
