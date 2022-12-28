#include <cabin_nav/utils/print.h>
#include <cabin_nav/task/task_manager.h>

namespace cabin {

std::string TaskManager::recommendNextBehavior(
        ContextData& context_data,
        const Behavior::Map& behavior_map,
        const std::string& current_behavior,
        const BehaviorFeedback& fb,
        std::vector<std::string>& required_inputs,
        Status& task_status)
{
    if ( context_data.isPlanValid() )
    {
        task_status = Status::RUNNING;
    }

    if ( behavior_map.find("joypad") != behavior_map.end() )
    {
        if ( behavior_map.at("joypad")->preConditionSatisfied(context_data, nullptr) )
        {
            required_inputs = behavior_map.at("joypad")->getRequiredInputs();
            return "joypad";
        }

        if ( current_behavior == "joypad" && 
             !behavior_map.at("joypad")->postConditionSatisfied(context_data) )
        {
            required_inputs = behavior_map.at("joypad")->getRequiredInputs();
            return "joypad";
        }
    }

    if ( context_data.is_paused )
    {
        required_inputs = behavior_map.at("standstill")->getRequiredInputs();
        return "standstill";
    }

    if ( !context_data.isPlanValid() )
    {
        required_inputs = behavior_map.at("standstill")->getRequiredInputs();
        return "standstill";
    }

    /* when everything is nominal, just follow the plan */
    std::vector<std::string> valid_action_types{
        "goto", "wall_following", "dock", "undock"};

    if ( std::find(valid_action_types.begin(), valid_action_types.end(),
                   context_data.plan[context_data.plan_index]->getType()) != valid_action_types.end() )
    {
        std::string next_behavior;
        Status status = context_data.plan[context_data.plan_index]->recommendNextBehavior(
                context_data, behavior_map, current_behavior, fb,
                required_inputs, next_behavior);
        if ( status == Status::RUNNING )
        {
            return next_behavior;
        }
        else if ( status == Status::SUCCESS )
        {
            context_data.plan_index ++;
            if ( context_data.plan_index >= context_data.plan.size() )
            {
                std::cout << Print::Success << Print::Time() << "[TaskManager] "
                          << "Task succeeded" << Print::End << std::endl;
                context_data.plan.clear();
                context_data.plan_index = 0;
                task_status = Status::SUCCESS;
                required_inputs = behavior_map.at("standstill")->getRequiredInputs();
                return "standstill";
            }
            else
            {
                return TaskManager::recommendNextBehavior(context_data,
                        behavior_map, current_behavior, fb, required_inputs,
                        task_status);
            }
        }
        else
        {
            std::cout << Print::Err << Print::Time() << "[TaskManager] "
                      << "Task failed" << Print::End << std::endl;
            context_data.plan.clear();
            context_data.plan_index = 0;
            task_status = Status::FAILURE;
            required_inputs = behavior_map.at("standstill")->getRequiredInputs();
            return "standstill";
        }
    }
    else
    {
        std::cerr << Print::Err << Print::Time() << "[TaskManager] "
                  << "Action of " << context_data.plan[context_data.plan_index]->getType()
                  << " type is not supported." << Print::End << std::endl;
        context_data.plan.clear();
        context_data.plan_index = 0;
        task_status = Status::FAILURE;
        required_inputs = behavior_map.at("standstill")->getRequiredInputs();
        return "standstill";
    }
}

} // namespace cabin
