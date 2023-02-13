#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/utils/utils.h>
#include <cabin_nav/task/task_planner.h>
#include <cabin_nav/cabin_navigator.h>

using Parser = kelo::yaml_common::Parser2;

namespace cabin {

CABINNavigator::~CABINNavigator()
{
    if ( is_active_ )
    {
        stop();
    }
}

bool CABINNavigator::configure(
        const YAML::Node& task_config,
        const YAML::Node& action_config,
        const YAML::Node& behavior_config,
        const YAML::Node& input_config,
        const YAML::Node& output_config)
{
    // TODO: add action params file

    if ( !input_manager_.configure(input_config) )
    {
        std::cout << Print::Err << Print::Time() << "[CABINNavigator] "
                  << "Failed to configure input_manger."
                  << Print::End << std::endl;
        return false;
    }
    input_manager_.initializeInputData(context_data_.input_data_map, context_data_.active_inputs);
    input_manager_.getActiveInputs(context_data_.active_inputs);

    if ( !output_manager_.configure(output_config) )
    {
        std::cout << Print::Err << Print::Time() << "[CABINNavigator] "
                  << "Failed to configure output_manger."
                  << Print::End << std::endl;
        return false;
    }
    output_manager_.initializeOutputDataMap(behavior_fb_.output_data_map);

    std::cout << std::setprecision(3) << std::fixed;

    if ( !behavior_factory_.configure(behavior_config) )
    {
        return false;
    }

    /* make sure "standstill" behavior is present */
    Behavior::Ptr standstill_behavior = behavior_factory_.createBehavior("standstill");
    if ( standstill_behavior == nullptr )
    {
        std::cerr << Print::Err << Print::Time() << "[CABINNavigator] "
                  << "standstill behavior is not available. It is mandatory."
                  << Print::End << std::endl;
        return false;
    }
    behavior_map_["standstill"] = standstill_behavior;
    context_data_.current_behavior_name = "standstill";

    Behavior::Ptr joypad_behavior = behavior_factory_.createBehavior("joypad");
    if ( joypad_behavior == nullptr )
    {
        std::cerr << Print::Warn << Print::Time() << "[CABINNavigator] "
                  << "joypad behavior is not available."
                  << Print::End << std::endl;
        return false;
    }
    behavior_map_["joypad"] = joypad_behavior;

    if ( !action_factory_.configure(action_config) )
    {
        return false;
    }

    is_configured_ = true;
    std::cout << Print::Success << Print::Time() << "[CABINNavigator] "
              << "configured" << Print::End << std::endl;
    return true;
}

bool CABINNavigator::start()
{
    if ( is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[CABINNavigator] "
                  << "Already active. Ignoring start() call."
                  << Print::End << std::endl;
        return false;
    }

    if ( !is_configured_ )
    {
        std::cout << Print::Warn << Print::Time() << "[CABINNavigator] "
                  << "Not configured yet. Ignoring start() call."
                  << Print::End << std::endl;
        return false;
    }

    is_active_ = true;
    loop_thread_ = std::thread(&CABINNavigator::mainLoop, this);
    std::cout << Print::Success << Print::Time() << "[CABINNavigator] "
              << "Started" << Print::End << std::endl;
    return true;
}

void CABINNavigator::stop()
{
    if ( !is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[CABINNavigator] "
                  << "Not active right now. Ignoring stop() call."
                  << Print::End << std::endl;
        return;
    }

    std::cout << "[CABINNavigator] Stopping" << std::endl;
    is_active_ = false;
    if ( loop_thread_.joinable() )
    {
        loop_thread_.join();
    }
    std::cout << Print::Success << Print::Time() << "[CABINNavigator] "
              << "Stopped" << Print::End << std::endl;
}

bool CABINNavigator::isActive() const
{
    return is_active_;
}

void CABINNavigator::mainLoop()
{
    float ideal_loop_duration = 1.0f; // seconds
    std::chrono::steady_clock::time_point start_time;
    std::chrono::duration<float> time_taken, sleep_duration;

    while ( is_active_ )
    {
        start_time = std::chrono::steady_clock::now();

        ideal_loop_duration = runOnce();

        /* sleep for remaining time */
        time_taken = std::chrono::steady_clock::now() - start_time;
        sleep_duration = std::chrono::duration<float>(ideal_loop_duration) - time_taken;
        std::this_thread::sleep_for(sleep_duration);
    }

    output_manager_.stopAllOutputs();

    std::cout << Print::Success << Print::Time() << "[CABINNavigator] Finished loop thread."
              << Print::End << std::endl;

}

float CABINNavigator::runOnce()
{
    std::lock_guard<std::mutex> guard(loop_thread_mutex_);

    /* get input data */
    if ( !input_manager_.getData(context_data_.input_data_map) )
    {
        return 0.5f;
    }

    input_manager_.getActiveInputs(context_data_.active_inputs);

    std::vector<std::string> required_inputs;

    output_manager_.resetOutputDataMap(behavior_fb_.output_data_map);

    /* check for transition */
    Status task_status{Status::UNKNOWN};
    const std::string suggested_behavior = TaskManager::recommendNextBehavior(
            context_data_, behavior_map_, context_data_.current_behavior_name,
            behavior_fb_, required_inputs, task_status);

    /* if transition required, handle transition */
    if ( context_data_.current_behavior_name != suggested_behavior )
    {
        transitionTo(suggested_behavior, required_inputs);
    }

    /* execute behavior */
    Behavior::Ptr& behavior = behavior_map_[context_data_.current_behavior_name];
    try
    {
        behavior->runOnce(context_data_, behavior_fb_);
    }
    catch ( const std::exception& e )
    {
        std::cout << Print::Err << Print::Time() << "[CABINNavigator] "
                  << "Following exception occurred while executing "
                  << context_data_.current_behavior_name << " behavior's step()"
                  << std::endl << e.what() << std::endl << "Exiting."
                  << Print::End << std::endl;
        is_active_ = false;
        return 0.0f;
    }

    if ( !output_manager_.setData(behavior_fb_.output_data_map, context_data_.input_data_map) )
    {
        std::cout << Print::Err << Print::Time() << "[CABINNavigator] "
                  << "Could not setData successfully after executing "
                  << context_data_.current_behavior_name << " behavior's step(). "
                  << "Exiting." << Print::End << std::endl;
        is_active_ = false;
        return 0.0f;
    }

    /* let behavior update current action */
    if ( context_data_.isPlanValid() )
    {
        behavior->updateAction(context_data_.plan[context_data_.plan_index]);
    }

    /* update input manager with required_inputs */
    input_manager_.updateActiveInputs(required_inputs);

    if ( task_status == Status::SUCCESS || task_status == Status::FAILURE )
    {
        cleanUp();
    }

    return behavior->getIdealLoopDuration();
}

void CABINNavigator::transitionTo(
        const std::string& suggested_behavior,
        std::vector<std::string>& required_inputs)
{
    behavior_map_[context_data_.current_behavior_name]->reset();

    if ( behavior_map_.find(suggested_behavior) == behavior_map_.end() )
    {
        std::cerr << Print::Err << Print::Time() << "[CABINNavigator] "
                  << suggested_behavior << " behavior is not available"
                  << Print::End << std::endl;
        required_inputs = behavior_map_["standstill"]->getRequiredInputs();
        context_data_.current_behavior_name = "standstill";
    }
    else
    {
        context_data_.current_behavior_name = suggested_behavior;
    }

}

void CABINNavigator::setNewTaskRequest(const YAML::Node& task_request)
{
    if ( !is_configured_ )
    {
        std::cout << Print::Warn << Print::Time() << "[CABINNavigator] "
                  << "Not configured yet. Ignoring setNewTaskRequest() call."
                  << Print::End << std::endl;
        return;
    }

    cancel();

    std::cout << Print::Time() << "[CABINNavigation] "
              << "Received a new task" << std::endl
              << task_request << std::endl;

    std::lock_guard<std::mutex> guard(loop_thread_mutex_);
    std::vector<YAML::Node> actions_params;
    if ( !TaskPlanner::plan(task_request, context_data_, actions_params) )
    {
        std::cout << Print::Err << Print::Time() << "[CABINNavigator] Planning failed"
                  << Print::End << std::endl;
        return;
    }

    std::cout << Print::Success << Print::Time() << "[CABINNavigator] "
              << "Successfully planned."
              << Print::End << std::endl;

    for ( const YAML::Node& action_params : actions_params )
    {
        Action::Ptr action = action_factory_.createAction(action_params, context_data_);
        if ( action == nullptr )
        {
            cleanUp();
            return;
        }
        context_data_.plan.push_back(action);

        const std::vector<std::string>& behavior_names =
            action->getAllRequiredBehaviorNames();
        for ( const std::string& behavior_name : behavior_names )
        {
            if ( behavior_map_.find(behavior_name) != behavior_map_.end() )
            {
                continue;
            }
            Behavior::Ptr behavior = behavior_factory_.createBehavior(behavior_name);
            if ( behavior == nullptr )
            {
                std::cout << Print::Err << Print::Time() << "[CABINNavigation] "
                          << "Could not create behavior " << behavior_name << "."
                          << Print::End << std::endl;
                cleanUp();
                return;
            }
            behavior_map_[behavior_name] = behavior;
        }
    }

    std::cout << context_data_.getPlanAsString() << std::endl;

    std::cout << Print::Success << Print::Time() << "[CABINNavigation] "
              << "Successfully initialised new task"
              << Print::End << std::endl;
}

void CABINNavigator::cancel()
{
    if ( !is_configured_ )
    {
        std::cout << Print::Warn << Print::Time() << "[CABINNavigator] "
                  << "Not configured yet. Ignoring cancel() call."
                  << Print::End << std::endl;
        return;
    }
    std::lock_guard<std::mutex> guard(loop_thread_mutex_);

    if ( !context_data_.isPlanValid() )
    {
        return;
    }

    std::cout << Print::Warn << Print::Time() << "[CABINNavigator] "
              << "Cancelling current task."
              << Print::End << std::endl;
    cleanUp();
}

void CABINNavigator::cleanUp()
{
    context_data_.plan.clear();
    context_data_.plan_index = 0;

    const std::string default_behavior_name = "standstill";
    context_data_.current_behavior_name = default_behavior_name;
    const std::string joypad_behavior_name = "joypad";

    /* erase all behaviors except default_behavior_name */
    Behavior::Map::iterator itr = behavior_map_.begin();
    while ( itr != behavior_map_.end() )
    {
        if ( itr->first != default_behavior_name &&
             itr->first != joypad_behavior_name )
        {
            itr = behavior_map_.erase(itr);
        }
        else
        {
            itr ++;
        }
    }
}

} // namespace cabin
