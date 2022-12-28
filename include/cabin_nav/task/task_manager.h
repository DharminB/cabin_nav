#ifndef CABIN_TRANSITION_MANAGER_H
#define CABIN_TRANSITION_MANAGER_H

#include <cabin_nav/behavior/behavior.h>
#include <cabin_nav/structs/context_data.h>
#include <cabin_nav/structs/status.h>

namespace cabin {

class TaskManager
{
    public:
        static std::string recommendNextBehavior(
                ContextData& context_data,
                const Behavior::Map& behavior_map,
                const std::string& current_behavior,
                const BehaviorFeedback& fb,
                std::vector<std::string>& required_inputs,
                Status& task_status);

        static void handleGripperStatusChange(
                ContextData& context_data,
                const std::string& current_behavior);

};

} // namespace cabin

#endif // CABIN_TRANSITION_MANAGER_H
