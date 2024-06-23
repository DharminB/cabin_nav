#pragma once

#include <thread>
#include <mutex>
#include <atomic>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <yaml-cpp/yaml.h>

#include <cabin_nav/structs/context_data.h>
#include <cabin_nav/task/task_manager.h>

#include <cabin_nav/action/action_factory.h>

#include <cabin_nav/behavior/behavior_factory.h>
#include <cabin_nav/behavior/behavior.h>
#include <cabin_nav/structs/behavior_feedback.h>

#include <cabin_nav/input/input_data.h>
#include <cabin_nav/input/input_manager.h>

#include <cabin_nav/output/output_data.h>
#include <cabin_nav/output/output_manager.h>

namespace cabin {

class CABINNavigator
{
    public:

        CABINNavigator() = default;

        virtual ~CABINNavigator();

        virtual bool configure(
                const YAML::Node& task_config,
                const YAML::Node& action_config,
                const YAML::Node& behavior_config,
                const YAML::Node& input_config,
                const YAML::Node& output_config);

        virtual bool start();

        virtual void stop();

        bool isActive() const;

        void setNewTaskRequest(const YAML::Node& task_request);

        void cancel();

    protected:

        std::thread loop_thread_;
        std::mutex loop_thread_mutex_;

        std::atomic<bool> is_active_{false};
        std::atomic<bool> is_configured_{false};

        ContextData context_data_;
        BehaviorFeedback behavior_fb_;

        YAML::Node new_task_request_;
        bool new_task_request_set_{false};

        // state machine related variables
        Behavior::Map behavior_map_;
        BehaviorFactory behavior_factory_;

        ActionFactory action_factory_;

        InputManager input_manager_;
        OutputManager output_manager_;

        void mainLoop();

        float runOnce();

        void handleStopPauseResume();

        void handleNewTaskRequest();

        void transitionTo(
                const std::string& suggested_behavior,
                std::vector<std::string>& required_inputs);

        void cleanUp();

};

} // namespace cabin
