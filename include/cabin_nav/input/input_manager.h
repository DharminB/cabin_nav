#ifndef CABIN_INPUT_MANAGER_H
#define CABIN_INPUT_MANAGER_H

#include <ros/ros.h>

#include <cabin_nav/input/input.h>
#include <cabin_nav/input/input_data.h>

namespace cabin {

class InputManager
{
    public:

        InputManager() = default;

        virtual ~InputManager() = default;

        bool configure(const YAML::Node& config);

        void initializeInputData(
                InputData::Map& input_data,
                std::unordered_map<std::string, bool> active_input);

        bool getData(InputData::Map& input_data);

        void getActiveInputs(
                std::unordered_map<std::string, bool>& active_inputs) const;

        void updateActiveInputs(
                const std::vector<std::string>& required_inputs);

    protected:

        std::unordered_map<std::string, Input::Ptr> inputs_;
        std::vector<std::string> input_names_;


};

} // namespace cabin

#endif // CABIN_INPUT_MANAGER_H
