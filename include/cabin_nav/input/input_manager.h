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

        bool getData(InputData::Ptr input_data);

        std::map<std::string, bool> getActiveInputs();

        void updateActiveInputs(
                const std::vector<std::string>& required_inputs);

    protected:

        std::map<std::string, Input::Ptr> inputs_;

};

} // namespace cabin

#endif // CABIN_INPUT_MANAGER_H
