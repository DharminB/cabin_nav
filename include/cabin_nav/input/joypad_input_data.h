#ifndef CABIN_JOYPAD_INPUT_DATA_H
#define CABIN_JOYPAD_INPUT_DATA_H

#include <cabin_nav/input/input_data.h>
#include <cabin_nav/structs/joypad.h>

namespace cabin {

class JoypadInputData : public InputData
{
    public:

        using Ptr = std::shared_ptr<JoypadInputData>;
        using ConstPtr = std::shared_ptr<const JoypadInputData>;

        Joypad joypad;

        JoypadInputData():
            InputData("joypad") {}

        virtual ~JoypadInputData() = default;

        std::ostream& write(std::ostream& out) const;

        static bool getJoypad(
                const InputData::Map& input_data_map,
                const std::string& input_name,
                Joypad& _joypad);

        static bool isRedButtonPressed(
                const InputData::Map& input_data_map,
                const std::string& input_name,
                bool& is_red_button_pressed);

        static bool isGreenButtonPressed(
                const InputData::Map& input_data_map,
                const std::string& input_name,
                bool& is_green_button_pressed);

        static bool isBlueButtonPressed(
                const InputData::Map& input_data_map,
                const std::string& input_name,
                bool& is_blue_button_pressed);

        static bool isYellowButtonPressed(
                const InputData::Map& input_data_map,
                const std::string& input_name,
                bool& is_yellow_button_pressed);

        static bool isLeftButtonPressed(
                const InputData::Map& input_data_map,
                const std::string& input_name,
                bool& is_left_button_pressed);

        static bool isRightButtonPressed(
                const InputData::Map& input_data_map,
                const std::string& input_name,
                bool& is_right_button_pressed);

};

} // namespace cabin

#endif // CABIN_JOYPAD_INPUT_DATA_H
