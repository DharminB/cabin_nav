#include <cabin_nav/input/joypad_input_data.h>

namespace cabin {

std::ostream& JoypadInputData::write(std::ostream& out) const
{
    out << "<Input type: " << getType() << ", joypad: " << joypad << ">";
    return out;
}

bool JoypadInputData::getJoypad(
        const InputData::Map& input_data_map,
        const std::string& input_name,
        Joypad& _joypad)
{
    if ( !InputData::isValid(input_data_map, input_name, "joypad") )
    {
        return false;
    }

    JoypadInputData::ConstPtr joypad_input_data =
        std::static_pointer_cast<JoypadInputData>(
                input_data_map.at(input_name));

    _joypad = joypad_input_data->joypad;

    return true;
}

bool JoypadInputData::isRedButtonPressed(
        const InputData::Map& input_data_map,
        const std::string& input_name,
        bool& is_red_button_pressed)
{
    if ( !InputData::isValid(input_data_map, input_name, "joypad") )
    {
        return false;
    }

    is_red_button_pressed = ( std::static_pointer_cast<JoypadInputData>(
                input_data_map.at(input_name))->joypad.buttons[2] == 1 );
    return true;
}

bool JoypadInputData::isGreenButtonPressed(
        const InputData::Map& input_data_map,
        const std::string& input_name,
        bool& is_green_button_pressed)
{
    if ( !InputData::isValid(input_data_map, input_name, "joypad") )
    {
        return false;
    }

    is_green_button_pressed = ( std::static_pointer_cast<JoypadInputData>(
                input_data_map.at(input_name))->joypad.buttons[1] == 1 );
    return true;
}

bool JoypadInputData::isBlueButtonPressed(
        const InputData::Map& input_data_map,
        const std::string& input_name,
        bool& is_blue_button_pressed)
{
    if ( !InputData::isValid(input_data_map, input_name, "joypad") )
    {
        return false;
    }

    is_blue_button_pressed = ( std::static_pointer_cast<JoypadInputData>(
                input_data_map.at(input_name))->joypad.buttons[0] == 1 );
    return true;
}

bool JoypadInputData::isYellowButtonPressed(
        const InputData::Map& input_data_map,
        const std::string& input_name,
        bool& is_yellow_button_pressed)
{
    if ( !InputData::isValid(input_data_map, input_name, "joypad") )
    {
        return false;
    }

    is_yellow_button_pressed = ( std::static_pointer_cast<JoypadInputData>(
                input_data_map.at(input_name))->joypad.buttons[3] == 1 );
    return true;
}

bool JoypadInputData::isLeftButtonPressed(
        const InputData::Map& input_data_map,
        const std::string& input_name,
        bool& is_left_button_pressed)
{
    if ( !InputData::isValid(input_data_map, input_name, "joypad") )
    {
        return false;
    }

    is_left_button_pressed = ( std::static_pointer_cast<JoypadInputData>(
                input_data_map.at(input_name))->joypad.buttons[4] == 1 );
    return true;
}

bool JoypadInputData::isRightButtonPressed(
        const InputData::Map& input_data_map,
        const std::string& input_name,
        bool& is_right_button_pressed)
{
    if ( !InputData::isValid(input_data_map, input_name, "joypad") )
    {
        return false;
    }

    is_right_button_pressed = ( std::static_pointer_cast<JoypadInputData>(
                input_data_map.at(input_name))->joypad.buttons[5] == 1 );
    return true;
}

} // namespace cabin
