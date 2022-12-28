#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/input/joypad_input.h>

using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool JoypadInput::configure(const YAML::Node& config)
{
    if ( !Parser::read<std::string>(config, "topic", topic_) )
    {
        std::cerr << Print::Err << Print::Time() << "[JoypadInput] "
                  << "topic not provided"
                  << Print::End << std::endl;
        return false;
    }
    always_active_ = Parser::get<bool>(config, "always_active", false);

    default_joypad_ = Joypad();
    default_joypad_.buttons.fill(0);
    default_joypad_.axes.fill(0.0f);
    return true;
}

bool JoypadInput::getData(InputData::Ptr& input_data, const std::string& input_name)
{
    std::lock_guard<std::mutex> guard(mutex_);

    input_data->joypad = default_joypad_;

    if ( joypad_.buttons.empty() || joypad_.axes.empty() )
    {
        return true;
    }

    /* ignore the message if joypad is not using same mode */
    if ( joypad_.buttons.size() != default_joypad_.buttons.size() ||
         joypad_.axes.size() != default_joypad_.axes.size() )
    {
        ROS_ERROR("Invalid joypad message. Ignoring.");
        return false;
    }

    for ( size_t i = 0; i < joypad_.buttons.size(); i++ )
    {
        input_data->joypad.buttons[i] = joypad_.buttons[i];
    }
    for ( size_t i = 0; i < joypad_.axes.size(); i++ )
    {
        input_data->joypad.axes[i] = joypad_.axes[i];
    }

    return true;
}

void JoypadInput::activate()
{
    if ( is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[JoypadInput] "
                  << "Already active. Ignoring activate() call"
                  << Print::End << std::endl;
        return;
    }

    is_active_ = true;
    sub_ = nh_.subscribe(topic_, 1, &JoypadInput::joyCb, this);
}

void JoypadInput::deactivate()
{
    if ( !is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[JoypadInput] "
                  << "Already inactive. Ignoring deactivate() call"
                  << Print::End << std::endl;
        return;
    }

    if ( always_active_ )
    {
        return;
    }

    is_active_ = false;
    sub_.shutdown();
}


void JoypadInput::joyCb(const sensor_msgs::Joy::ConstPtr& msg)
{
    std::lock_guard<std::mutex> guard(mutex_);
    joypad_ = sensor_msgs::Joy(*msg);
}

} // namespace cabin
