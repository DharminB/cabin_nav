#ifndef CABIN_JOYPAD_INPUT_H
#define CABIN_JOYPAD_INPUT_H

#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <cabin_nav/input/input_data.h>
#include <cabin_nav/structs/joypad.h>
#include <cabin_nav/input/input.h>

namespace cabin {

class JoypadInput : public Input
{
    public:

        JoypadInput():
            nh_("~") {}

        virtual ~JoypadInput() = default;

        bool configure(const YAML::Node& config);

        bool getData(InputData::Ptr& input_data, const std::string& input_name);

        void activate();

        void deactivate();

    protected:

        ros::NodeHandle nh_;
        ros::Subscriber sub_;

        std::string topic_;

        std::mutex mutex_;
        sensor_msgs::Joy joypad_;

        Joypad default_joypad_;

        void joyCb(const sensor_msgs::Joy::ConstPtr& msg);

};

} // namespace cabin

#endif // CABIN_JOYPAD_INPUT_H
