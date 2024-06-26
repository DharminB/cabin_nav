#pragma once

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
            Input("joypad"),
            nh_("~") {}

        virtual ~JoypadInput() = default;

        bool configure(const YAML::Node& config);

        bool getData(InputData::Ptr& input_data);

        void activate();

        void deactivate();

        std::ostream& write(std::ostream& out) const;

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
