#ifndef CABIN_LOCALISATION_INPUT_H
#define CABIN_LOCALISATION_INPUT_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>

#include <geometry_common/XYTheta.h>

#include <cabin_nav/input/input_data.h>
#include <cabin_nav/input/input.h>

namespace cabin {

class LocalisationInput : public Input
{
    public:

        LocalisationInput():
            Input("localisation"),
            nh_("~") {}

        virtual ~LocalisationInput() = default;

        bool configure(const YAML::Node& config);

        bool getData(InputData::Ptr& input_data);

        void activate();

        void deactivate();

        std::ostream& write(std::ostream& out) const;

    protected:

        ros::NodeHandle nh_;

        std::string robot_frame_{"base_link"};
        std::string global_frame_{"map"};

        std::shared_ptr<tf::TransformListener> tf_listener_;

};

} // namespace cabin

#endif // CABIN_LOCALISATION_INPUT_H
