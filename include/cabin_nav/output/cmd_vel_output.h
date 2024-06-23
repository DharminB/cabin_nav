#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>

#include <geometry_common/XYTheta.h>
#include <geometry_common/Polygon2D.h>

#include <cabin_nav/output/output_data.h>
#include <cabin_nav/output/output.h>

namespace cabin {

class CmdVelOutput : public Output
{
    public:

        CmdVelOutput():
            Output("cmd_vel"),
            nh_("~") {}

        virtual ~CmdVelOutput() = default;

        bool configure(const YAML::Node& config);

        void initializeOutputData(OutputData::Ptr& output_data) const;

        bool setData(
                const OutputData::Ptr& output_data,
                const InputData::Map& input_data_map);

        void start() override;

        void stop() override;

        std::ostream& write(std::ostream& out) const;

    protected:

        ros::NodeHandle nh_;

        ros::Publisher cmd_vel_pub_;
        size_t queue_size_{1};
        std::string topic_;

        ros::Publisher debug_target_vel_pub_;
        ros::Publisher traj_pub_;
        ros::Publisher footprint_pub_;
        std::string robot_frame_{"base_link"};

        kelo::geometry_common::Velocity2D vel_;
        kelo::geometry_common::Velocity2D target_vel_;
        std::chrono::steady_clock::time_point target_vel_time_;

        kelo::geometry_common::Velocity2D max_vel_, min_vel_;
        kelo::geometry_common::Acceleration2D max_acc_, min_acc_;
        kelo::geometry_common::Acceleration2D max_braking_acc_, min_braking_acc_;
        bool use_braking_acc_{false};

        std::string laser_input_name_;

        float sample_time_{1.0f};

        kelo::geometry_common::Polygon2D footprint_;
        geometry_msgs::PolygonStamped footprint_msg_;

        bool dummy_mode_{false};

        void step();

        void applySafetyConstraints(const InputData::Map& input_data_map);

        bool parseFootprint(const YAML::Node& config);

};

} // namespace cabin
