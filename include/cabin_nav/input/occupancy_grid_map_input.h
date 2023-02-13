#ifndef CABIN_OCCUPANCY_GRID_MAP_INPUT_H
#define CABIN_OCCUPANCY_GRID_MAP_INPUT_H

#include <mutex>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <cabin_nav/input/input_data.h>
#include <cabin_nav/input/input.h>

namespace cabin {

class OccupancyGridMapInput : public Input
{
    public:

        OccupancyGridMapInput():
            Input("occupancy_grid_map"),
            nh_("~") {}

        virtual ~OccupancyGridMapInput() = default;

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
        nav_msgs::OccupancyGrid occ_grid_;

        void occGridMapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg);

};

} // namespace cabin

#endif // CABIN_OCCUPANCY_GRID_MAP_INPUT_H
