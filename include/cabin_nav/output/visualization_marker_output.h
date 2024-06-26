#pragma once

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <cabin_nav/output/output_data.h>
#include <cabin_nav/output/output.h>

namespace cabin {

class VisualizationMarkerOutput : public Output
{
    public:

        VisualizationMarkerOutput():
            Output("visualization_marker"),
            nh_("~") {}

        virtual ~VisualizationMarkerOutput() = default;

        void initializeOutputData(OutputData::Ptr& output_data) const;

        bool configure(const YAML::Node& config);

        bool setData(
                const OutputData::Ptr& output_data,
                const InputData::Map& input_data_map);

        void start() override;

        void stop() override;

        std::ostream& write(std::ostream& out) const;

    protected:

        ros::NodeHandle nh_;

        ros::Publisher pub_;
        size_t queue_size_{1};
        std::string topic_;

        std::string robot_frame_{"base_link"};
        std::string global_frame_{"map"};

        /**
         * @brief when set to \n
         *   true: it will DELETEALL markers everytime before publishing the
         *         new ones \n
         *       pro: Any number of markers can be viewed. \n
         *       con: It has the side effect of flickering. \n
         *   false: it will update current markers in place. If the number of
         *          markers are less than max_markers_, tiny almost
         *          transparent ARROW markers are added at the end. If the
         *          number of markers are more than max_markers_, only the
         *          first `max_markers_` will be displayed. \n
         *       pro: No flickering \n
         *       con: Limited number of markers.
         */
        bool use_refresh_style_{false};
        size_t max_markers_{50};
        visualization_msgs::Marker default_marker_;

        std::string marker_namespace_;

        void step();

        void pubDeleteAllMarker();

};

} // namespace cabin
