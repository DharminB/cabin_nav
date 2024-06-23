#pragma once

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <visualization_msgs/Marker.h>

#include <geometry_common/Point2D.h>

#include <cabin_nav/utils/utils.h>

namespace cabin {

class TopologyNode
{
    public:

        using Ptr = std::shared_ptr<TopologyNode>;
        using ConstPtr = std::shared_ptr<const TopologyNode>;

        TopologyNode() = default;

        virtual ~TopologyNode() = default;

        bool initialise(const YAML::Node& node_yaml);

        float distTo(const TopologyNode& other) const;

        const size_t getId() const;

        const kelo::geometry_common::Point2D& getPosition() const;

        visualization_msgs::Marker asMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float diameter = 0.2f) const;

        friend std::ostream& operator << (
                std::ostream& out,
                const TopologyNode& node);

        friend YAML::Emitter& operator << (
                YAML::Emitter& out,
                const TopologyNode& node);

    private:

        size_t id_;
        kelo::geometry_common::Point2D position_;

        friend class SemanticMapEditor;
};

} // namespace cabin
