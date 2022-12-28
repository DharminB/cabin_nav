#ifndef CABIN_AREA_H
#define CABIN_AREA_H

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <visualization_msgs/Marker.h>

#include <geometry_common/Point2D.h>
#include <geometry_common/Polygon2D.h>

#include <cabin_nav/utils/utils.h>

namespace cabin {

class Area
{
    public:

        using Ptr = std::shared_ptr<Area>;
        using ConstPtr = std::shared_ptr<const Area>;

        bool initialise(const YAML::Node& area_yaml);

        float distTo(const Area& other) const;

        const std::string& getName() const;

        const std::string& getType() const;

        const kelo::geometry_common::Polygon2D& getPolygon() const;

        const kelo::geometry_common::Point2D& getCenter() const;

        visualization_msgs::Marker asMarker(
                const std::string& frame = "base_link",
                float alpha = 0.8f,
                float line_width = 0.05f) const;

        friend std::ostream& operator << (std::ostream& out, const Area& area);

        friend YAML::Emitter& operator << (YAML::Emitter& out, const Area& area);

    private:

        std::string type_;
        std::string name_;
        kelo::geometry_common::Polygon2D polygon_;
        kelo::geometry_common::Point2D center_;

        static const std::array<float, 3> getcolorFromType(
                const std::string& type);

        friend class SemanticMapEditor;
};

} // namespace cabin
#endif // CABIN_AREA_H
