#include <iomanip>
#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/semantic_map/area.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Polygon2D;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool Area::initialise(const YAML::Node& area_yaml)
{
    if ( !Parser::read<std::string>(area_yaml, "type", type_) ||
         !Parser::read<std::string>(area_yaml, "name", name_) ||
         !Parser::read<Polygon2D>(area_yaml, "polygon", polygon_) )
    {
        std::cerr << Print::Err << Print::Time() << "[Area] "
                  << "Semantic map contains an area with incorrect format."
                  << Print::End << std::endl;
        std::cout << area_yaml << std::endl;
        std::cout << "Each area in semantic map file should contain" << std::endl
                  << "  - type" << std::endl
                  << "  - name" << std::endl
                  << "  - polygon" << std::endl;
        return false;
    }

    if ( polygon_.size() < 2 )
    {
        std::cerr << Print::Err << Print::Time() << "[Area] "
                  << name_ << " area contains polygon with less than 2 points"
                  << Print::End << std::endl;
        return false;
    }

    center_ = polygon_.meanPoint();
    if ( !polygon_.isConvex() )
    {
        std::cerr << Print::Err << Print::Time() << "[Area] "
                  << name_ << " is not convex."
                  << Print::End << std::endl;
        return false;
    }
    return true;
}

float Area::distTo(const Area& other) const
{
    return center_.distTo(other.center_);
}

const std::string& Area::getName() const
{
    return name_;
}

const std::string& Area::getType() const
{
    return type_;
}

const kelo::geometry_common::Polygon2D& Area::getPolygon() const
{
    return polygon_;
}

const kelo::geometry_common::Point2D& Area::getCenter() const
{
    return center_;
}

const std::array<float, 3> Area::getcolorFromType(const std::string& type)
{
    std::array<float, 3> rgb =
        ( type == "corridor" )  ? std::array<float, 3>({1.0f, 0.1f, 0.6f}) :
        ( type == "junction" )  ? std::array<float, 3>({0.5f, 0.2f, 1.0f}) :
        ( type == "area" )      ? std::array<float, 3>({0.5f, 0.5f, 0.5f}) :
        ( type == "open_area" ) ? std::array<float, 3>({0.2f, 0.5f, 1.0f}) :
        ( type == "room" )      ? std::array<float, 3>({0.0f, 1.0f, 1.0f}) :
        ( type == "door" )      ? std::array<float, 3>({1.0f, 0.7f, 0.0f}) :
                                  std::array<float, 3>({0.2f, 0.2f, 0.2f});
    return rgb;
}

visualization_msgs::Marker Area::asMarker(
        const std::string& frame,
        float alpha,
        float line_width) const
{
    std::array<float, 3> rgb = Area::getcolorFromType(type_);
    return polygon_.asMarker(frame, rgb[0], rgb[1], rgb[2], alpha, line_width, false);
}

std::ostream& operator << (std::ostream& out, const Area& area)
{
    out << "type: " << area.type_ << std::endl;
    out << "name: " << area.name_ << std::endl;
    out << "center: " << area.center_ << std::endl;
    out << "polygon:" << std::endl;
    for ( size_t i = 0; i < area.polygon_.size(); i++ )
    {
        out << "    " << area.polygon_[i] << std::endl;
    }
    return out;
}

YAML::Emitter& operator << (YAML::Emitter& out, const Area& area)
{
    out << YAML::BeginMap;
    out << YAML::Key << "name";
    out << YAML::Value << area.name_;
    out << YAML::Key << "type";
    out << YAML::Value << area.type_;
    out << YAML::Key << "polygon";
    out << YAML::Value << YAML::BeginSeq;
    std::stringstream ss;
    ss << std::setprecision(3) << std::fixed;
    for ( const Point2D& pt : area.polygon_.vertices )
    {
        out << YAML::Flow << YAML::BeginMap;
        out << YAML::Key << "x" << YAML::Value;
        ss << pt.x;
        out << ss.str();
        ss.str(std::string());
        out << YAML::Key << "y" << YAML::Value;
        ss << pt.y;
        out << ss.str();
        ss.str(std::string());
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
    return out;
}

} // namespace cabin
