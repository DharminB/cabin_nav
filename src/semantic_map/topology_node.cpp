#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/semantic_map/topology_node.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Polygon2D;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool TopologyNode::initialise(const YAML::Node& node_yaml)
{
    if ( !Parser::read<size_t>(node_yaml, "id", id_) ||
         !Parser::read<Point2D>(node_yaml, "position", position_) )
    {
        std::cerr << Print::Err << Print::Time() << "[TopologyNode] "
                  << "Semantic map contains an node with incorrect format."
                  << Print::End << std::endl;
        std::cout << node_yaml << std::endl;
        std::cout << "Each node in semantic map file should contain" << std::endl
                  << "  - id (unsigned int)" << std::endl
                  << "  - position" << std::endl;
        return false;
    }

    return true;
}

float TopologyNode::distTo(const TopologyNode& other) const
{
    return position_.distTo(other.position_);
}

const size_t TopologyNode::getId() const
{
    return id_;
}

const Point2D& TopologyNode::getPosition() const
{
    return position_;
}

visualization_msgs::Marker TopologyNode::asMarker(
        const std::string& frame,
        float red,
        float green,
        float blue,
        float alpha,
        float diameter) const
{
    return position_.asMarker(frame, red, green, blue, alpha, diameter);
}

std::ostream& operator << (std::ostream& out, const TopologyNode& node)
{
    out << "id: " << node.id_ << std::endl;
    out << "position: " << node.position_ << std::endl;
    return out;
};

YAML::Emitter& operator << (YAML::Emitter& out, const TopologyNode& node)
{
    out << YAML::BeginMap;
    out << YAML::Key << "id";
    out << YAML::Value << node.id_;
    out << YAML::Key << "position";
    out << YAML::Value << YAML::Flow << YAML::BeginMap;
    std::stringstream ss;
    ss << std::setprecision(3) << std::fixed;
    out << YAML::Key << "x" << YAML::Value;
    ss << node.position_.x;
    out << ss.str();
    ss.str(std::string());
    out << YAML::Key << "y" << YAML::Value;
    ss << node.position_.y;
    out << ss.str();
    ss.str(std::string());
    out << YAML::EndMap;
    out << YAML::EndMap;
    return out;
}

} // namespace cabin
