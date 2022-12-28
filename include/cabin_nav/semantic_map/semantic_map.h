#ifndef CABIN_SEMANTIC_MAP_H
#define CABIN_SEMANTIC_MAP_H

#include <yaml-cpp/yaml.h>

#include <cabin_nav/semantic_map/area.h>
#include <cabin_nav/semantic_map/connection.h>
#include <cabin_nav/semantic_map/topology_node.h>
#include <cabin_nav/semantic_map/topology_connection.h>

namespace cabin {

class SemanticMap
{
    public:

        using Ptr = std::shared_ptr<SemanticMap>;
        using ConstPtr = std::shared_ptr<const SemanticMap>;

        SemanticMap() = default;

        virtual ~SemanticMap() = default;

        bool initialise(const std::string& semantic_map_file);

        std::string getAreaContainingPoint(
                const kelo::geometry_common::Point2D& point) const;

        std::vector<Area::ConstPtr> plan(
                const std::string& start,
                const std::string& goal) const;

        Area::ConstPtr getAreaNamed(const std::string& area_name) const;

        bool isValid() const;

        friend std::ostream& operator << (
                std::ostream& out,
                const SemanticMap& semantic_map);

        friend YAML::Emitter& operator << (
                YAML::Emitter& out,
                const SemanticMap& semantic_map);

    private:

        std::map<std::string, Area::Ptr> areas_;
        std::vector<Connection> connections_;
        std::map<size_t, TopologyNode::Ptr> topology_nodes_;
        std::vector<TopologyConnection> topology_connections_;

        bool is_initialised_{false};

        struct AreaNode
        {
            std::string area_name;
            float g, h, f;
        };

        static bool greaterAreaNode(const AreaNode& n1, const AreaNode& n2)
        {
            return ( n1.f > n2.f );
        }

        friend class SemanticMapEditor;

};

} // namespace cabin
#endif // CABIN_SEMANTIC_MAP_H
