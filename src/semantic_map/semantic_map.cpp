#include <queue>

#include <geometry_common/Utils.h>
#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/semantic_map/semantic_map.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Polygon2D;
using kelo::geometry_common::LineSegment2D;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool SemanticMap::initialise(const std::string& semantic_map_file)
{
    YAML::Node semantic_map_yaml;
    if ( !Parser::loadFile(semantic_map_file, semantic_map_yaml, true) )
    {
        return false;
    }

    if ( !Parser::hasKey(semantic_map_yaml, "areas") ||
         !Parser::hasKey(semantic_map_yaml, "connections") ||
         !Parser::hasKey(semantic_map_yaml, "topology_nodes") ||
         !Parser::hasKey(semantic_map_yaml, "topology_connections") )
    {
        std::cerr << Print::Err << Print::Time() << "[SemanticMap] "
                  << "Semantic map file does not have areas, connections, "
                  << "topology_nodes and/or topology_connections."
                  << Print::End << std::endl;
        return false;
    }

    /* read individual areas */
    const YAML::Node& areas_yaml = semantic_map_yaml["areas"];
    for ( const YAML::Node& area_yaml : areas_yaml )
    {
        Area::Ptr area = std::make_shared<Area>();
        if ( !area->initialise(area_yaml) )
        {
            areas_.clear();
            return false;
        }
        else if ( areas_.find(area->getName()) != areas_.end() )
        {
            std::cerr << Print::Err << Print::Time() << "[SemanticMap] "
                      << "Semantic map contains multiple areas with same name."
                      << std::endl << "Area name: " << area->getName()
                      << Print::End << std::endl;
            areas_.clear();
            return false;
        }
        else
        {
            areas_[area->getName()] = area;
        }
    }

    /* read individual connections */
    const YAML::Node& connections_yaml = semantic_map_yaml["connections"];
    connections_.reserve(connections_yaml.size());
    for ( const YAML::Node& connection_yaml : connections_yaml )
    {
        Connection connection;
        if ( connection.initialise(connection_yaml) &&
             areas_.find(connection.area_1) != areas_.end() &&
             areas_.find(connection.area_2) != areas_.end() )
        {
            connections_.push_back(connection);
        }
        else
        {
            std::cerr << Print::Err << Print::Time() << "[SemanticMap] "
                      << "Could not initialise a connection"
                      << std::endl << "Connection: " << connection_yaml
                      << Print::End << std::endl;
            areas_.clear();
            connections_.clear();
            return false;
        }
    }

    /* read individual topology nodes */
    const YAML::Node& topology_nodes_yaml = semantic_map_yaml["topology_nodes"];
    for ( const YAML::Node& node_yaml : topology_nodes_yaml )
    {
        TopologyNode::Ptr node = std::make_shared<TopologyNode>();
        if ( !node->initialise(node_yaml) )
        {
            topology_nodes_.clear();
            return false;
        }
        else if ( topology_nodes_.find(node->getId()) != topology_nodes_.end() )
        {
            std::cerr << Print::Err << Print::Time() << "[SemanticMap] "
                      << "Semantic map contains multiple topology nodes with same id."
                      << std::endl << "TopologyNode id: " << node->getId()
                      << Print::End << std::endl;
            topology_nodes_.clear();
            return false;
        }
        else
        {
            topology_nodes_[node->getId()] = node;
        }
    }

    /* read individual topology connection */
    const YAML::Node& topology_connections_yaml = semantic_map_yaml["topology_connections"];
    topology_connections_.reserve(topology_connections_yaml.size());
    for ( const YAML::Node& connection_yaml : topology_connections_yaml )
    {
        TopologyConnection connection;
        if ( connection.initialise(connection_yaml) &&
             topology_nodes_.find(connection.node_1) != topology_nodes_.end() &&
             topology_nodes_.find(connection.node_2) != topology_nodes_.end() )
        {
            topology_connections_.push_back(connection);
        }
        else
        {
            std::cerr << Print::Err << Print::Time() << "[SemanticMap] "
                      << "Could not initialise a topology connection"
                      << std::endl << "Connection: " << connection_yaml
                      << Print::End << std::endl;
            topology_nodes_.clear();
            topology_connections_.clear();
            return false;
        }
    }

    std::cout << Print::Success << Print::Time() << "[SemanticMap] "
              << "Initialised " << *this
              << Print::End << std::endl;
    is_initialised_ = true;
    return true;
}

std::string SemanticMap::getAreaContainingPoint(
        const kelo::geometry_common::Point2D& point) const
{
    for ( auto it = areas_.begin(); it != areas_.end(); it ++ )
    {
        if ( it->second->getPolygon().containsPoint(point) )
        {
            return it->first;
        }
    }

    /* if point is in none of the areas, return the area whose boundary is closest */
    float min_dist = std::numeric_limits<float>::max();
    std::string min_dist_area_name;
    Point2D intersection_pt;
    LineSegment2D l;
    l.start = point;
    for ( auto it = areas_.begin(); it != areas_.end(); it ++ )
    {
        l.end = it->second->getCenter();
        if ( !it->second->getPolygon().calcClosestIntersectionPointWith(l, intersection_pt) )
        {
            continue;
        }

        float dist = point.distTo(intersection_pt);
        if ( dist < min_dist )
        {
            min_dist = dist;
            min_dist_area_name = it->first;
        }
    }
    return min_dist_area_name;
}

std::vector<Area::ConstPtr> SemanticMap::plan(
        const std::string& start,
        const std::string& goal) const
{
    std::vector<Area::ConstPtr> connection_path;
    std::priority_queue<AreaNode, std::vector<AreaNode>,
        std::function<bool(const AreaNode&, const AreaNode&)> > fringe(SemanticMap::greaterAreaNode);

    std::map<std::string, bool> closed;
    std::map<std::string, std::string> parent;
    for ( auto itr = areas_.begin(); itr != areas_.end(); itr++ )
    {
        closed[itr->first] = false;
    }
    AreaNode start_node;
    start_node.area_name = start;
    start_node.g = 0.0f;
    start_node.h = areas_.at(start)->distTo(*(areas_.at(goal)));
    start_node.f = start_node.g + start_node.h;
    parent[start] = start;
    fringe.push(start_node);

    while ( !fringe.empty() )
    {
        AreaNode current = fringe.top();
        fringe.pop();

        /* check for repetition */
        if ( closed[current.area_name] )
        {
            continue;
        }

        /* goal test */
        if ( goal == current.area_name )
        {
            /* backtrack to populate connection_path */
            for ( std::string current_area_name = current.area_name;
                  current_area_name != start;
                  current_area_name = parent[current_area_name] )
            {
                connection_path.push_back(areas_.at(current_area_name));
            }
            connection_path.push_back(areas_.at(start));
            std::reverse(connection_path.begin(), connection_path.end());
            return connection_path;
        }

        closed[current.area_name] = true;

        /* generate neighbours and add to fringe */
        for ( const Connection& connection : connections_ )
        {
            if ( connection.area_1 != current.area_name &&
                 connection.area_2 != current.area_name )
            {
                continue;
            }
            std::string connected_area = ( connection.area_1 == current.area_name )
                                         ? connection.area_2 : connection.area_1;
            if ( closed[connected_area] )
            {
                continue;
            }
            AreaNode neighbour;
            neighbour.area_name = connected_area;
            neighbour.g = current.g + areas_.at(connected_area)->distTo(*(areas_.at(current.area_name)));
            neighbour.h = areas_.at(connected_area)->distTo(*(areas_.at(goal)));
            neighbour.f = neighbour.g + neighbour.h;
            parent[neighbour.area_name] = current.area_name;
            fringe.push(neighbour);
        }
    }
    std::cerr << Print::Err << Print::Time() << "[SemanticMap] "
              << "Could not search a connected path from " << start
              << " to " << goal
              << Print::End << std::endl;
    return connection_path;
}

Area::ConstPtr SemanticMap::getAreaNamed(const std::string& area_name) const
{
    if ( areas_.find(area_name) != areas_.end() )
    {
        return areas_.at(area_name);
    }
    return nullptr;
}

bool SemanticMap::isValid() const
{
    return ( is_initialised_ && !areas_.empty() );
}

std::ostream& operator << (std::ostream& out, const SemanticMap& semantic_map)
{
    out << "<SemanticMap: "
        << semantic_map.areas_.size() << " areas, "
        << semantic_map.connections_.size() << " connections, "
        << semantic_map.topology_nodes_.size() << " topology nodes & "
        << semantic_map.topology_connections_.size() << " topology connections>";
    return out;
}

YAML::Emitter& operator << (YAML::Emitter& out, const SemanticMap& semantic_map)
{
    out << YAML::BeginMap;

    out << YAML::Key << "areas";
    out << YAML::Value << YAML::BeginSeq;
    for ( auto it = semantic_map.areas_.begin();
          it != semantic_map.areas_.end();
          it ++ )
    {
        out << *(it->second);
    }
    out << YAML::EndSeq;

    out << YAML::Key << "connections";
    out << YAML::Value << YAML::BeginSeq;
    for ( const Connection& connection : semantic_map.connections_ )
    {
        out << connection;
    }
    out << YAML::EndSeq;

    out << YAML::Key << "topology_nodes";
    out << YAML::Value << YAML::BeginSeq;
    for ( auto it = semantic_map.topology_nodes_.begin();
          it != semantic_map.topology_nodes_.end();
          it ++ )
    {
        out << *(it->second);
    }
    out << YAML::EndSeq;

    out << YAML::Key << "topology_connections";
    out << YAML::Value << YAML::BeginSeq;
    for ( const TopologyConnection& connection : semantic_map.topology_connections_ )
    {
        out << connection;
    }
    out << YAML::EndSeq;

    out << YAML::EndMap;
    return out;
}

} // namespace cabin
