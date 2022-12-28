#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/input/occupancy_grid_map_input.h>

using kelo::geometry_common::TransformMatrix2D;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool OccupancyGridMapInput::configure(const YAML::Node& config)
{
    if ( !Parser::read<std::string>(config, "topic", topic_) )
    {
        std::cerr << Print::Err << Print::Time() << "[occupancy_grid_map_input] "
                  << "topic not provided"
                  << Print::End << std::endl;
        return false;
    }
    always_active_ = Parser::get<bool>(config, "always_active", false);
    return true;
}

bool OccupancyGridMapInput::getData(InputData::Ptr& input_data,
                                     const std::string& input_name)
{
    std::lock_guard<std::mutex> guard(mutex_);
    if ( input_data->occ_grid_map ) // already filled
    {
        return true;
    }

    // TODO: maybe downsample the grid to be more managable for planning???
    input_data->occ_grid_map = std::make_shared<OccupancyGrid>(
                static_cast<size_t>(occ_grid_.info.width),
                static_cast<size_t>(occ_grid_.info.height),
                static_cast<float>(occ_grid_.info.resolution),
                TransformMatrix2D(occ_grid_.info.origin));

    for ( size_t i = 0; i < occ_grid_.info.width; i++ )
    {
        for ( size_t j = 0; j < occ_grid_.info.height; j++ )
        {
            if ( occ_grid_.data[(j * occ_grid_.info.width) + i] != 0 )
            {
                input_data->occ_grid_map->setOccupied(i, j);
            }
        }
    }
    return true;
}

void OccupancyGridMapInput::activate()
{
    if ( is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[OccupancyGridMapInput] "
                  << "Already active. Ignoring activate() call"
                  << Print::End << std::endl;
        return;
    }

    is_active_ = true;
    sub_ = nh_.subscribe(topic_, 1, &OccupancyGridMapInput::occGridMapCb, this);
}

void OccupancyGridMapInput::deactivate()
{
    if ( !is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[OccupancyGridMapInput] "
                  << "Already inactive. Ignoring deactivate() call"
                  << Print::End << std::endl;
        return;
    }

    if ( always_active_ )
    {
        return;
    }

    is_active_ = false;
    sub_.shutdown();
}

void OccupancyGridMapInput::occGridMapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    std::lock_guard<std::mutex> guard(mutex_);
    occ_grid_ = nav_msgs::OccupancyGrid(*msg);
    sub_.shutdown(); // single shot subscription
}

} // namespace cabin
