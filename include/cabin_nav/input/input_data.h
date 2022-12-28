#ifndef CABIN_INPUT_DATA_H
#define CABIN_INPUT_DATA_H

#include <vector>
#include <unordered_map>
#include <regex>

#include <geometry_common/XYTheta.h>
#include <geometry_common/Point2D.h>
#include <geometry_common/Point3D.h>
#include <geometry_common/TransformMatrix2D.h>

#include <cabin_nav/structs/joypad.h>
#include <cabin_nav/structs/img_data.h>
#include <cabin_nav/structs/occupancy_grid.h>
#include <cabin_nav/semantic_map/semantic_map.h>

namespace cabin {

struct InputData
{

    using Ptr = std::shared_ptr<InputData>;
    using ConstPtr = std::shared_ptr<const InputData>;


    kelo::geometry_common::TransformMatrix2D localisation_tf;

    kelo::geometry_common::Velocity2D current_vel;

    kelo::geometry_common::PointCloud2D laser_pts;

    std::unordered_map<std::string, kelo::geometry_common::PointCloud3D> pointcloud_data;

    std::unordered_map<std::string, ImgData> img_data;

    Joypad joypad;

    SemanticMap::ConstPtr semantic_map{nullptr};

    float max_wheel_current;

    OccupancyGrid::Ptr occ_grid_map;



    friend std::ostream& operator << (std::ostream& out, const InputData& input_data)
    {
        out << "laser_pts #: " << input_data.laser_pts.size() << std::endl;

        out << "img_data #: " << input_data.img_data.size() << std::endl;
        for ( auto itr = input_data.img_data.begin();
              itr != input_data.img_data.end(); itr ++ )
        {
            std::ostringstream img_data_stream;
            img_data_stream << "  " << itr->first << ":\n" << itr->second;
            std::string img_data_string = img_data_stream.str();
            img_data_string = std::regex_replace(img_data_string, std::regex("\n"), "\n    ");
            out << img_data_string << std::endl;
        }

        out << "pointcloud_data #: " << input_data.pointcloud_data.size() << std::endl;
        for ( auto itr = input_data.pointcloud_data.begin();
              itr != input_data.pointcloud_data.end(); itr ++ )
        {
            out << "  " << itr->first
                << " #: " << itr->second.size() << std::endl;
        }

        out << "joypad: " << std::endl;
        std::ostringstream joypad_stream;
        joypad_stream << "  " << input_data.joypad;
        std::string joypad_string = joypad_stream.str();
        joypad_string = std::regex_replace(joypad_string, std::regex("\n"), "\n  ");
        out << joypad_string << std::endl;

        if ( input_data.semantic_map != nullptr )
        {
            out << "semantic_map: " << *(input_data.semantic_map) << std::endl;
        }

        out << "max_wheel_current: " << input_data.max_wheel_current << std::endl;

        if ( input_data.occ_grid_map != nullptr )
        {
            out << "occ_grid: " << input_data.occ_grid_map->getGridSizeX()
                                << " x " << input_data.occ_grid_map->getGridSizeY();
        }

        return out;
    };


};

} // namespace cabin

#endif // CABIN_INPUT_DATA_H
