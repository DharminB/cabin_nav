#ifndef CABIN_OCCUPANCY_GRID_H
#define CABIN_OCCUPANCY_GRID_H

#include <visualization_msgs/Marker.h>

#include <geometry_common/Box2D.h>
#include <geometry_common/Circle.h>
#include <geometry_common/Point2D.h>
#include <geometry_common/Pose2D.h>
#include <geometry_common/TransformMatrix2D.h>

namespace cabin {

// Forward declaration
using Footprint = std::vector<kelo::geometry_common::Circle>;

class OccupancyGrid
{
    public:

        using Ptr = std::shared_ptr<OccupancyGrid>;

        /**
         * General occupancy grid (mostly used for global planning)
         */
        OccupancyGrid(
                size_t grid_size_x, size_t grid_size_y,
                float grid_cell_size,
                const kelo::geometry_common::TransformMatrix2D& tf);

        /**
         * Used for local planning where the grid is centered around the robot
         */
        OccupancyGrid(float grid_radius, float grid_cell_size);

        OccupancyGrid(const OccupancyGrid& occ_grid);

        virtual ~OccupancyGrid() = default;

        void clear();

        bool calcGridIndex(const kelo::geometry_common::Point2D& point,
                           size_t& i, size_t& j) const;

        /**
         * @brief get point from index
         *
         * @param i
         * @param j
         *
         * @return 
         */
        kelo::geometry_common::Point2D calcPointAt(size_t i, size_t j) const;

        void update(const kelo::geometry_common::PointCloud2D& points);

        void setOccupied(size_t i, size_t j);

        void setUnoccupied(size_t i, size_t j);

        bool isColliding(const kelo::geometry_common::Pose2D& pose) const;

        bool isOccupied(size_t i, size_t j) const;

        bool isOccupied(size_t index) const;

        visualization_msgs::Marker asMarker(
                const std::string& frame = "",
                float red = 1.0f, float green = 0.0f,
                float blue = 0.0f, float alpha = 0.5f) const;

        void generateRobotPts(const kelo::geometry_common::Polygon2D& footprint);

        void generateRobotPts(const kelo::geometry_common::Circle& circle);

        void generateRobotPts(const kelo::geometry_common::Box2D& box);

        float getGridCellSize() const;

        size_t getGridSizeX() const;

        size_t getGridSizeY() const;

        kelo::geometry_common::TransformMatrix2D getTransform() const;

        OccupancyGrid::Ptr calcDilated(float circle_radius) const;

        friend std::ostream& operator << (
                std::ostream& out, const OccupancyGrid& point);

    protected:
        std::vector<bool> grid_;

        kelo::geometry_common::PointVec2D robot_pts_;

        size_t grid_size_x_;
        size_t grid_size_y_;
        float grid_length_;
        float grid_width_;
        float grid_cell_size_;
        float inv_grid_cell_size_;
        float half_grid_cell_size_;
        kelo::geometry_common::TransformMatrix2D tf_, tf_inv_;
};

} // namespace cabin

#endif // CABIN_OCCUPANCY_GRID_H
