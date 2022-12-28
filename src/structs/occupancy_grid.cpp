#include <geometry_common/Utils.h>
#include <geometry_common/Polygon2D.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/structs/occupancy_grid.h>

using kelo::geometry_common::Box2D;
using kelo::geometry_common::Circle;
using kelo::geometry_common::Point2D;
using kelo::geometry_common::PointVec2D;
using kelo::geometry_common::PointCloud2D;
using kelo::geometry_common::Polygon2D;
using kelo::geometry_common::Pose2D;
using kelo::geometry_common::TransformMatrix2D;
using GCUtils = kelo::geometry_common::Utils;

namespace cabin {

OccupancyGrid::OccupancyGrid(size_t grid_size_x, size_t grid_size_y,
                             float grid_cell_size, const TransformMatrix2D& tf):
    grid_size_x_(grid_size_x),
    grid_size_y_(grid_size_y),
    grid_cell_size_(grid_cell_size),
    tf_(tf)
{
    grid_length_ = grid_size_x_ * grid_cell_size_;
    grid_width_ = grid_size_y_ * grid_cell_size_;
    grid_ = std::vector<bool>(grid_size_x_*grid_size_y_, false); // everything is free
    inv_grid_cell_size_ = 1.0f/grid_cell_size_;
    half_grid_cell_size_ = grid_cell_size_/2;
    tf_inv_.update(tf_.calcInverse());
}

OccupancyGrid::OccupancyGrid(float grid_radius, float grid_cell_size):
    OccupancyGrid(2*(size_t)(grid_radius/grid_cell_size) + 1,
                  2*(size_t)(grid_radius/grid_cell_size) + 1,
                  grid_cell_size,
                  TransformMatrix2D(
                      -(grid_cell_size * (2*(size_t)(grid_radius/grid_cell_size) + 1))/2,
                      -(grid_cell_size * (2*(size_t)(grid_radius/grid_cell_size) + 1))/2,
                      0.0f))
{
}

OccupancyGrid::OccupancyGrid(const OccupancyGrid& occ_grid)
{
    grid_size_x_ = occ_grid.grid_size_x_;
    grid_size_y_ = occ_grid.grid_size_y_;
    grid_length_= occ_grid.grid_length_;
    grid_width_ = occ_grid.grid_width_;
    grid_cell_size_ = occ_grid.grid_cell_size_;
    inv_grid_cell_size_ = occ_grid.inv_grid_cell_size_;
    half_grid_cell_size_ = occ_grid.half_grid_cell_size_;
    tf_ = occ_grid.tf_;
    tf_inv_ = occ_grid.tf_inv_;
    grid_ = std::vector<bool>(occ_grid.grid_);
    robot_pts_ = PointVec2D(occ_grid.robot_pts_);
}

void OccupancyGrid::clear()
{
    std::fill(grid_.begin(), grid_.end(), false);
}

/* get index from point */
bool OccupancyGrid::calcGridIndex(const Point2D& point, size_t& i, size_t& j) const
{
    Point2D transformed_pt = tf_inv_ * point;
    if ( transformed_pt.x < 0.0f || transformed_pt.x > grid_length_ ||
         transformed_pt.y < 0.0f || transformed_pt.y > grid_width_ )
    {
        return false;
    }
    i = transformed_pt.x * inv_grid_cell_size_;
    j = transformed_pt.y * inv_grid_cell_size_;
    return true;
}

Point2D OccupancyGrid::calcPointAt(size_t i, size_t j) const
{
    return tf_ * Point2D(
            (static_cast<float>(i) * grid_cell_size_) + half_grid_cell_size_,
            (static_cast<float>(j) * grid_cell_size_) + half_grid_cell_size_);
}

void OccupancyGrid::update(const PointCloud2D& points)
{
    size_t i = 0, j = 0;
    for ( const Point2D& pt : points )
    {
        if ( calcGridIndex(pt, i, j) )
        {
            grid_[(i*grid_size_y_)+j] = true;
        }
    }
}

void OccupancyGrid::setOccupied(size_t i, size_t j)
{
    size_t index = (i*grid_size_y_)+j;
    if ( index < grid_.size() )
    {
        grid_[index] = true;
    }
}

void OccupancyGrid::setUnoccupied(size_t i, size_t j)
{
    size_t index = (i*grid_size_y_)+j;
    if ( index < grid_.size() )
    {
        grid_[index] = false;
    }
}

bool OccupancyGrid::isColliding(const Pose2D& pose) const
{
    size_t i = 0, j = 0;
    TransformMatrix2D tf(pose);
    for ( const Point2D& pt : robot_pts_ )
    {
        Point2D transformed_pt = tf * pt;
        if ( calcGridIndex(transformed_pt, i, j) && isOccupied(i, j) )
        {
            return true;
        }
    }
    return false;
}

bool OccupancyGrid::isOccupied(size_t i, size_t j) const
{
    return isOccupied((i*grid_size_y_)+j);
}

bool OccupancyGrid::isOccupied(size_t index) const
{
    if ( index >= grid_.size() ) // lies outside the grid and hence is not colliding
    {
        return false;
    }
    return grid_[index];
}

visualization_msgs::Marker OccupancyGrid::asMarker(
        const std::string& frame,
        float red, float green, float blue, float alpha) const
{
    visualization_msgs::Marker grid_marker;
    grid_marker.header.frame_id = frame;
    grid_marker.type = visualization_msgs::Marker::CUBE_LIST;
    grid_marker.pose.orientation.w = 1.0f;
    grid_marker.scale.x = grid_cell_size_*0.99f;
    grid_marker.scale.y = grid_cell_size_*0.99f;
    grid_marker.scale.z = 0.05f;
    grid_marker.color.r = red;
    grid_marker.color.g = green;
    grid_marker.color.b = blue;
    grid_marker.color.a = alpha;
    for ( size_t i = 0; i < grid_size_x_; i++ )
    {
        for ( size_t j = 0; j < grid_size_y_; j++ )
        {
            if ( !grid_[(i*grid_size_y_)+j] )
            {
                continue;
            }
            grid_marker.points.push_back(calcPointAt(i, j).asPoint());
        }
    }
    return grid_marker;
}

void OccupancyGrid::generateRobotPts(const Polygon2D& footprint)
{
    generateRobotPts(Box2D(footprint));
}

void OccupancyGrid::generateRobotPts(const Circle& circle)
{
    Box2D box;
    box.max_x = circle.x + circle.r;
    box.max_y = circle.y + circle.r;
    box.min_x = circle.x - circle.r;
    box.min_y = circle.y - circle.r;
    generateRobotPts(box);
}

void OccupancyGrid::generateRobotPts(const Box2D& box)
{
    float robot_length = box.max_x - box.min_x;
    float robot_width = box.max_y - box.min_y;
    size_t num_of_x_pts = std::ceil(robot_length/grid_cell_size_) + 1;
    size_t num_of_y_pts = std::ceil(robot_width/grid_cell_size_) + 1;
    float delta_x = robot_length/(num_of_x_pts-1);
    float delta_y = robot_width/(num_of_y_pts-1);
    robot_pts_.clear();
    robot_pts_.reserve(num_of_x_pts * num_of_y_pts);
    for ( size_t i = 0; i < num_of_x_pts; i++ )
    {
        for ( size_t j = 0; j < num_of_y_pts; j++ )
        {
            robot_pts_.push_back(Point2D(box.min_x + (i*delta_x),
                                         box.min_y + (j*delta_y)));
        }
    }
}

float OccupancyGrid::getGridCellSize() const
{
    return grid_cell_size_;
}

size_t OccupancyGrid::getGridSizeX() const
{
    return grid_size_x_;
}

size_t OccupancyGrid::getGridSizeY() const
{
    return grid_size_y_;
}

TransformMatrix2D OccupancyGrid::getTransform() const
{
    return tf_;
}

OccupancyGrid::Ptr OccupancyGrid::calcDilated(float circle_radius) const
{
    // create a convolution mask
    OccupancyGrid conv_mask(circle_radius, grid_cell_size_);
    size_t conv_mask_radius = conv_mask.getGridSizeX()/2;
    size_t conv_mask_radius_sq = std::pow(conv_mask_radius, 2);
    int conv_mask_center_index = conv_mask_radius;
    for ( size_t i = 0; i < conv_mask.getGridSizeX(); i++ )
    {
        for ( size_t j = 0; j < conv_mask.getGridSizeY(); j++ )
        {
            size_t dist_sq = std::pow(static_cast<int>(i) - conv_mask_center_index, 2) +
                             std::pow(static_cast<int>(j) - conv_mask_center_index, 2);
            if ( dist_sq < conv_mask_radius_sq )
            {
                conv_mask.setOccupied(i, j);
            }
        }
    }

    OccupancyGrid::Ptr dilated_occ_grid = OccupancyGrid::Ptr( new OccupancyGrid(*this) );
    for ( size_t i = 1; i+1 < grid_size_x_; i++ )
    {
        for ( size_t j = 1; j+1 < grid_size_y_; j++ )
        {
            if ( !isOccupied(i, j) )
            {
                continue;
            }
            if ( isOccupied(i+1, j) && isOccupied(i, j+1) &&
                 isOccupied(i-1, j) && isOccupied(i, j-1) )
            {
                continue;
            }
            // apply conv mask
            for ( size_t ci = 0; ci < conv_mask.getGridSizeX(); ci++ )
            {
                for ( size_t cj = 0; cj < conv_mask.getGridSizeY(); cj++ )
                {
                    if ( !conv_mask.isOccupied(ci, cj) )
                    {
                        continue;
                    }
                    dilated_occ_grid->setOccupied(i + ci - conv_mask_radius,
                                                  j + cj - conv_mask_radius);
                }
            }
        }
    }
    return dilated_occ_grid;
}

std::ostream& operator << (std::ostream& out, const OccupancyGrid& occ_grid)
{
    for ( size_t i = 0; i < occ_grid.grid_size_x_; i++ )
    {
        for ( size_t j = 0; j < occ_grid.grid_size_y_; j++ )
        {
            if ( occ_grid.grid_[(i*occ_grid.grid_size_y_)+j] )
            {
                out << "# ";
            }
            else
            {
                out << "- ";
            }
        }
        out << std::endl;
    }
    return out;
}

} // namespace cabin
