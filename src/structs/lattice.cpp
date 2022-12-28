#include <cabin_nav/utils/print.h>
#include <cabin_nav/structs/lattice.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Pose2D;
using kelo::geometry_common::TransformMatrix2D;

namespace cabin {

template <typename T>
Lattice<T>::Lattice(const LatticeConfig& config, T default_value):
    grid_size_x_(config.grid_size_x),
    grid_size_y_(config.grid_size_y),
    grid_cell_size_(config.grid_cell_size),
    num_of_angles_(config.num_of_angles),
    default_value_(default_value),
    tf_(config.tf)
{
    grid_length_ = grid_size_x_ * grid_cell_size_;
    grid_width_ = grid_size_y_ * grid_cell_size_;

    lattice_ = std::vector<T>(grid_size_x_ * grid_size_y_ * num_of_angles_,
                              default_value_);

    inv_grid_cell_size_ = 1.0f/grid_cell_size_;
    half_grid_cell_size_ = grid_cell_size_/2;

    angle_cell_size_ = (2*M_PI)/num_of_angles_;
    inv_angle_cell_size_ = 1.0f/angle_cell_size_;
    angle_offset_ = angle_cell_size_/2;

    tf_inv_.update(tf_.calcInverse());
}
template Lattice<int>::Lattice(const LatticeConfig& config, int default_value);
template Lattice<float>::Lattice(const LatticeConfig& config, float default_value);
template Lattice<size_t>::Lattice(const LatticeConfig& config, size_t default_value);
template Lattice<bool>::Lattice(const LatticeConfig& config, bool default_value);



template <typename T>
Lattice<T>::Lattice(const Lattice<T>& lattice)
{
    grid_size_x_ = lattice.grid_size_x_;
    grid_size_y_ = lattice.grid_size_y_;
    grid_length_ = lattice.grid_length_;
    grid_width_ = lattice.grid_width_;
    grid_cell_size_ = lattice.grid_cell_size_;
    inv_grid_cell_size_ = lattice.inv_grid_cell_size_;
    grid_offset_ = lattice.grid_offset_;
    half_grid_cell_size_ = lattice.half_grid_cell_size_;
    angle_cell_size_ = lattice.angle_cell_size_;
    inv_angle_cell_size_ = lattice.inv_angle_cell_size_;
    angle_offset_ = lattice.angle_offset_;
    num_of_angles_ = lattice.num_of_angles_;
    lattice_ = std::vector<T>(lattice.lattice_);
    tf_.update(lattice.tf_);
    tf_inv_.update(lattice.tf_inv_);
}
template Lattice<int>::Lattice(const Lattice<int>& lattice);
template Lattice<float>::Lattice(const Lattice<float>& lattice);
template Lattice<size_t>::Lattice(const Lattice<size_t>& lattice);
template Lattice<bool>::Lattice(const Lattice<bool>& lattice);



/* get index from point */
template <typename T>
bool Lattice<T>::calcGridIndex(const Pose2D& pose, LatticeIndex& lat_ind) const
{
    Pose2D transformed_pose = tf_inv_ * pose;
    if ( transformed_pose.x < 0.0f || transformed_pose.x > grid_length_ ||
         transformed_pose.y < 0.0f || transformed_pose.y > grid_width_ )
    {
        return false;
    }
    lat_ind.i = transformed_pose.x * inv_grid_cell_size_;
    lat_ind.j = transformed_pose.y * inv_grid_cell_size_;
    lat_ind.k = (transformed_pose.theta + M_PI + angle_offset_) * inv_angle_cell_size_;
    lat_ind.k %= num_of_angles_; // TODO: maybe find a way to not use % for efficiency
    return true;
}
template bool Lattice<int>::calcGridIndex(const Pose2D& pose, LatticeIndex& lat_ind) const;
template bool Lattice<float>::calcGridIndex(const Pose2D& pose, LatticeIndex& lat_ind) const;
template bool Lattice<size_t>::calcGridIndex(const Pose2D& pose, LatticeIndex& lat_ind) const;
template bool Lattice<bool>::calcGridIndex(const Pose2D& pose, LatticeIndex& lat_ind) const;



/* get pose from index */
template <typename T>
Pose2D Lattice<T>::calcPoseAt(const LatticeIndex& lat_ind) const
{
    return tf_ * Pose2D(
            (static_cast<float>(lat_ind.i) * grid_cell_size_) + half_grid_cell_size_,
            (static_cast<float>(lat_ind.j) * grid_cell_size_) + half_grid_cell_size_,
            (static_cast<float>(lat_ind.k) * angle_cell_size_) - M_PI);
}
template Pose2D Lattice<int>::calcPoseAt(const LatticeIndex& lat_ind) const;
template Pose2D Lattice<float>::calcPoseAt(const LatticeIndex& lat_ind) const;
template Pose2D Lattice<size_t>::calcPoseAt(const LatticeIndex& lat_ind) const;
template Pose2D Lattice<bool>::calcPoseAt(const LatticeIndex& lat_ind) const;



template <typename T>
std::vector<visualization_msgs::Marker> Lattice<T>::asMarker(
        const std::string& frame,
        float red, float green, float blue, float alpha) const
{
    std::vector<visualization_msgs::Marker> markers;
    float arrow_width = grid_cell_size_ * 0.1f;
    float arrow_length = 5 * arrow_width;
    for ( size_t i = 0; i < grid_size_x_; i++ )
    {
        for ( size_t j = 0; j < grid_size_y_; j++ )
        {
            for ( size_t k = 0; k < num_of_angles_; k++ )
            {
                LatticeIndex lat_ind;
                lat_ind.i = i;
                lat_ind.j = j;
                lat_ind.k = k;
                if ( getValueAt(lat_ind) == default_value_ )
                {
                    continue;
                }
                Pose2D pose = calcPoseAt(lat_ind);
                markers.push_back(pose.asMarker(
                            frame, red, green, blue, alpha,
                            arrow_length, arrow_width, arrow_width));
            }
        }
    }
    return markers;
}
template std::vector<visualization_msgs::Marker> Lattice<int>::asMarker(
        const std::string& frame, float red, float green, float blue, float alpha) const;
template std::vector<visualization_msgs::Marker> Lattice<float>::asMarker(
        const std::string& frame, float red, float green, float blue, float alpha) const;
template std::vector<visualization_msgs::Marker> Lattice<size_t>::asMarker(
        const std::string& frame, float red, float green, float blue, float alpha) const;
template std::vector<visualization_msgs::Marker> Lattice<bool>::asMarker(
        const std::string& frame, float red, float green, float blue, float alpha) const;


} // namespace cabin
