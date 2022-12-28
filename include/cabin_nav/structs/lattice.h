#ifndef CABIN_LATTICE_H
#define CABIN_LATTICE_H

#include <visualization_msgs/Marker.h>

#include <geometry_common/Point2D.h>
#include <geometry_common/Pose2D.h>
#include <geometry_common/TransformMatrix2D.h>

#include <cabin_nav/structs/lattice_index.h>
#include <cabin_nav/structs/lattice_config.h>

namespace cabin {

template <typename T>
class Lattice
{
    public:
        Lattice(const LatticeConfig& config, T default_value);

        Lattice(const Lattice<T>& lattice);

        virtual ~Lattice() = default;

        void clear()
        {
            std::fill(lattice_.begin(), lattice_.end(), default_value_);
        };

        bool calcGridIndex(
                const kelo::geometry_common::Pose2D& pose,
                LatticeIndex& lat_ind) const;

        kelo::geometry_common::Pose2D calcPoseAt(const LatticeIndex& lat_ind) const;

        void setValueAt(const LatticeIndex& lat_ind, T value)
        {
            setValueAt(calcIndex(lat_ind), value);
        };

        void setValueAt(size_t index, T value)
        {
            if ( index < lattice_.size() )
            {
                lattice_[index] = value;
            }
        };

        T getValueAt(const LatticeIndex& lat_ind) const
        {
            return getValueAt(calcIndex(lat_ind));
        };

        T getValueAt(size_t index) const
        {
            if ( index < lattice_.size() )
            {
                return lattice_[index];
            }
            return default_value_;
        };

        inline size_t calcIndex(const LatticeIndex& lat_ind) const
        {
            size_t index = (((lat_ind.i*grid_size_y_)+lat_ind.j)*num_of_angles_) + lat_ind.k;
            if ( lat_ind.i < grid_size_x_ &&
                 lat_ind.j < grid_size_y_ &&
                 index < lattice_.size() )
            {
                return index;
            }
            else
            {
                return (size_t)-1;
            }
        };

        inline LatticeIndex calcLatticeIndex(size_t index) const
        {
            LatticeIndex lat_ind;
            lat_ind.k = index % num_of_angles_;
            index /= num_of_angles_;
            lat_ind.j = index % grid_size_y_;
            index /= grid_size_y_;
            lat_ind.i = index;
            return lat_ind;
        };

        std::vector<visualization_msgs::Marker> asMarker(
                const std::string& frame = "",
                float red = 1.0f, float green = 0.0f,
                float blue = 0.0f, float alpha = 0.5f) const;

        float getGridLength() const
        {
            return grid_length_;
        };

        float getGridWidth() const
        {
            return grid_width_;
        };

        size_t getGridSizeX() const
        {
            return grid_size_x_;
        };

        size_t getGridSizeY() const
        {
            return grid_size_y_;
        }

        float getGridCellSize() const
        {
            return grid_cell_size_;
        };

        size_t getNumOfAngles() const
        {
            return num_of_angles_;
        };

        void setDefaultValue(T value)
        {
            default_value_ = value;
        };

        T getDefaultValue() const
        {
            return default_value_;
        };


    protected:
        std::vector<T> lattice_;

        float grid_length_;
        float grid_width_;
        size_t grid_size_x_;
        size_t grid_size_y_;
        float grid_cell_size_;
        float inv_grid_cell_size_;
        float grid_offset_;
        float half_grid_cell_size_;
        float angle_cell_size_;
        float inv_angle_cell_size_;
        float angle_offset_;
        size_t num_of_angles_;
        T default_value_;
        kelo::geometry_common::TransformMatrix2D tf_, tf_inv_;

        friend std::ostream& operator << (std::ostream& out, const Lattice& lattice)
        {
            LatticeIndex lat_ind;
            for ( size_t k = 0; k < lattice.num_of_angles_; k++ )
            {
                out << "k: " << k << std::endl;
                for ( size_t i = 0; i < lattice.grid_size_; i++ )
                {
                    for ( size_t j = 0; j < lattice.grid_size_; j++ )
                    {
                        lat_ind.i = i;
                        lat_ind.j = j;
                        lat_ind.k = k;
                        size_t index = lattice.calcIndex(lat_ind);
                        if ( lattice.lattice_[index] == lattice.default_value_ )
                        {
                            out << "- ";
                        }
                        else
                        {
                            out << lattice.lattice_[index] << " ";
                        }
                    }
                    out << std::endl;
                }
            }
            return out;
        };
};

} // namespace cabin

#endif // CABIN_LATTICE_H
