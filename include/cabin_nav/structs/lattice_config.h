#ifndef CABIN_LATTICE_CONFIG_H
#define CABIN_LATTICE_CONFIG_H

#include <geometry_common/TransformMatrix2D.h>

namespace cabin {

struct LatticeConfig
{
    size_t grid_size_x;
    size_t grid_size_y;
    float grid_cell_size;
    size_t num_of_angles;
    kelo::geometry_common::TransformMatrix2D tf;

    LatticeConfig() = default;

    LatticeConfig(float _grid_radius, float _grid_cell_size, size_t _num_of_angles)
    {
        size_t grid_size = (2 * static_cast<size_t>(_grid_radius/_grid_cell_size)) + 1;
        grid_size_x = grid_size;
        grid_size_y = grid_size;
        grid_cell_size = _grid_cell_size;
        num_of_angles = _num_of_angles;
        tf = kelo::geometry_common::TransformMatrix2D(
                -(grid_cell_size * grid_size)/2,
                -(grid_cell_size * grid_size)/2,
                0.0f);
    }
};

} // namespace cabin

#endif // CABIN_LATTICE_CONFIG_H
