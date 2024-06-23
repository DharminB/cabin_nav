#pragma once

#include <cabin_nav/structs/lattice_index.h>

namespace cabin {

struct Node
{
    float g, h, f;
    LatticeIndex lat_ind;

    Node(float _g=0, float _h=0, float _f=0):
        g(_g), h(_h), f(_f) {};

    Node(const Node& parent, int i_offset, int j_offset, int k_offset, size_t num_of_angles):
        h(0.0f), f(0.0f)
    {
        g = parent.g;
        lat_ind.i = parent.lat_ind.i + i_offset;
        lat_ind.j = parent.lat_ind.j + j_offset;
        lat_ind.k = (parent.lat_ind.k + k_offset + num_of_angles) % num_of_angles;
    };

    friend std::ostream& operator << (std::ostream& out, const Node& n)
    {
        out << "<"
            << n.lat_ind << " "
            << "g: " << n.g << ", "
            << "h: " << n.h << ", "
            << "f: " << n.f << ">";
        return out;
    };
    
};

} // namespace cabin
