#pragma once

#include <cabin_nav/structs/lattice_index.h>

namespace cabin {

struct NeighbourOffset
{
    int i, j, k;
    float g;

    NeighbourOffset(int _i=0, int _j=0, int _k=0, float _g=0.0f):
        i(_i), j(_j), k(_k), g(_g) {};

    friend std::ostream& operator << (std::ostream& out, const NeighbourOffset& n_offset)
    {
        out << "<"
            << "i: " << n_offset.i << ", "
            << "j: " << n_offset.j << ", "
            << "k: " << n_offset.k << ", "
            << "g: " << n_offset.g << ">";
        return out;
    };
};

typedef std::vector<std::vector<NeighbourOffset> > AllNeighbourOffsets;

} // namespace cabin
