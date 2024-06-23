#pragma once

namespace cabin {

struct LatticeIndex
{
    size_t i, j, k;

    friend std::ostream& operator << (std::ostream& out, const LatticeIndex& lat_ind)
    {
        out << "<"
            << "i: " << lat_ind.i << ", "
            << "j: " << lat_ind.j << ", "
            << "k: " << lat_ind.k << ">";
        return out;
    };

    friend bool operator == (const LatticeIndex& l1, const LatticeIndex& l2)
    {
        return ( l1.i == l2.i && l1.j == l2.j && l1.k == l2.k );
    };

};

} // namespace cabin
