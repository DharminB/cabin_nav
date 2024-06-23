#pragma once

#include <geometry_common/TransformMatrix2D.h>

#include <cabin_nav/input/input_data.h>

namespace cabin {

class LocalisationInputData : public InputData
{
    public:

        using Ptr = std::shared_ptr<LocalisationInputData>;
        using ConstPtr = std::shared_ptr<const LocalisationInputData>;

        kelo::geometry_common::TransformMatrix2D localisation_tf;

        LocalisationInputData():
            InputData("localisation") {}

        virtual ~LocalisationInputData() = default;

        std::ostream& write(std::ostream& out) const;

        static bool getLocalisationTF(
                const InputData::Map& input_data_map,
                const std::string& input_name,
                kelo::geometry_common::TransformMatrix2D& _loc_tf);

};

} // namespace cabin
