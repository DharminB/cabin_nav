#include <cabin_nav/input/localisation_input_data.h>

namespace cabin {

std::ostream& LocalisationInputData::write(std::ostream& out) const
{
    out << "<Input type: " << getType() << ", localisation_tf: " << localisation_tf << ">";
    return out;
}

bool LocalisationInputData::getLocalisationTF(
        const InputData::Map& input_data_map,
        const std::string& input_name,
        kelo::geometry_common::TransformMatrix2D& _loc_tf)
{
    if ( !InputData::isValid(input_data_map, input_name, "localisation") )
    {
        return false;
    }

    LocalisationInputData::ConstPtr localisation_input_data =
        std::static_pointer_cast<LocalisationInputData>(
                input_data_map.at(input_name));

    _loc_tf = localisation_input_data->localisation_tf;

    return true;
}

} // namespace cabin
