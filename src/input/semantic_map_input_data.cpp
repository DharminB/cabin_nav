#include <cabin_nav/input/semantic_map_input_data.h>

namespace cabin {

std::ostream& SemanticMapInputData::write(std::ostream& out) const
{
    out << "<Input type: " << getType() << ", semantic_map: " << semantic_map << ">";
    return out;
}

bool SemanticMapInputData::getSemanticMap(
        const InputData::Map& input_data_map,
        const std::string& input_name,
        SemanticMap::ConstPtr& _semantic_map)
{
    if ( !InputData::isValid(input_data_map, input_name, "semantic_map") )
    {
        return false;
    }

    SemanticMapInputData::ConstPtr semantic_map_input_data =
        std::static_pointer_cast<SemanticMapInputData>(
                input_data_map.at(input_name));

    _semantic_map = semantic_map_input_data->semantic_map;

    return true;
}

} // namespace cabin
