#include <yaml_common/Parser2.h>
#include <cabin_nav/action/action.h>
#include <cabin_nav/utils/utils.h>

using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool Action::parseInputs(const YAML::Node& config)
{
    std::map<std::string, std::string> inputs_map;
    if ( !Parser::read<std::map<std::string, std::string>>(
                 config, "inputs", inputs_map) )
    {
        return false;
    }
    inputs_map_ = Utils::convertMapToUnorderedMap(inputs_map);
    return true;
}

} // namespace cabin
