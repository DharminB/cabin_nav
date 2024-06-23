#pragma once

#include <cabin_nav/input/input_data.h>
#include <cabin_nav/semantic_map/semantic_map.h>

namespace cabin {

class SemanticMapInputData : public InputData
{
    public:

        using Ptr = std::shared_ptr<SemanticMapInputData>;
        using ConstPtr = std::shared_ptr<const SemanticMapInputData>;

        SemanticMap::ConstPtr semantic_map{nullptr};

        SemanticMapInputData():
            InputData("semantic_map") {}

        virtual ~SemanticMapInputData() = default;

        std::ostream& write(std::ostream& out) const;

        static bool getSemanticMap(
                const InputData::Map& input_data_map,
                const std::string& input_name,
                SemanticMap::ConstPtr& _semantic_map);

};

} // namespace cabin
