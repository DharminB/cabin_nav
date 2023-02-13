#ifndef CABIN_SEMANTIC_MAP_INPUT_H
#define CABIN_SEMANTIC_MAP_INPUT_H

#include <mutex>

#include <cabin_nav/semantic_map/semantic_map.h>
#include <cabin_nav/input/input_data.h>
#include <cabin_nav/input/input.h>

namespace cabin {

class SemanticMapInput : public Input
{
    public:

        SemanticMapInput():
            Input("semantic_map") {}

        virtual ~SemanticMapInput() = default;

        bool configure(const YAML::Node& config);

        bool getData(InputData::Ptr& input_data);

        void activate();

        void deactivate();

        std::ostream& write(std::ostream& out) const;

    protected:

        SemanticMap::Ptr semantic_map_{nullptr};

};

} // namespace cabin

#endif // CABIN_SEMANTIC_MAP_INPUT_H
