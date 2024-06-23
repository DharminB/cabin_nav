#pragma once

#include <ros/ros.h>

#include <cabin_nav/output/output.h>
#include <cabin_nav/output/output_data.h>

namespace cabin {

class OutputManager
{
    public:

        OutputManager() = default;

        virtual ~OutputManager() = default;

        bool configure(const YAML::Node& config);

        void initializeOutputDataMap(OutputData::Map& output_data_map);

        bool setData(
                const OutputData::Map& output_data_map,
                const InputData::Map& input_data_map);

        void stopAllOutputs();

        void resetOutputDataMap(OutputData::Map& output_data_map) const;

    protected:

        std::unordered_map<std::string, Output::Ptr> outputs_;
        std::vector<std::string> output_names_; 

};

} // namespace cabin
