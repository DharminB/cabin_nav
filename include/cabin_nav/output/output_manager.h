#ifndef CABIN_OUTPUT_MANAGER_H
#define CABIN_OUTPUT_MANAGER_H

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

        bool setData(OutputData::Ptr& output_data, const InputData::Ptr& input_data);

        void stopAllOutputs();

    protected:

        std::map<std::string, Output::Ptr> outputs_;

};

} // namespace cabin

#endif // CABIN_OUTPUT_MANAGER_H
