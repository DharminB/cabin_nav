#pragma once

#include <memory>
#include <iostream>
#include <string>
#include <unordered_map>

#include <cabin_nav/utils/print.h>

namespace cabin {

class OutputData
{

    public:

        using Ptr = std::shared_ptr<OutputData>;
        using ConstPtr = std::shared_ptr<const OutputData>;
        using Map = std::unordered_map<std::string, OutputData::Ptr>;
        using ConstMap = std::unordered_map<std::string, OutputData::ConstPtr>;

        OutputData(const std::string& type):
            type_(type) {}

        virtual std::ostream& write(std::ostream& out) const = 0;

        virtual void reset() = 0;

        const std::string& getType() const
        {
            return type_;
        }

    protected:

        static bool isValid(
                const OutputData::Map& output_data_map,
                const std::string& output_name,
                const std::string& output_type)
        {
            if ( output_data_map.find(output_name) == output_data_map.end() )
            {
                std::cout << Print::Err << Print::Time() << "[Output] "
                          << "OutputData::Map does not contain output data with name "
                          << output_name << "." << Print::End << std::endl;
                return false;
            }

            if ( output_data_map.at(output_name) == nullptr )
            {
                std::cout << Print::Err << Print::Time() << "[Output] "
                          << "OutputData::Map has nullptr for output data with name "
                          << output_name << "." << Print::End << std::endl;
                return false;
            }

            if ( output_data_map.at(output_name)->getType() != output_type )
            {
                std::cout << Print::Err << Print::Time() << "[Output] "
                          << "OutputData::Map contains output data with name "
                          << output_name << " but not with type " << output_type
                          << Print::End << std::endl;
                return false;
            }
            return true;
        }

    private:

        const std::string type_;

        OutputData() = delete;

};

inline std::ostream& operator << (std::ostream& out, const OutputData& output_data)
{
    return output_data.write(out);
};

} // namespace cabin
