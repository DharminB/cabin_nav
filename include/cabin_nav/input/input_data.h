#ifndef CABIN_INPUT_DATA_H
#define CABIN_INPUT_DATA_H

#include <memory>
#include <iostream>
#include <string>
#include <unordered_map>

#include <cabin_nav/utils/print.h>

namespace cabin {

class InputData
{

    public:

        using Ptr = std::shared_ptr<InputData>;
        using ConstPtr = std::shared_ptr<const InputData>;
        using Map = std::unordered_map<std::string, InputData::Ptr>;
        using ConstMap = std::unordered_map<std::string, InputData::ConstPtr>;

        InputData(const std::string& type):
            type_(type) {}

        virtual std::ostream& write(std::ostream& out) const = 0;

        const std::string& getType() const
        {
            return type_;
        }

    protected:

        static bool isValid(
                const InputData::Map& input_data_map,
                const std::string& input_name,
                const std::string& input_type)
        {
            if ( input_data_map.find(input_name) == input_data_map.end() )
            {
                std::cout << Print::Err << Print::Time() << "[Input] "
                          << "InputData::Map does not contain input data with name "
                          << input_name << "." << Print::End << std::endl;
                return false;
            }

            if ( input_data_map.at(input_name) == nullptr )
            {
                std::cout << Print::Err << Print::Time() << "[Input] "
                          << "InputData::Map has nullptr for input data with name "
                          << input_name << "." << Print::End << std::endl;
                return false;
            }

            if ( input_data_map.at(input_name)->getType() != input_type )
            {
                std::cout << Print::Err << Print::Time() << "[Input] "
                          << "InputData::Map contains input data with name "
                          << input_name << " but not with type " << input_type
                          << Print::End << std::endl;
                return false;
            }
            return true;
        }

    private:

        const std::string type_;

        InputData() = delete;

};

inline std::ostream& operator << (std::ostream& out, const InputData& input_data)
{
    return input_data.write(out);
};

} // namespace cabin

#endif // CABIN_INPUT_DATA_H
