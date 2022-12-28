#ifndef CABIN_INPUT_H
#define CABIN_INPUT_H

#include <string>
#include <iostream>
#include <memory>

#include <yaml-cpp/yaml.h>

#include <cabin_nav/input/input_data.h>

namespace cabin {


class Input
{
    public:

        using Ptr = std::shared_ptr<Input>;

        virtual bool configure(const YAML::Node& config) = 0;

        virtual bool getData(InputData::Ptr& input_data,
                             const std::string& input_name) = 0;

        virtual void activate() = 0;

        virtual void deactivate() = 0;

        bool isActive() const
        {
            return is_active_;
        }

        bool alwaysActive() const
        {
            return always_active_;
        };

    protected:

        bool is_active_{false};
        bool always_active_{false};

        Input() = default;

};

} // namespace cabin

#endif // CABIN_INPUT_H
