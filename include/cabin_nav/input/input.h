#pragma once

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
        using ConstPtr = std::shared_ptr<const Input>;

        Input(const std::string& type):
            type_(type) {}

        virtual bool configure(const YAML::Node& config) = 0;

        virtual bool getData(InputData::Ptr& input_data) = 0;

        virtual void activate() = 0;

        virtual void deactivate() = 0;

        virtual std::ostream& write(std::ostream& out) const = 0;

        const std::string& getType() const
        {
            return type_;
        }

        bool isActive() const
        {
            return is_active_;
        }

        bool alwaysActive() const
        {
            return always_active_;
        }

    protected:

        bool is_active_{false};
        bool always_active_{false};

    private:

        const std::string type_;

        Input() = delete;

};

inline std::ostream& operator << (std::ostream& out, const Input& input)
{
    return input.write(out);
};

} // namespace cabin
