#include <thread>

#include <yaml_common/Parser2.h>
#include <geometry_common/Utils.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/input/localisation_input.h>
#include <cabin_nav/input/localisation_input_data.h>

using kelo::geometry_common::TransformMatrix2D;
using GCUtils = kelo::geometry_common::Utils;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool LocalisationInput::configure(const YAML::Node& config)
{
    robot_frame_ = Parser::get<std::string>(config, "robot_frame", "base_link");
    global_frame_ = Parser::get<std::string>(config, "global_frame", "map");
    always_active_ = Parser::get<bool>(config, "always_active", false);
    tf_listener_ = std::make_shared<tf::TransformListener>();
    std::this_thread::sleep_for(std::chrono::duration<float>(0.5f)); // for tf_listener_ to accumulate tf msgs
    return true;
}

bool LocalisationInput::getData(InputData::Ptr& input_data)
{
    if ( input_data == nullptr ) // for first iteration
    {
        input_data = std::make_shared<LocalisationInputData>();
    }

    if ( input_data->getType() != getType() )
    {
        std::cout << Print::Err << Print::Time() << "[LocalisationInput] "
                  << "input_data's type is not \"" << getType() << "\"."
                  << Print::End << std::endl;
        return false;
    }

    LocalisationInputData::Ptr localisation_input_data =
        std::static_pointer_cast<LocalisationInputData>(input_data);

    tf::StampedTransform stamped_transform;
    try
    {
        tf_listener_->lookupTransform(global_frame_, robot_frame_,
                                      ros::Time(0), stamped_transform);
        localisation_input_data->localisation_tf.update(stamped_transform);
    }
    catch ( const tf::TransformException& ex )
    {
        std::cout << Print::Err << Print::Time() << "[LocalisationInput] "
                  << "Could not lookup transform from " << global_frame_
                  << " to " << robot_frame_ << ". Following exception occured"
                  << std::endl << ex.what()
                  << Print::End << std::endl;
        return false;
    }
    return true;
}

void LocalisationInput::activate()
{
    if ( is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[LocalisationInput] "
                  << "Already active. Ignoring activate() call"
                  << Print::End << std::endl;
        return;
    }

    is_active_ = true;
}

void LocalisationInput::deactivate()
{
    if ( !is_active_ )
    {
        std::cout << Print::Warn << Print::Time() << "[LocalisationInput] "
                  << "Already inactive. Ignoring deactivate() call"
                  << Print::End << std::endl;
        return;
    }

    if ( always_active_ )
    {
        return;
    }

    is_active_ = false;
}

std::ostream& LocalisationInput::write(std::ostream& out) const
{
    out << "<Input type: " << getType() << ">";
    return out;
}

} // namespace cabin
