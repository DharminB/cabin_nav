#include <cabin_nav/utils/print.h>
#include <cabin_nav/structs/context_data.h>

namespace cabin {

bool ContextData::isPlanValid() const
{
    return ( !plan.empty() && plan_index < plan.size() );
}

std::string ContextData::getPlanAsString()
{
    std::string plan_string;
    plan_string += "plan:\n";
    for ( size_t i = plan_index; i < plan.size(); i++ )
    {
        std::string action_plan_string = "  - ";
        action_plan_string += plan[i]->getPlanAsString();
        action_plan_string = std::regex_replace(action_plan_string,
                                                std::regex("\n"), "\n    ");
        plan_string += action_plan_string + "\n";
    }
    return plan_string;
}

std::ostream& operator << (std::ostream& out, const ContextData& context_data)
{
    std::ostringstream input_data_stream;
    input_data_stream << "input_data:" << std::endl << *(context_data.input_data);
    std::string input_data_string = input_data_stream.str();
    input_data_string = std::regex_replace(input_data_string, std::regex("\n"), "\n    ");
    out << input_data_string << std::endl;

    if ( context_data.is_paused )
    {
        out << "Paused." << std::endl;
    }
    out << "plan size: " << context_data.plan.size() << std::endl;
    out << "plan index: " << context_data.plan_index << std::endl;
    if ( context_data.plan.size() - context_data.plan_index > 0 )
    {
        out << "plan:" << std::endl;
        for ( size_t i = context_data.plan_index; i < context_data.plan.size(); i++ )
        {
            std::ostringstream action_stream;
            action_stream << "  - " << *context_data.plan[i];
            std::string action_string = action_stream.str();
            action_string = std::regex_replace(action_string, std::regex("\n"), "\n    ");
            out << action_string << std::endl;
        }
    }
    return out;
}

std::vector<visualization_msgs::Marker> ContextData::getPlanMarkers(
        const std::string& frame) const
{
    std::vector<visualization_msgs::Marker> plan_markers;
    if ( !isPlanValid() )
    {
        return plan_markers;
    }

    for ( size_t i = plan_index; i < plan.size(); i++ )
    {
        std::vector<visualization_msgs::Marker> action_markers = plan[i]->asMarkers();
        plan_markers.reserve(plan_markers.size() + action_markers.size());
        plan_markers.insert(plan_markers.end(), action_markers.begin(),
                action_markers.end());
    }

    return plan_markers;
}

} // namespace cabin
