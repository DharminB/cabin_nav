#ifndef CABIN_context_data_H
#define CABIN_context_data_H

#include <vector>
#include <string>
#include <iostream>
#include <regex>

#include <visualization_msgs/Marker.h>

#include <cabin_nav/action/action.h>
#include <cabin_nav/structs/trajectory_point.h>
#include <cabin_nav/input/input_data.h>

namespace cabin {

struct ContextData
{
    InputData::Ptr input_data;

    std::string current_behavior_name;

    std::vector<Action::Ptr> plan;
    size_t plan_index{0};

    bool is_paused{false};

    std::map<std::string, bool> active_inputs;

    bool isPlanValid() const;

    std::string getPlanAsString();

    std::vector<visualization_msgs::Marker> getPlanMarkers(
            const std::string& frame = "global") const;

    friend std::ostream& operator << (
            std::ostream& out,
            const ContextData& context_data);

};

} // namespace cabin

#endif // CABIN_context_data_H
