#ifndef CABIN_behavior_FEEDBACK_H
#define CABIN_behavior_FEEDBACK_H

#include <vector>
#include <string>
#include <iostream>

#include <geometry_common/Circle.h>

#include <cabin_nav/structs/trajectory_point.h>
#include <cabin_nav/output/output_data.h>

namespace cabin {

struct BehaviorFeedback
{
    OutputData::Ptr output_data;

    bool success{true};
    std::string failure_code;

    friend std::ostream& operator << (std::ostream& out, const BehaviorFeedback& fb)
    {
        out << "output_data: " << std::endl;
        out << *(fb.output_data);
        if ( fb.success )
        {
            out << "success: True" << std::endl;
        }
        else
        {
            out << "success: False" << std::endl;
            out << "failure_code: " << fb.failure_code << std::endl;
        }
        return out;
    }
};

} // namespace cabin

#endif // CABIN_behavior_FEEDBACK_H
