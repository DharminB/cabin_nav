#ifndef CABIN_BEHAVIOR_FEEDBACK_H
#define CABIN_BEHAVIOR_FEEDBACK_H

#include <vector>
#include <string>
#include <iostream>

#include <cabin_nav/output/output_data.h>

namespace cabin {

struct BehaviorFeedback
{
    OutputData::Map output_data_map;

    bool success{true};
    std::string failure_code;

    friend std::ostream& operator << (std::ostream& out, const BehaviorFeedback& fb)
    {
        out << "output_data: " << std::endl;
        for ( auto itr = fb.output_data_map.begin(); itr != fb.output_data_map.end(); itr ++ )
        {
            out << "    " << itr->first << ": " << itr->second << std::endl;
        }

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

#endif // CABIN_BEHAVIOR_FEEDBACK_H
