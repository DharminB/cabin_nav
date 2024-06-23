#pragma once

namespace cabin {

enum class Status
{
    UNKNOWN,
    RUNNING,
    SUCCESS,
    FAILURE
};

const std::vector<std::string> status_strings = {
    "UNKNOWN",
    "RUNNING",
    "SUCCESS",
    "FAILURE"
};

inline std::ostream& operator << (std::ostream& out, const Status& status)
{
    size_t status_uint = static_cast<size_t>(status);
    status_uint = ( status_uint > status_strings.size() ) ? 0 : status_uint;
    out << status_strings[status_uint];
    return out;
};

} // namespace cabin
