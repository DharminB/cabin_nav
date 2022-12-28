#ifndef CABIN_PRINT_H
#define CABIN_PRINT_H

#include <string>
#include <chrono>
#include <ctime>

namespace cabin {

struct Print
{
    static constexpr const char* Err     = "\033[31m";    // start red
    static constexpr const char* Warn    = "\033[33m";    // start yellow
    static constexpr const char* Success = "\033[32m";    // start green
    static constexpr const char* End     = "\033[0m";     // back to normal

    /**
     * @brief Return time as a std::string
     *
     * @return return time as a string in "[YYYY-MM-DD_hh:mm::ss.xxx]" format
     */
    static std::string Time()
    {
        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
        long now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(
                now).time_since_epoch().count();
        long now_s = std::chrono::time_point_cast<std::chrono::seconds>(
                now).time_since_epoch().count();
        int ms = now_ms - (now_s * 1000);
        std::time_t now_t = std::chrono::system_clock::to_time_t(now);
        char time_str[25];
        size_t n = std::strftime(time_str, 24, "[%F_%T", std::localtime(&now_t));
        sprintf(time_str+n, ".%03d]", ms);
        return std::string(time_str);
    }

};

} // namespace cabin
#endif // CABIN_PRINT_H
