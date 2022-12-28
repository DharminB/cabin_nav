#ifndef CABIN_OUTPUT_H
#define CABIN_OUTPUT_H

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>

#include <cabin_nav/output/output_data.h>
#include <cabin_nav/input/input_data.h>

namespace cabin {

class Output
{
    public:

        using Ptr = std::shared_ptr<Output>;

        virtual ~Output() = default;

        virtual bool configure(const YAML::Node& config) = 0;

        virtual bool setData(
                const OutputData::Ptr& output_data,
                const InputData::Ptr& input_data,
                const std::string& output_name) = 0;

        virtual void start() = 0;

        virtual void stop() = 0;

    protected:

        float rate_{1.0f};
        std::chrono::duration<float> ideal_loop_duration_{1.0f};

        std::thread loop_thread_;
        std::mutex loop_thread_mutex_;

        std::atomic<bool> is_active_{false};

        void mainLoop()
        {
            std::chrono::steady_clock::time_point start_time;
            std::chrono::duration<float> time_taken;

            while ( is_active_ )
            {
                start_time = std::chrono::steady_clock::now();

                loop_thread_mutex_.lock();
                step();
                loop_thread_mutex_.unlock();

                /* sleep for remaining time */
                time_taken = std::chrono::steady_clock::now() - start_time;
                std::this_thread::sleep_for(ideal_loop_duration_ - time_taken);
            }
        }

        virtual void step() = 0;

        Output() = default;

};

} // namespace cabin

#endif // CABIN_OUTPUT_H
