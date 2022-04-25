#include "time_profile.hpp"

using timer = spark_profiler::Timer;

inline void timer::start() {
    t_s_ = std::chrono::steady_clock::now();
}

inline void timer::stop() {
    t_e_ = std::chrono::steady_clock::now();
}

/**
 * @brief get the duration count in milliseconds
 * 
 * @return the milli second for the duration count between start and stop
 */
inline double timer::duration() {
    return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_e_ - t_s_).count();
}