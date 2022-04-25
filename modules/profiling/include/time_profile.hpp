#ifndef TIME_PROFILE_HPP_
#define TIME_PROFILE_HPP_

#include <chrono>
#include <iostream>

namespace spark_profiler{

class Timer {
public:
    inline void start();
    inline void stop();
    /**
     * @brief get the duration count in milliseconds
     * 
     * @return the milli second for the duration count between start and stop
     */
    inline double duration();
private:
    std::chrono::steady_clock::time_point t_s_;
    std::chrono::steady_clock::time_point t_e_;
};

}

#endif