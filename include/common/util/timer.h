#pragma once

#include <chrono>
#include <string>

class Timer {
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> endTime;
    long long durationTime = 0;

public:
    Timer start();
    Timer stop();
    std::string duration() const;
};


