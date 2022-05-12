#include "timer.h"

Timer Timer::start() {
    startTime = std::chrono::high_resolution_clock::now();
    return *this;
}

Timer Timer::stop() {
    endTime = std::chrono::high_resolution_clock::now();
    durationTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
    return *this;
}

std::string Timer::duration() const {
    long long seconds = durationTime / 1000;
    long long milliseconds = durationTime - (durationTime / 1000) * 1000;
    return {std::to_string(seconds) + "." + std::to_string(milliseconds)};
}
