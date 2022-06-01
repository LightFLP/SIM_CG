#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <iostream>

#include "util/recorder.h"

Recorder::Recorder(double *_dt, double _timing, int _iterations)
            : dt(_dt), max_timing(_timing), max_iterations(_iterations) {}

Recorder::Recorder(double *_dt) : dt(_dt), max_timing((*_dt) * 100), max_iterations(1) {}

void Recorder::start() {
    // Get current date and time
    struct tm  tstruct;
    time_t now = time(0);
    tstruct = *localtime(&now);

    static char filename[80];
    strftime(filename, sizeof(filename), "recorder_%Y-%m-%d_%H-%M-%S.txt", &tstruct);
    out_file.open(filename);
}

void Recorder::write(std::string value) {
    out_file << value << "\n";
}

bool Recorder::reset() {
    timing += (*dt);
    if (timing >= max_timing) {
        timing = 0.0;
        iteration++;
        return true;
    }
    return false;
}

bool Recorder::stop() {
    if (iteration >= max_iterations) {
        iteration = 0;
        out_file << "\n";
        return true;
    }
    return false;
}

void Recorder::close() {
    out_file.close();
}