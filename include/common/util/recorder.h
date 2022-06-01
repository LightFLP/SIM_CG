#pragma once

class Recorder {
    double *dt;
    double timing, max_timing;
    int iteration, max_iterations;

    std::ofstream out_file;

public:
    Recorder(double *_dt);
    Recorder(double *_dt, double _timing, int _iterations);

    void start();
    void write(std::string value);
    bool reset();
    bool stop();
    void close();
};
