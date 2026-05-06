#pragma once

#include <chrono>
#include <iostream>

struct ScopedTimer {
    const char* msg;
    bool enabled;
    std::chrono::high_resolution_clock::time_point start;

    ScopedTimer(const char* m, bool en)
        : msg(m), enabled(en), start(std::chrono::high_resolution_clock::now()) {}

    ~ScopedTimer() {
        if (!enabled) return;
        auto end = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(end - start).count();
        std::cout << msg << " took " << std::fixed << ms << " ms\n";
    }
};


