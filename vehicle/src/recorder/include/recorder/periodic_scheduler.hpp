#pragma once

#include <chrono>

class PeriodicScheduler {
public:
    explicit PeriodicScheduler( std::chrono::microseconds rate_in );

    void reset();
    void schedule_next();
    void wait();

    uint64_t missed_cycles();

private:
    const std::chrono::microseconds         _rate;
    std::chrono::steady_clock::time_point   _next;
    uint64_t                                _missed;
};