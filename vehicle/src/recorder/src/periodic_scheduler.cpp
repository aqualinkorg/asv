#include <recorder/periodic_scheduler.hpp>

#include <thread>

PeriodicScheduler::PeriodicScheduler( std::chrono::microseconds rate_in )
    : _rate{ rate_in }
    , _next{ std::chrono::steady_clock::now() }
    , _missed{ 0 }
{}

// Resets the waiting period
void PeriodicScheduler::reset()
{
    _next = std::chrono::steady_clock::now();
}

void PeriodicScheduler::schedule_next()
{
    auto elapsed = std::chrono::steady_clock::now() - _next;
    if( elapsed > _rate )
    {
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>( elapsed ).count();

        // Calculate number of whole cycles missed
        auto missed = elapsed_us / _rate.count();

        // Update total missed cycles
        _missed += static_cast<uint32_t>( missed );

        // Align to next multiple of rate period. Always add at least one period.
        _next += std::chrono::microseconds( ( missed + 1 ) * _rate.count() );
    }
    else
    {
        // If we woke up, then we must have slept until the next scheduled time
        // Potential for early wakeup here, but ignore for now and proceed to next schedule
        _next += _rate;
    }
}

void PeriodicScheduler::wait()
{
    std::this_thread::sleep_until( _next );
}

uint64_t PeriodicScheduler::missed_cycles()
{
    return _missed;
}