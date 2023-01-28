#ifndef CLASS_WAIT_HPP
#define CLASS_WAIT_HPP

#include <ros/ros.h>
#include <iostream>

class Wait
{
    private:
        ros::Time _start, _end;
        bool _is_first = true;
        ros::Duration _duration;
        double _duration_seconds;

    public:
        bool isWaiting(double seconds_);
        void reset();
};

bool Wait::isWaiting(double seconds_)
{
    if(_is_first)
    {
        _start = ros::Time::now();
        _is_first = false;
    }

    _end = ros::Time::now();

    _duration = _end-_start;

    _duration_seconds = _duration.toSec();

    // std::cout << _duration_seconds << std::endl;

    if(_duration_seconds < seconds_)
    {
        return true;
    }

    _is_first = true;
    return false;
}

void Wait::reset()
{
    _is_first = true;
}

#endif