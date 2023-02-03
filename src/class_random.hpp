#ifndef CLASS_RANDOM_HPP
#define CLASS_RANDOM_HPP

#include <iostream>
#include <time.h>

class Random
{
    public:
        Random();
        int dice();
        int getRandomNumber(int minimum, int maximum);
        double getRandomNumber(double minimum, double maximum, double space);
};

Random::Random()
{
    srand((unsigned int)time(NULL));
}

int Random::dice()
{
    return rand() % 6 +1;
}

int Random::getRandomNumber(int minimum, int maximum)
{
    return rand()%(maximum-minimum+1)+minimum;
}

double Random::getRandomNumber(double minimum, double maximum, double space)
{
    return getRandomNumber(minimum/space, maximum/space)*space;
}

#endif