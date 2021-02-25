#include "CircularObstacle.h"

CircularObstacle::CircularObstacle(double x, double y, double radius):
    x_(x), y_(y), radius_(radius) {}

double CircularObstacle::getX() const
{
    return x_;
}

double CircularObstacle::getY() const
{
    return y_;
}

double CircularObstacle::getRadius() const
{
    return radius_;
}
