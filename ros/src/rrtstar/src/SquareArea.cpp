#include "SquareArea.h"
#include "CircularObstacle.h"

#include <random>

SquareArea::SquareArea(double side):
    side_(side) {}

double SquareArea::getSide() const
{
    return side_;
}

ObstacleList SquareArea::getObstacles() const
{
    return obstacles_;
}

SquareAreaPtr SquareArea::create(double squareSide,
        double obstacleRadius, int obstacleCount)
{
    std::shared_ptr<SquareArea> area{new SquareArea(squareSide)};

    // Make sure the distribution satisfies that random circular
    // obstacles are always fully within the square area.
    std::uniform_real_distribution<double> dist(2*obstacleRadius,
                                            squareSide - 2*obstacleRadius);
    std::default_random_engine engine;

    // Populate random circular obstacles within the square area.
    for (int i = 0; i < obstacleCount; i++)
    {
        double ox = dist(engine);
        double oy = dist(engine);

        auto obstacle = std::make_shared<CircularObstacle>(ox, oy, obstacleRadius);
        area->obstacles_.push_back(obstacle);
    }

    return area;
}
