#ifndef SQUARE_AREA_
#define SQUARE_AREA_

#include <memory>
#include <vector>

class CircularObstacle;
typedef std::shared_ptr<CircularObstacle> CircularObstaclePtr;

class SquareArea;
typedef std::shared_ptr<SquareArea> SquareAreaPtr;

typedef std::vector<CircularObstaclePtr> ObstacleList;

class SquareArea
{
    public:
        // Singletons should not be cloneable.
        SquareArea(SquareArea &other) = delete;
        // Singletons should not be assignable.
        void operator=(const SquareArea &) = delete;

        double getSide() const;
        ObstacleList getObstacles() const;
        static SquareAreaPtr create(double squareSide,
                double obstacleRadius, int obstacleCount);
    protected:
        // Disable public access to constructor.
        SquareArea(double side);
    private:
        double side_;
        ObstacleList obstacles_;
};

#endif

