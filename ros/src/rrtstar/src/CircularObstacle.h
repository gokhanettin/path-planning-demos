#ifndef CIRCULAR_OBSTACLE_
#define CIRCULAR_OBSTACLE_

class CircularObstacle
{
    public:
        CircularObstacle(double x, double y, double radius);

        double getX() const;
        double getY() const;
        double getRadius() const;

    private:
        double x_;
        double y_;
        double radius_;
};

#endif
