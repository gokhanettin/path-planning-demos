#include <random>
#include <limits>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include "SquareArea.h"
#include "CircularObstacle.h"
#include "rrtstar/MakePlan.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Collion checker. If the point robot runs into any of the circular
// obstacles within the square area, the state is said to be invalid.
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si,
            const ObstacleList& obstacles={}):
        ob::StateValidityChecker(si), obstacles_(obstacles) {}

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const override
    {
        return this->clearance(state) > 0.0;
    }

    // Returns the nearest distance from the given state's position to the
    // boundary of the circular obstacles.
    double clearance(const ob::State* state) const override
    {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const auto* state2D =
            state->as<ob::RealVectorStateSpace::StateType>();

        // Extract the robot's (x,y) position from its state
        double x = state2D->values[0];
        double y = state2D->values[1];

        double minDistance = std::numeric_limits<double>::infinity();
        for (auto obstacle : obstacles_)
        {
            double ox = obstacle->getX();
            double oy = obstacle->getY();
            double radius = obstacle->getRadius();
            // Distance formula between two points, offset by the circle's
            // radius
            double distance = sqrt((ox-x)*(ox-x) + (oy-y)*(oy-y)) - radius;
            if (distance < minDistance)
            {
                minDistance = distance;
            }
        }
        return minDistance;
    }
    private:
        ObstacleList obstacles_;
};

class RRTStarPlanner
{
    public:
        RRTStarPlanner(ros::NodeHandle *nh)
        {
            markerPublisher_ = nh->advertise<visualization_msgs::MarkerArray>(
                        "visualization_marker_array", 1);
            pathPublisher_ = nh->advertise<nav_msgs::Path>("path", 1);
            planService_ = nh->advertiseService("make_plan",
                                                &RRTStarPlanner::makePlan, this);

            double squareAreaSide;
            double obstacleRadius;
            int    obstacleCount;

            // Get params from parameter server
            nh->param<double>("/max_run_time_seconds", maxRunTimeSeconds_, 1.0);
            nh->param<double>("/square_area_side", squareAreaSide, 100.0);
            nh->param<double>("/obstacle_radius", obstacleRadius, 5.0);
            nh->param<int>("/obstacle_count", obstacleCount, 30);

            // Create a square area with some random cirular obstacles
            area_ = SquareArea::create(squareAreaSide,
                    obstacleRadius, obstacleCount);
        }

        void publishSolutionPath(ob::PathPtr path)
        {
            og::PathGeometric pg = *static_cast<og::PathGeometric*>(path.get());
            auto states = pg.getStates();
            std::vector<geometry_msgs::PoseStamped> poses(states.size());
            nav_msgs::Path msg;
            msg.header.frame_id = "map";
            msg.header.stamp = ros::Time::now();

            for (int i = 0; i < states.size(); i++)
            {
                const auto* state =
                        states.at(i)->as<ob::RealVectorStateSpace::StateType>();
                poses.at(i).pose.position.x = state->values[0];
                poses.at(i).pose.position.y = state->values[1];
            }

            msg.poses = poses;
            pathPublisher_.publish(msg);
        }

        void publishConfiguration(double startX, double startY,
                double goalX, double goalY)
        {
            visualization_msgs::MarkerArray markerArray;
            visualization_msgs::Marker      marker;
            int id = 0;

            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            // obstacles
            for (auto obstacle : area_->getObstacles())
            {
                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time::now();
                marker.id = id;
                marker.scale.x = obstacle->getRadius() * 2;
                marker.scale.y = obstacle->getRadius() * 2;
                marker.scale.z = 0.5;
                marker.pose.position.x = obstacle->getX();
                marker.pose.position.y = obstacle->getY();
                marker.pose.position.z = 0.0;
                markerArray.markers.push_back(marker);
                id++;
            }

            // start position
            marker.type = visualization_msgs::Marker::CUBE;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.id = id;
            marker.scale.x = 2.5;
            marker.scale.y = 2.5;
            marker.scale.z = 2.5;
            marker.pose.position.x = startX;
            marker.pose.position.y = startY;
            marker.pose.position.z = 0.0;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            id++;
            markerArray.markers.push_back(marker);

            // goal position
            marker.type = visualization_msgs::Marker::CUBE;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.id = id;
            marker.scale.x = 2.5;
            marker.scale.y = 2.5;
            marker.scale.z = 2.5;
            marker.pose.position.x = goalX;
            marker.pose.position.y = goalY;
            marker.pose.position.z = 0.0;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            id++;
            markerArray.markers.push_back(marker);

            markerPublisher_.publish(markerArray);
        }
        bool makePlan(rrtstar::MakePlan::Request &req,
                    rrtstar::MakePlan::Response & res)
        {
            publishConfiguration(req.startX, req.startY, req.goalX, req.goalY);

            // Construct the robot state space in which we're planning. We're
            // planning in [0, SQUARE_AREA_SIDE]x[0, SQUARE_AREA_SIDE], a subset of R^2.
            auto space(std::make_shared<ob::RealVectorStateSpace>(2));

            // Set the bounds of space to be in [0, SQUARE_AREA_SIDE].
            space->setBounds(0.0, area_->getSide());

            // Construct a space information instance for this state space
            auto si(std::make_shared<ob::SpaceInformation>(space));

            // Set the object used to check which states in the space are valid
            auto validityChecker = std::make_shared<ValidityChecker>(si,
                    area_->getObstacles());
            si->setStateValidityChecker(validityChecker);

            si->setup();

            // Set our robot's starting state
            ob::ScopedState<> start(space);
            start->as<ob::RealVectorStateSpace::StateType>()->values[0] = req.startX;
            start->as<ob::RealVectorStateSpace::StateType>()->values[1] = req.startY;

            // Set our robot's goal state
            ob::ScopedState<> goal(space);
            goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = req.goalX;
            goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = req.goalY;

            // Create a problem instance
            auto pdef(std::make_shared<ob::ProblemDefinition>(si));

            // Set the start and goal states
            pdef->setStartAndGoalStates(start, goal);

            // Create the optimization objective


            auto objective(std::make_shared<ob::PathLengthOptimizationObjective>(si));
            //objective->setCostThreshold(ob::Cost(area_->getSide() * 10));
            pdef->setOptimizationObjective(objective);

            // Construct the optimal planner
            ob::PlannerPtr optimizingPlanner(std::make_shared<og::RRTstar>(si));

            // Set the problem instance for our planner to solve
            optimizingPlanner->setProblemDefinition(pdef);
            optimizingPlanner->setup();

            // Attempt to solve the planning problem in the given runtime
            double startTime = ros::Time::now().toSec();
            ob::PlannerStatus solved = optimizingPlanner->solve(maxRunTimeSeconds_);
            double endTime = ros::Time::now().toSec();

            if (solved)
            {
                res.elapsedTimeMiliseconds = (endTime - startTime) * 1000.;
                res.found = true;
                publishSolutionPath(pdef->getSolutionPath());
            }
            else
            {
                res.elapsedTimeMiliseconds = 0.0;
                res.found = false;
            }

            ROS_INFO("request: [start(%lf, %lf), goal(%lf, %lf)]",
                    req.startX, req.startY, req.goalX, req.goalY);
            ROS_INFO("sending back response: [path found: %s within %lf miliseconds]",
                    res.found ? "true" : "false", res.elapsedTimeMiliseconds);

            return true;
        }

    private:
        double             maxRunTimeSeconds_;
        SquareAreaPtr       area_;
        ros::Publisher     markerPublisher_;
        ros::Publisher     pathPublisher_;
        ros::ServiceServer planService_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrtstar");
  ros::NodeHandle nh;
  RRTStarPlanner planner(&nh);

  ROS_INFO("Ready for making a plan.");
  ros::spin();

  return 0;
}
