#include <iostream>
#include "Potential.h"

#include <cmath>

using namespace std;

/*********************************************************************************************************************************/
static const double INKR = 0.01;        // step size for robot along gradient
static const double DIST_MIN = 0.05;    // minimum distance between the robot and the goal
static const double GOAL_ERROR = 0.01;  // distance between the robot and the goal

/*********************************************************************************************************************************/
Potential::Potential(const std::string& name)
    : goalPosition(1.0, 1.0, 0.0) //set the goal position
    , startPosition(0.0, 0.0, 0.0) //set the start position
{
}

void Potential::setGoalPosition(double x, double y, double z)
{
    goalPosition.x = x;
    goalPosition.y = y;
    goalPosition.z = z;
}

void Potential::setStartPosition(double x, double y, double z)
{
    startPosition.x = x;
    startPosition.y = y;
    startPosition.z = z;
}

void Potential::setActPoint(Point p)
{
    actPoint.x = p.x;
    actPoint.y = p.y;
    actPoint.z = p.z;
}

Point Potential::getGoalPosition()
{
    return goalPosition;
}

Point Potential::getStartPosition()
{
    return startPosition;
}

Point Potential::getRobPos()
{
    return actPoint;
}

bool Potential::goalReached(Point robotPos, Point goalPos, double distError)
{
    return (robotPos.Distance(goalPos) <= distError);
}

/*************************************************************************************************************************/
bool Potential::update_box(Box obstacle[], Box robot[], int nObst)
{
    Point robotPos = actPoint;
    static int cnt = 0;

    cnt++;

    if (goalReached(robotPos, goalPosition, GOAL_ERROR))
    {
        actPoint = goalPosition;
        //cout << "at goal, smile :)\n";

        return true;
	}
	else if (cnt == 1000)
	{
		return true;
	}

	double sf_att = 1.;
	double sf_rep = 1.;
	double dist_goal = robotPos.Distance(goalPosition);
	double d_star = 0.05;
	double q_star = 0.05;

	//quadratic pot.
	double d2 = 0.5 * sf_att * pow(dist_goal, 2);
	double d2_delta_att = sf_att * dist_goal;
	Point d2_delta_att_grad = sf_att * ( robotPos - goalPosition);

	double d2_delta_rep_grad = 0;

	//repulsive pot.
	Point point(0,0,0);
	double min_dist_obst = 1.;

	for (int i = 0; i < nObst; ++i)
	{
		Point point_obst;
		double dist_obst = obstacle[i].distance(robot[0], &point_obst);
		min_dist_obst = dist_obst < min_dist_obst ? dist_obst : min_dist_obst;
	}

	for (int i = 0; i < nObst; ++i)
	{
		Point point_obst;
		double dist_obst = obstacle[i].distance(robot[0], &point_obst);
		//double force = sf_rep * pow(1. / dist_obst - 1. / q_star, 2.) * 1. / pow(dist_obst, 2);
		Point force = sf_rep * (1./q_star - 1. / dist_obst ) * 1. / q_star * point_obst.Normalize();
		//if (dist_obst < q_star)
			point += force.Normalize();
	}
	Point goalPos = (goalPosition - robotPos);
	robotPos = ((goalPos * d2_delta_att_grad) + point).Normalize();
	actPoint.Mac((goalPosition - robotPos).Normalize(), INKR); // move next step

    return false;

	if (dist_goal > d_star)
	{
		//conical pot.
		double d1 = sf_att * dist_goal;
	}

}

/*************************************************************************************************************************/
bool Potential::update_cylinder(Cylinder obstacle[], Cylinder robot[], int nObst)
{
    Point robotPos = actPoint;
    static int cnt = 0;

    cnt++;

    if (goalReached(robotPos, goalPosition, GOAL_ERROR))
    {
        actPoint = goalPosition;
        //cout << "at goal, smile :)\n";

        return true;
    }

    actPoint.Mac((goalPosition - robotPos).Normalize(), INKR); // move next step

    return false;
}

/*************************************************************************************************************************/
bool Potential::update_cylinder_navigation(Cylinder obstacle[], Cylinder robot[], int nObst)
{
    Point robotPos = actPoint;
    static int cnt = 0;

    cnt++;

    if (goalReached(robotPos, goalPosition, GOAL_ERROR))
    {
        actPoint = goalPosition;
        //cout << "at goal, smile :)\n";

        return true;
    }


	
    actPoint.Mac((goalPosition - robotPos).Normalize(), INKR); // move next step

    return false;
}
