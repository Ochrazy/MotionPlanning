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
	else if (cnt == 100)
	{
		return true;
	}

	double sf_att = 1.;
	double sf_rep = 1.;
	double dist_goal = robotPos.Distance(goalPosition);
	double d_star = 0.05;
	double q_star = 0.04;

	//quadratic pot.
	double d2 = 0.5 * sf_att * pow(dist_goal, 2);
	//double d2_delta_att = sf_att * dist_goal;
	Point d2_delta_att_grad = sf_att * (robotPos - goalPosition);

	Point d2_delta_rep_grad(0., 0., 0.);

	//repulsive pot.
	for (int i = 0; i < nObst; ++i)
	{
		Point point_obst;
		double dist_obst = obstacle[i].distance(robot[0], &point_obst);
		Point force = sf_rep * (1. / q_star - 1. / dist_obst) * 1. / pow(dist_obst, 2) * point_obst.Normalize();
		if (abs(dist_obst) <= q_star)
			d2_delta_rep_grad += force;
	}

	robotPos += d2_delta_rep_grad + d2_delta_att_grad;
	actPoint.Mac((goalPosition - robotPos).Normalize(), INKR); // move next step

	robot[0].Translate((goalPosition - robotPos).Normalize() * INKR);

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
	else if (cnt == 100)
	{
		return true;
	}

	double sf_att = 1.;
	double sf_rep = 1.;
	double dist_goal = robotPos.Distance(goalPosition);
	double d_star = 0.05;
	double q_star = 0.04;

	//quadratic pot.
	double d2 = 0.5 * sf_att * pow(dist_goal, 2);
	//double d2_delta_att = sf_att * dist_goal;
	Point d2_delta_att_grad = sf_att * (robotPos - goalPosition);

	Point d2_delta_rep_grad(0.,0.,0.);

	//repulsive pot.
	for (int i = 0; i < nObst; ++i)
	{
		Point point_obst;
		double dist_obst = obstacle[i].distance(robot[0], &point_obst);
		Point force = sf_rep * (1. / q_star - 1. / dist_obst) * 1. / pow(dist_obst, 2) * point_obst.Normalize();
		if (abs(dist_obst) <= q_star)
			d2_delta_rep_grad += force;
	}

	robotPos += d2_delta_rep_grad + d2_delta_att_grad;
	actPoint.Mac((goalPosition - robotPos).Normalize(), INKR); // move next step

	robot[0].SetCenter(actPoint);

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
	else if (cnt > 200)
		return true;

	// beta
	double beta_q = 1.;
	for (int x = 0; x < nObst; x++)
	{
		if (x == 0)
			beta_q *= -pow(robotPos.Distance(obstacle[x].GetCenter()), 2) + pow(obstacle[x].GetRadius(), 2);
		else
			beta_q *= pow(robotPos.Distance(obstacle[x].GetCenter()), 2) - pow(obstacle[x].GetRadius(), 2);
	}

	// Repulsive Gradient
	Point delta_beta_q(0., 0., 1.);
	for (int i = 0; i < nObst; i++)
	{
		// gradient
		Point repulsive_gradient(0., 0., 1.);
		if (i == 0)
			repulsive_gradient = -2 * (robotPos - obstacle[i].GetCenter());
		else 
			repulsive_gradient = 2 * (robotPos - obstacle[i].GetCenter());

		// Potential
		double repulsive_potential = 1.;
		for (int j = 0; j < nObst; j++)
		{
			if (j != i)
			{
				if (j == 0)
					repulsive_potential *= -pow((robotPos.Distance(obstacle[j].GetCenter())), 2) + pow(obstacle[j].GetRadius(), 2);
				else 
					repulsive_potential *= pow(robotPos.Distance(obstacle[j].GetCenter()), 2) - pow(obstacle[j].GetRadius(), 2);
			}
		}

		// Total repulsive gradient
		delta_beta_q += repulsive_gradient * repulsive_potential * 100.;
	}

	// Attractive Gradient
	Point q_minus_qgoal = robotPos - goalPosition;
	double dist_q_qgoal = robotPos.Distance(goalPosition);
	double dist_q_qgoal_squared = pow(dist_q_qgoal, 2);
	double K = 2.;

	Point gamma = 2 * q_minus_qgoal * pow(pow(dist_q_qgoal, 2 * K) + beta_q, 1. / K)
		- dist_q_qgoal_squared*1. / K*pow(pow(dist_q_qgoal, 2 * K) + beta_q, 1. / (K - 1.)) * (2 * K*pow(dist_q_qgoal, 2 * K - 2)*(q_minus_qgoal)+delta_beta_q);
	
	gamma /= pow(pow(dist_q_qgoal, 2 * K) + beta_q, 2. / K);
	gamma.z = 0.;
	robotPos += gamma;
	//robotPos += gamma + delta_beta_q;
	actPoint.Mac((goalPosition - robotPos).Normalize(), INKR); // move next step

	robot[0].SetCenter(actPoint);

	return false;
}