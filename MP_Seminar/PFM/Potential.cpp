#include <iostream>
#include "Potential.h"

#include <cmath>

using namespace std;

/*********************************************************************************************************************************/
static const double INKR = 0.005;        // step size for robot along gradient
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
	else if (cnt > 300)
	{
		cout << "That took way too long, abort!\n";
		return true;
	}

	double sf_att = 1.;
	double sf_rep = 1.;
	double dist_goal = robotPos.Distance(goalPosition);
	double d_star = 0.025;
	double q_star = 0.025;

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

bool Potential::update_cylinder(Cylinder obstacle[], Cylinder* robot, int nObst)
{
	/*Point robotPos = actPoint;
	static int cnt = 0;

	cnt++;

	if (goalReached(robotPos, goalPosition, GOAL_ERROR))
	{
		actPoint = goalPosition;
		//cout << "at goal, smile :)\n";
		cnt = 0;
		return true;
	}
	else if (cnt > 10000)
	{
		cout << "That took way too long, abort!\n";
		cnt = 0;
		return true;
	}


	double dist_goal = robotPos.Distance(goalPosition);
	double d_star = (*robot).GetRadius() - 0.03;
	double q_star = (*robot).GetRadius() - 0.03;
	double sf_att = 1.;
	//quadratic pot.
	double d2 = 0.;
	//double d2_delta_att = sf_att * dist_goal;
	Point d2_delta_att_grad;

	Point d2_delta_rep_grad(0.,0.,0.);
	double min_dist_obst = std::numeric_limits<double>::max();
	//repulsive pot.
	for (int i = 0; i < nObst; ++i)
	{
		sf_att = (obstacle[i].GetRepulsivness() + 30.);
		d2 = 0.5 * sf_att * pow(dist_goal, 2);
		
		d2_delta_att_grad = sf_att * (robotPos - goalPosition);
		double sf_rep = obstacle[i].GetRepulsivness();
		Point point_obst;
		double dist_obst = obstacle[i].distance((*robot), &point_obst);
		Point force = sf_rep * (1. / q_star - 1. / dist_obst) * 1. / pow(dist_obst, 2) * point_obst.Normalize();
		if (abs(dist_obst) <= q_star && dist_obst < min_dist_obst){

			Point  rob_origin = ((*robot).GetCenter() - obstacle[i].GetCenter());
			Point rob_rot = Point((cos(20.5) * rob_origin.x) + (-sin(20.5) * rob_origin.y),
				(sin(20.5) * rob_origin.x) + (cos(20.5) * rob_origin.y), 0.);
			Point rob_back = rob_rot + obstacle[i].GetCenter();
			d2_delta_rep_grad = force + rob_back;
			min_dist_obst = dist_obst;
		}
	}

	robotPos += d2_delta_rep_grad + d2_delta_att_grad;
	actPoint.Mac((goalPosition - robotPos).Normalize(), INKR); // move next step

	(*robot).SetCenter(actPoint);

	return false;*/




	Point robotPos = actPoint;
	static int cnt = 0;

	cnt++;

	if (goalReached(robotPos, goalPosition, GOAL_ERROR))
	{
		actPoint = goalPosition;
		//cout << "at goal, smile :)\n";

		return true;
	}
	else if (cnt > 10000)
	{
		cout << "That took way too long, abort!\n";
		return true;
	}

	double sf_att = 1.;

	double dist_goal = robotPos.Distance(goalPosition);
	double d_star = (*robot).GetRadius() - 0.025;
	double q_star = (*robot).GetRadius() - 0.025;

	//quadratic pot.
	double d2 = 0.5 * sf_att * pow(dist_goal, 2);
	//double d2_delta_att = sf_att * dist_goal;
	Point d2_delta_att_grad = sf_att * (robotPos - goalPosition);

	Point d2_delta_rep_grad(0., 0., 0.);
	double min_dist_obst = std::numeric_limits<double>::max();
	//repulsive pot.
	for (int i = 0; i < nObst; ++i)
	{
		double sf_rep = obstacle[i].GetRepulsivness();
		Point point_obst;
		double dist_obst = obstacle[i].distance((*robot), &point_obst);
		Point force = sf_rep * (1. / q_star - 1. / dist_obst) * 1. / pow(dist_obst, 2) * point_obst.Normalize();
		if (abs(dist_obst) <= q_star && dist_obst < min_dist_obst) {
			d2_delta_rep_grad = force;
			min_dist_obst = dist_obst;
		}
	}

	robotPos += d2_delta_rep_grad + d2_delta_att_grad;
	actPoint.Mac((goalPosition - robotPos).Normalize(), INKR); // move next step

	(*robot).SetCenter(actPoint);

	return false;
}

/*************************************************************************************************************************/
bool Potential::update_cylinder_navigation(Cylinder obstacle[], Cylinder* robot, int nObst, double k)
{
	Point robotPos = actPoint;
	static int cnt = 0;

	cnt++;

	if (goalReached(robotPos, goalPosition, GOAL_ERROR))
	{
		actPoint = goalPosition;
		//cout << "at goal, smile :)\n";
		cnt = 0;
		return true;
	}
	else if (cnt > 10000)
	{
		cout << "That took way too long, abort!\n";
		cnt = 0;
		return true;
	}

	// beta
	double beta_q = 1.;
	for (int x = 0; x < nObst; x++)
	{
		if (x == 0)
			beta_q *= -pow(robotPos.Distance(obstacle[x].GetCenter()), 2) + pow(obstacle[x].GetRadius() + (*robot).GetRadius(), 2);
		else
			beta_q *= pow(robotPos.Distance(obstacle[x].GetCenter()), 2) - pow(obstacle[x].GetRadius() + (*robot).GetRadius(), 2);
	}

	// Repulsive Gradient
	Point delta_beta_q(0., 0., 0.);
	for (int i = 0; i < nObst; i++)
	{
		// Potential
		double beta_j = 1.;
		for (int j = 0; j < nObst; j++)
		{
			if (j != i)
			{
				if (j == 0)
					beta_j *= -pow(robotPos.Distance(obstacle[j].GetCenter()), 2) + pow(obstacle[j].GetRadius() + (*robot).GetRadius(), 2);
				else 
					beta_j *= pow(robotPos.Distance(obstacle[j].GetCenter()), 2) - pow(obstacle[j].GetRadius() + (*robot).GetRadius(), 2);
			}
		}

		// gradient
		Point delta_beta_i(0., 0., 0.);
		if (i == 0)
			delta_beta_i = -2 * (robotPos - obstacle[i].GetCenter());
		else
			delta_beta_i = 2 * (robotPos - obstacle[i].GetCenter());

		// Total repulsive gradient
		delta_beta_q += delta_beta_i * beta_j;
	}

	// Attractive Gradient
	Point q_minus_qgoal = robotPos - goalPosition;
	double dist_q_qgoal = robotPos.Distance(goalPosition);
	double dist_q_qgoal_squared = pow(dist_q_qgoal, 2);
	double K = k;

	// A - B
	Point A = 2. * q_minus_qgoal * pow(pow(dist_q_qgoal, 2. * K) + beta_q, 1. / K);

	double B1 = dist_q_qgoal_squared * (1. / K);
	double B2 = pow(pow(dist_q_qgoal, 2. * K) + beta_q, 1. / K - 1.);
	Point B3 = (2. * K * pow(dist_q_qgoal, 2. * K - 2.)*(q_minus_qgoal)+delta_beta_q);
	Point B = B1 * B2 * B3;
	
	Point gamma = A	- B;
	
	gamma /= pow(pow(dist_q_qgoal, 2. * K) + beta_q, 2. / K);
	gamma.z = 0.;

	robotPos += gamma.Normalize();

	actPoint.Mac((goalPosition - robotPos).Normalize(), INKR); // move next step

	(*robot).SetCenter(actPoint);

	return false;
}