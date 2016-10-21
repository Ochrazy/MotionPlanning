#include "BugAlgorithm.h"

using namespace std;

BugAlgorithm::BugAlgorithm(const string& name)
	: firstMove(true)
	, totalDistance(0.0)
	, goalPosition(1.0, 1.0, 0.0) //set the goal position
	, startPosition(0.0, 0.0, 0.0) //set the start position
	, firstHit(false)
	, wallFollowingMode(false)
	, bWallFollowingClockwise(false)
	, cornerPoint(Point(-99, 0, 0))
{
}

void BugAlgorithm::setIntermediatePoint(Point pt)
{
	intermediate = pt;
}

void BugAlgorithm::setHeading(Point dir)
{
	heading = dir;
}

void BugAlgorithm::setHeading(double x, double y, double z)
{
	heading.x = x;
	heading.y = y;
	heading.z = z;
}

void BugAlgorithm::setGoalPosition(double x, double y)
{
	goalPosition.x = x;
	goalPosition.y = y;
	goalPosition.z = 0.0;
}

void BugAlgorithm::setStartPosition(double x, double y)
{
	startPosition.x = x;
	startPosition.y = y;
	startPosition.z = 0.0;
}

void BugAlgorithm::setActPoint(Point p)
{
	actPoint.x = p.x;
	actPoint.y = p.y;
	actPoint.z = 0.0;
}

Point BugAlgorithm::getGoalPosition()
{
	return goalPosition;
}

Point BugAlgorithm::getStartPosition()
{
	return startPosition;
}

Point BugAlgorithm::getHeading()
{
	return heading;
}

Point BugAlgorithm::getRobPos()
{
	return actPoint;
}

// euklidischer Abstand
double BugAlgorithm::distanceEuclid(Point positionA, Point positionB)
{
	double x_diff = positionA.x - positionB.x;
	double y_diff = positionA.y - positionB.y;
	double z_diff = positionA.z - positionB.z;
	return sqrt((x_diff * x_diff) + (y_diff * y_diff));// + (z_diff * z_diff));
}

/**********************************************************************************************/
void BugAlgorithm::setTotalDistance(double value)
{
	totalDistance = value;
}

/**********************************************************************************************/
double BugAlgorithm::getTotalDistance()
{
	return totalDistance;
}

/*************************************************************************************************************************/
bool BugAlgorithm::update(Box obstacle[], Box robot[], int nObst)
{
	Point pt, robotPos = actPoint;
	static Box mink_diff[2] = { obstacle[0].MinkowskiDifference(robot[0]),
								obstacle[1].MinkowskiDifference(robot[0]) };
	static int ret = -1;  // no obstacle
	double dist_current = DIST_MIN;

	if (goalReached(robotPos, goalPosition, dist_current)) {
		actPoint = goalPosition;
		// cout << "at goal, smile :) \n";
		return true;
	}
	// First Phase: Move to Goal
	if (wallFollowingMode == false)
	{
		// Head to goal
		heading = (goalPosition - robotPos).Normalize();

		// Check if robo could move to goal without collision
		Point possibleRobotPos = robotPos + heading * dist_current;
		int obstacleHit = -1;
		if ((obstacleHit = obstacleInWay(mink_diff, robot, possibleRobotPos, nObst)) == -1)
		{
			// Free to move to goal
		}
		else // robo would hit a wall
		{
			std::cout << "Hit Point: " << robotPos.x << "," << robotPos.y << std::endl;
			latestHitPoint = robotPos;

			// Follow the wall
			wallFollowingMode = true;

			// Move along wall clockwise or counterclockwise?
			double direction = 1;
			if (bWallFollowingClockwise == false)
				direction = -1;

			// Find Heading along Wall
			Box boxHit = mink_diff[obstacleHit];
			// Check on what side of the rectangle/obstacle robo is and set heading accordingly
			// Left side
			if (robotPos.x < boxHit.GetVertex(0).x && robotPos.x < boxHit.GetVertex(3).x)
			{
				// Bottom-Left
				if (robotPos.y < boxHit.GetVertex(0).y && robotPos.y < boxHit.GetVertex(1).y)
				{
					if (bWallFollowingClockwise) heading = Point(0, direction, 0);
					else heading = Point(-direction, 0, 0);
				}
				// Top-Left
				else if (robotPos.y > boxHit.GetVertex(3).y && robotPos.y > boxHit.GetVertex(2).y)
				{
					if (bWallFollowingClockwise) heading = Point(direction, 0, 0);
					else heading = Point(0, direction, 0);
				}
				else // Only Left side
					heading = Point(0, direction, 0);
			}
			// Right side
			else if (robotPos.x > boxHit.GetVertex(1).x && robotPos.x > boxHit.GetVertex(2).x)
			{
				// Bottom-Right
				if (robotPos.y < boxHit.GetVertex(0).y && robotPos.y < boxHit.GetVertex(1).y)
				{
					if (bWallFollowingClockwise) heading = Point(-direction, 0, 0);
					else heading = Point(0, -direction, 0);
				}
				// Top-Right
				else if (robotPos.y > boxHit.GetVertex(3).y && robotPos.y > boxHit.GetVertex(2).y)
				{
					if (bWallFollowingClockwise) heading = Point(0, -direction, 0);
					else heading = Point(direction, 0, 0);
				}
				else // Only Right side
					heading = Point(0, -direction, 0);
			}
			// Only Top side
			else if (robotPos.y > boxHit.GetVertex(3).y && robotPos.y > boxHit.GetVertex(2).y)
				heading = Point(direction, 0, 0);
			// Only Bottom side
			else if (robotPos.y < boxHit.GetVertex(0).y && robotPos.y < boxHit.GetVertex(1).y)
				heading = Point(-direction, 0, 0);
		}
	}
	else // Second Phase: Wall Following Mode
	{
		// Check if robo is back at hitpoint: No Solution can be found :(
		if (distanceEuclid(robotPos, latestHitPoint) < 0.005)
		{
			std::cout << "No Path found!" << std::endl;
			return true;
		}

		// Turn 90 degrees to left/right 
		Point heading90 = Point(heading.y, -heading.x, 0);
		if (bWallFollowingClockwise == false)
			heading90 = Point(-heading.y, heading.x, 0);

		// Check if the obstacle/wall is still to right/left
		Point possibleRobotPos = robotPos + heading90 * dist_current;
		if (obstacleInWay(mink_diff, robot, possibleRobotPos, nObst) == -1)
		{
			// Robo is located at a corner:
			// Bug0: check if robo can move to goal
			Point headingGoal = (goalPosition - robotPos).Normalize();
			Point possibleRobotPos2 = robotPos + headingGoal * dist_current * 1.5;
			if (obstacleInWay(mink_diff, robot, possibleRobotPos2, nObst) == -1)
			{
				// Found leave point: move to goal again (first phase)
				heading = headingGoal;
				wallFollowingMode = false;
				std::cout << "Leave Point: " << robotPos.x << "," << robotPos.y << std::endl;
			}
			else
			{
				// Robo can not move to goal:
				// Robo is located at a corner of obstacle turn 90 degree right/left to further follow wall 
				heading = heading90;
			}
		}
	}

	actPoint.Mac(heading, dist_current); // one step forward
	return false;
}

/*****************************************************************************/
int BugAlgorithm::obstacleInWay(Box obstacle[], Box robot[], Point robotPos, int nObst)
{
	for (int i = 0; i < nObst; i++)
	{
		if (obstacle[i].isPointInsideAABB(robotPos))
		{
			return i;
		}
	}

	return(-1);
}

/******************************************************************************/
void BugAlgorithm::move()
{
	actPoint.Mac(heading, DIST_MIN);
}

/******************************************************************************/
void BugAlgorithm::wallFollowingToPoint(Point p, bool dir, Box box)
{
	double distanceMargin = 0.005;

	while (distanceEuclid(actPoint, p) > distanceMargin)
	{
		wallFollowing(dir, box);
	}

	cout << "at leave point \n";
	move();
}

/******************************************************************************/
void BugAlgorithm::motionToGoal()
{
	actPoint.Mac(heading, DIST_MIN);
	//cout << "motion to goal\n";
}

/**********************************************************************************************/
void BugAlgorithm::halt()
{
}

//----- Obstacle Avoidance ----//

/**********************************************************************************************/
bool BugAlgorithm::completeCycleAroundObstacle(Point robotPos, Point hitPoint)
{
	//.....
	return true;
}

/**********************************************************************************************/
void BugAlgorithm::identifyLeavePoint(bool direction, Point robotPos, Point goalPos, Box box)
{
	recordHitPoint(robotPos);
	//wallFollowing(direction, box);
	//double tot = totalDistance + 1;
	//setTotalDistance(tot);
	findLeavePoint(robotPos, getLastHitPoint());
}

/**********************************************************************************************/
bool BugAlgorithm::goalReached(Point robotPos, Point goalPos, double distError)
{
	return (distanceEuclid(robotPos, goalPos) <= distError);
}

/**********************************************************************************************/
//get the first hit point
Point BugAlgorithm::getFirstHitPoint()
{
	vector<Point>::iterator it = hitPoint.begin();
	return Point(*it);
}

/**********************************************************************************************/
//get the last hit point
Point BugAlgorithm::getLastHitPoint()
{
	vector<Point>::iterator it = hitPoint.end() - 1;
	return Point(*it);
}

/**********************************************************************************************/
//get the first leave point
Point BugAlgorithm::getFirstLeavePoint()
{
	vector<Point>::iterator it = leavePoint.begin();
	return ::Point(*it);
}

/**********************************************************************************************/
//get the last leave point
Point BugAlgorithm::getLastLeavePoint()
{
	vector<Point>::iterator it = leavePoint.end() - 1;
	return Point(*it);
}

/**********************************************************************************************/
void BugAlgorithm::recordHitPoint(Point point)
{
	if (!hitDefined)
	{
		// Add the hit point
		hitPoint.push_back(point);
		// now the robot can switch to the boundary following mode
		hitDefined = true;
		//isAlreadyLeftLastHitPoint = false;
	}
}

/**********************************************************************************************/
void BugAlgorithm::recordLeavePoint(Point point)
{
	leavePoint.push_back(point);
}

/**********************************************************************************************/
bool BugAlgorithm::obstacleInFrontOfTheRobot()
{
	return(true);
}

/**********************************************************************************************/
bool BugAlgorithm::goalIsReachable(Point robotPos, Point goalPos, double distError)
{
	bool res = false;
	//if (obstacleInFrontOfTheRobot() && goalReached(robotPos, goalPos, distError))
	if (goalReached(robotPos, goalPos, distError))
		res = true;
	//else if (!obstacleInFrontOfTheRobot())
	//  res = true;
	//cout << "Goal reachable from this point?? " << res << "\n";
	return res;
}

/****************************************************************************************************
 * Return value: true, if line from point p1, p2 intersects line from point p3, p4
 * if not, the function returns false
 * in case of intersection, the intersection point intersection is calculated,
 * also
 * t1 the parameter value between [0,1]: intersection = p1 + t*(p2-p1)
 * t2 the parameter value between [0,1]: intersection = p3 + t*(p4-p3)
 */
bool BugAlgorithm::IntersectionLineLine(Point p1, Point p2, Point p3, Point p4, Point *intersection, double *t1, double *t2)
{
	// Store the values for fast access and easy
	// equations-to-code conversion
	// intersection = NULL;
	double x1 = p1.x, x2 = p2.x, x3 = p3.x, x4 = p4.x;
	double y1 = p1.y, y2 = p2.y, y3 = p3.y, y4 = p4.y;

	double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
	// If d is zero, there is no intersection
	if (d == 0)
		return false;

	// Get the x and y
	double pre = (x1*y2 - y1*x2), post = (x3*y4 - y3*x4);
	double x = (pre * (x3 - x4) - (x1 - x2) * post) / d;
	double y = (pre * (y3 - y4) - (y1 - y2) * post) / d;

	// Check if the x and y coordinates are within both lines
	if (x < min(x1, x2) || x > max(x1, x2) || x < min(x3, x4) || x > max(x3, x4))
		return false;
	if (y < min(y1, y2) || y > max(y1, y2) || y < min(y3, y4) || y > max(y3, y4))
		return false;

	// Return the point of intersection
	intersection->x = x;
	intersection->y = y;
	intersection->z = 0.0;

	double par_t = intersection->SquareDistance(p1) / p2.SquareDistance(p1);
	*t1 = sqrt(par_t);
	par_t = intersection->SquareDistance(p3) / p4.SquareDistance(p3);
	*t2 = sqrt(par_t);
	return true;
}

/**********************************************************************************************
 * Find the points of intersection of a circle and a line segment. There max. 2 intersection points
 * returns number of intersection points
 */
int BugAlgorithm::FindLineCircleIntersections(Point center, double radius, Point point1, Point point2, Point *intersection1, Point *intersection2)
{
	int ret = 0; // number of intersection points in the interval [0,1]
	double dx, dy, A, B, C, det, t;
	double cx = center.x;
	double cy = center.y;
	dx = point2.x - point1.x;
	dy = point2.y - point1.y;

	A = dx * dx + dy * dy;
	B = 2 * (dx * (point1.x - cx) + dy * (point1.y - cy));
	C = (point1.x - cx) * (point1.x - cx) + (point1.y - cy) * (point1.y - cy) - radius * radius;

	det = B * B - 4 * A * C;
	if ((A <= 0.0000001) || (det < 0))
	{
		// No real solutions.
		return ret;
	}
	else if (det == 0)
	{
		// possibly one solution.
		t = -B / (2 * A);
		if (t >= 0.0 && t <= 1.0)
		{
			intersection1 = new Point(point1.x + t * dx, point1.y + t * dy, 0.0);
			ret++;
		}
		return ret;
	}
	else
	{
		// possibly two solutions.
		t = (double)((-B + sqrt(det)) / (2. * A));
		if (t >= 0.0 && t <= 1.0)
		{
			intersection1 = new Point(point1.x + t * dx, point1.y + t * dy, 0.0);
			ret++;
		}
		t = (double)((-B - sqrt(det)) / (2. * A));
		if (t >= 0.0 && t <= 1.0)
		{
			intersection2 = new Point(point1.x + t * dx, point1.y + t * dy, 0.0);
			ret++;
		}
		if (ret == 2)
		{
			ret--;
			intersection1->Lerp(*intersection1, *intersection2, 0.5);
		}

		return ret;
	}
}
