#include "BugAlgorithm.h"

using namespace std;

BugAlgorithm::BugAlgorithm(const string& name)
	: goalPosition(1.0, 1.0, 0.0) //set the goal position
	, startPosition(0.0, 0.0, 0.0) //set the start position
	, wallFollowingMode(false)
	, bWallFollowingClockwise(false)
	, cornerPoint(Point(nanf(" "), 0, 0))
	, dist_current(DIST_MIN)
	, obstacleHit(-1)
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

/*************************************************************************************************************************/
bool BugAlgorithm::update(Box obstacle[], Box robot[], int nObst)
{
	Point robotPos = actPoint;
	static Box mink_diff[2] = { obstacle[0].MinkowskiDifference(robot[0]),
								obstacle[1].MinkowskiDifference(robot[0]) };
	dist_current = DIST_MIN;

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
		obstacleHit = -1;
		Point possibleRobotPos = robotPos + heading * dist_current;
		if ((obstacleHit = obstaclesInWay(mink_diff, possibleRobotPos, nObst)) == -1)
		{
			// Free to move to goal
		}
		else // robo would hit a wall
		{
			std::cout << "Hit Point: " << robotPos.x << "," << robotPos.y << std::endl;
			latestHitPoint = robotPos;

			// Follow the wall
			wallFollowingMode = true;

			// Find Heading along wall
			findHeadingAlongWall(robotPos, mink_diff[obstacleHit]);
		}
	}
	else // Second Phase: Wall Following Mode
	{
		// Check if robo is back at hitpoint: No Path can be found :(
		if (distanceEuclid(robotPos, latestHitPoint) < DIST_MIN / 2)
		{
			std::cout << "No Path found!" << std::endl;
			return true;
		}

		// Find the heading and distance for next move
		wallFollowing(robotPos, mink_diff[obstacleHit]);
	}

	// Move with current heading and dist_current
	actPoint.Mac(heading, dist_current); // one step forward
	return false;
}

/*****************************************************************************/
int BugAlgorithm::obstaclesInWay(Box obstacle[], Point robotPos, int nObst)
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

bool BugAlgorithm::obstacleInWay(Box obstacle, Point robotPos)
{
	return obstacle.isPointInsideAABB(robotPos);
}

/**********************************************************************************************/
bool BugAlgorithm::goalReached(Point robotPos, Point goalPos, double distError)
{
	return (distanceEuclid(robotPos, goalPos) <= distError);
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
