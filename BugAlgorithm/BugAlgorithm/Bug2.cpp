#include "Bug2.h"


Bug2::Bug2(string name, bool bClockwise) : BugAlgorithm(name, bClockwise)
{
	heading = (goalPosition - startPosition).Normalize();
}


Bug2::~Bug2()
{
}

void Bug2::wallFollowing(Point robotPos, Box obstacle)
{
	// Turn 90 degrees to left/right 
	Point heading90 = Point(heading.y, -heading.x, 0);
	if (bWallFollowingClockwise == false)
		heading90 = Point(-heading.y, heading.x, 0);

	// Check if the obstacle/wall is still to right/left
	Point possibleRobotPos = robotPos + heading90 * dist_current;
	if (obstacleInWay(obstacle, possibleRobotPos) == false)
	{
		// Free to move on = corner
		heading = heading90;
		cornerPoint = robotPos;
	}

	if (isnan(cornerPoint.x) == false)
	{
		// Robo hit m-line? 
		Point inter;
		double t1, t2;
		Point possibleRobotPos3 = robotPos + heading * dist_current;
		float p1x = robotPos.x, p1y = robotPos.y, p2x = possibleRobotPos3.x, p2y = possibleRobotPos3.y,
			p3x = startPosition.x, p3y = startPosition.y, p4x = goalPosition.x, p4y = goalPosition.y;
		// Intersection between Robo-line(possiblePos - lastCornerPoint) and m-line
		if (IntersectionLineLine(cornerPoint, Point(p2x, p2y, 0), Point(p3x, p3y, 0), Point(p4x, p4y, 0),
			&inter, &t1, &t2))
		{
			// Move to intersection
			dist_current = robotPos.Distance(inter);
			wallFollowingMode = false;
			std::cout << "Leave Point: " << robotPos.x << "," << robotPos.y << std::endl;
			cornerPoint.x = nanf(" ");
		}
	}
}
