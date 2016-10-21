#include "Bug0.h"


Bug0::Bug0(string name, bool bClockwise) : BugAlgorithm(name, bClockwise)
{
	heading = goalPosition - startPosition;
}


Bug0::~Bug0()
{
}

void Bug0::wallFollowing(Point robotPos, Box obstacle)
{
	// Turn 90 degrees to left/right 
	Point heading90 = Point(heading.y, -heading.x, 0);
	if (bWallFollowingClockwise == false)
		heading90 = Point(-heading.y, heading.x, 0);

	// Check if the obstacle/wall is still to right/left
	Point possibleRobotPos = robotPos + heading90 * dist_current;
	if (obstacleInWay(obstacle, possibleRobotPos) == false)
	{
		// Robo is located at a corner:
		// Bug0: check if robo can move to goal
		Point headingGoal = (goalPosition - robotPos).Normalize();
		Point possibleRobotPos2 = robotPos + headingGoal * dist_current * 1.5;
		if (obstacleInWay(obstacle, possibleRobotPos2) == false)
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
