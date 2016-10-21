#include "Bug0.h"


Bug0::Bug0(string name, bool bClockwise) : BugAlgorithm(name, bClockwise)
{
	heading = goalPosition - startPosition;
}


Bug0::~Bug0()
{
}

void Bug0::findHeadingAlongWall(Point robotPos, Box obstacle)
{
	// Move along wall clockwise or counterclockwise?
	double direction = 1;
	if (bWallFollowingClockwise == false)
		direction = -1;

	// Find Heading along Wall
	Box boxHit = obstacle;
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
