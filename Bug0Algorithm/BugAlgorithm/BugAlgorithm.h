#ifndef __BUGALGORITHM_H__
#define __BUGALGORITHM_H__

#include <iostream>
#include <vector>

#include "Point.h"
#include "Box.h"

using namespace std;

/*********************************************************************************************************************************/
static const double DIST_MIN = 0.01;            //minimum distance between the robot and the goal
static const double DIST_ERROR = DIST_MIN / 2.; //minimum allowed distance between the robot and the obstacle divided by 2
//static const double M_PI = 3.1415926;

class BugAlgorithm
{
protected:
	double totalDistance;
	Point goalPosition;     //the goal position
	Point startPosition;    //start point position
	Point heading;          // direction to move
	Point intermediate;     // intermediate Point
	Point actPoint;
	bool wallFollowingMode;
	bool bWallFollowingClockwise;
	Point cornerPoint;
	Point latestHitPoint;
	double dist_current;
	int obstacleHit;

public:
	BugAlgorithm::BugAlgorithm(const std::string& name);

	// ------ Virtuelle Methoden ----- //
	virtual void wallFollowing(Point, Box) {};
	virtual void findHeadingAlongWall(Point, Box) {};

	void setIntermediatePoint(Point pt);
	void setHeading(Point dir);
	void setHeading(double x, double y, double z = 0.f);
	void setGoalPosition(double x, double y);
	void setStartPosition(double x, double y);
	void setActPoint(Point p);
	Point getGoalPosition();
	Point getStartPosition();
	Point getHeading();
	Point getRobPos();

	// euklidischer Abstand
	double distanceEuclid(Point positionA, Point positionB);

	bool update(Box obstacle[], Box robot[], int nObst);
	int obstaclesInWay(Box obstacle[], Point robotPos, int nObst);
	bool obstacleInWay(Box obstacle, Point robotPos);
	bool goalReached(Point robotPos, Point goalPos, double distError);

	/****************************************************************************************************
	 * Return value: true, if line from point p1, p2 intersects line from point p3, p4
	 * if not, the function returns false
	 * in case of intersection, the intersection point intersection is calculated,
	 * also
	 * t1 the parameter value between [0,1]: intersection = p1 + t*(p2-p1)
	 * t2 the parameter value between [0,1]: intersection = p3 + t*(p4-p3)
	 */
	bool IntersectionLineLine(Point p1, Point p2, Point p3, Point p4, Point *intersection, double *t1, double *t2);
};

#endif /* __BUGALGORITHM_H__ */
