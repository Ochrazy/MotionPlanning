#pragma once
#include "BugAlgorithm.h"

class Bug2 : public BugAlgorithm
{
public:
	Bug2(string name);
	~Bug2();

	virtual void wallFollowing(Point, Box);
	virtual void findHeadingAlongWall(Point, Box);
};

