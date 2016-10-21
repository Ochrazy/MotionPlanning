#pragma once
#include "BugAlgorithm.h"

class Bug2 : public BugAlgorithm
{
public:
	Bug2(string name, bool bClockwise);
	~Bug2();

	virtual void wallFollowing(Point, Box);
};

