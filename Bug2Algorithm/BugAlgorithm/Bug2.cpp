#include "Bug2.h"


Bug2::Bug2(string name) : BugAlgorithm(name)
{
	heading = goalPosition - startPosition;
}


Bug2::~Bug2()
{
}


void Bug2::wallFollowing(bool, Box)
{
	heading = goalPosition - startPosition;
}
