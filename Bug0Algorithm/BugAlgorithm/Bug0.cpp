#include "Bug0.h"


Bug0::Bug0(string name) : BugAlgorithm(name)
{
	heading = goalPosition - startPosition;
}


Bug0::~Bug0()
{
}


void Bug0::wallFollowing(bool, Box)
{
	heading = goalPosition - startPosition;
}
