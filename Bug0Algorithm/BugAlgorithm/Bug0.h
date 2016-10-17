#pragma once
#include "BugAlgorithm.h"

class Bug0 : public BugAlgorithm
{
public:
	Bug0(string name);
	~Bug0();

	virtual void wallFollowing(bool, Box);

};

