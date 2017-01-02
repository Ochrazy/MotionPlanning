#pragma once
#include <vector>

struct Point2D
{
	double x, y;
};

class Program
{
public:
	Program();
	~Program();

	std::vector<std::vector<Point2D>> program;
};

