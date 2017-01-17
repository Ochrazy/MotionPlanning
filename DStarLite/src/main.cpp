#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>

#include "dstarlite.h"

/**
 * Main.
 *
 * @param   int      number of arguments
 * @param   char**   arguments
 * @return  int
 */
int main(int argc, char **argv)
{
	//// Priority Algorithm
	//std::pair<unsigned int, unsigned int> current = std::make_pair(2, 1), goal = std::make_pair(2, 4);
	//std::pair<unsigned int, unsigned int>  current2 = std::make_pair(0, 2), goal2 = std::make_pair(4, 2);
	//DStarLite dstarlite(current, goal, 5, 5);
	//DStarLite dstarlite2(current2, goal2, 5, 5);

	//list<std::pair<unsigned int, unsigned int>> path;
	//path.push_back(current);
	//list<std::pair<unsigned int, unsigned int>> path2;
	//path2.push_back(current2);

	//while (current != goal || current2 != goal2)
	//{
	//	// Calc path1
	//	current = dstarlite.step(current, current2);
	//	if (current.first != -99) path.push_back(current);
	//	else current = goal;

	//	// Calc path1
	//	current2 = dstarlite2.step(current2, current);
	//	if (current2.first != -99) path2.push_back(current2);
	//	else current2 = goal2;
	//}

	std::vector<position4D> startGoalPositions;
	position4D sg1 = std::make_pair(std::make_pair(2, 2), std::make_pair(2, 2));
	startGoalPositions.push_back(sg1);
	position4D sg2 = std::make_pair(std::make_pair(0, 4), std::make_pair(3, 4));
	startGoalPositions.push_back(sg2);
	position4D sg3 = std::make_pair(std::make_pair(2, 0), std::make_pair(2, 4));
	startGoalPositions.push_back(sg3);
	
	DStarLite dstarlite(5, 5, 5, startGoalPositions, 20);
	std::vector<std::vector<position2D>> paths = dstarlite.calculatePaths();

	return 0;
}
