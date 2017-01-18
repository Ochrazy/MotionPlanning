#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <fstream>

#include "dstarlite.h"

void write_easyrob_program_file(std::vector<std::vector<position2D>> path, float scale, std::string filename, bool jump_to)
{
	std::ofstream myfile;

	myfile.open(filename);
	myfile << "ProgramFile" << std::endl;

	for (int i = 0; i < (int)path.size(); i++)
	{
		if (jump_to)
			myfile << "JUMP_TO_AX ";
		else
			myfile << "PTP_AX ";

		myfile << path[i][0].first/ scale << " " << path[i][0].second/ scale;
		for (int r = 1; r < (int)path[i].size(); r++)
			myfile << " " << path[i][r].first/ scale << " " << path[i][r].second/ scale;

		myfile << std::endl;
	}
	myfile << "EndProgramFile" << std::endl;
	myfile.close();
}

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
	
	std::vector<std::vector<position2D>> paths;
	std::vector<position4D> startGoalPositions;
	startGoalPositions.push_back(std::make_pair(std::make_pair(0, 25), std::make_pair(50, 25)));
	//startGoalPositions.push_back(std::make_pair(std::make_pair(0, 0), std::make_pair(99, 99)));
	startGoalPositions.push_back(std::make_pair(std::make_pair(26, 50), std::make_pair(26, 0)));
	
	DStarLite dstarlite(5, 100, 100, startGoalPositions, 30);
	paths = dstarlite.calculatePaths();
	write_easyrob_program_file(paths, 100, "rrt.prg", false);
	return 0;
}
