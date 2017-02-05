/**
 * DStarLite.
 *
 * @package		DStarLite
 * @author		Aaron Zampaglione <azampagl@gmail.com>
 * @copyright	Copyright (C) 2011 Aaron Zampaglione
 * @license		MIT
 */
#include "DStarLite.h"
#include <iostream>

/**
* Main execution method.
*
* @return  int  successfull
*/
list<Map::Cell*> DStarLite::replan()
{
	// Replan the path
	if (!_planner[0]->replan())
	{
		std::cout << "No Solution Found!";
		throw;
	}

	return _planner[0]->path();
}

list<Map::Cell*> DStarLite::replan(unsigned int plannerIndex)
{
	// Replan the path
	if (!_planner[plannerIndex]->replan())
	{
		std::cout << "No Solution Found! " << plannerIndex;
		return _planner[plannerIndex]->path();
	}

	return _planner[plannerIndex]->path();
}

/**
 * Constructor.
 * 
 * @param  unsigned char*    name of the DStarLite'
 * @param  Config            config options
*/
DStarLite::DStarLite(position2D start, position2D goal, unsigned int width, unsigned int height, double inGoalCostObjects)
	: goalCostObjects(inGoalCostObjects)
{
	// Prepare real and robot image buffers
	const int img_width = width, img_height = height;

	// Make the map
	_map = new Map(img_height, img_width);

	// Build map
	for (int i = 0; i < img_height; i++)
	{
		for (int j = 0; j < img_width; j++)
		{
			double v = 3.0; // always walkable

			// Cell is unwalkable
			if (v == 2.0)
			{
				v = Map::Cell::COST_UNWALKABLE;
			}
			else
			{
				v = 1.0;
			}

			(*_map)(i, j)->cost = v;
		}
	}

	// Make planner
	_planner.push_back(new Planner(_map, (*_map)(start.second, start.first), (*_map)(goal.second, goal.first)));

	// Save Robot Positions
	currentPositions.push_back(start);
}

DStarLite::DStarLite(Map* map, std::vector<position4D> startGoalPositions, double inGoalCostObjects)
	: goalCostObjects(inGoalCostObjects)
{
	_map = map;

	// Make planner
	for each (position4D startGoal in startGoalPositions)
	{
		_planner.push_back(new Planner(_map, (*_map)(startGoal.first.second, startGoal.first.first), (*_map)(startGoal.second.second, startGoal.second.first)));
		
		// Save Robot Positions
		currentPositions.push_back(startGoal.first);
	}
}

DStarLite::DStarLite(int MapWithConstantCost, int mapSizeX, int mapSizeY, std::vector<position4D> startGoalPositions, double inGoalCostObjects)
	: goalCostObjects(inGoalCostObjects)
{
	// Prepare real and robot image buffers
	const int img_width = mapSizeX, img_height = mapSizeY;

	// Make the map
	Map* map = new Map(img_height, img_width);
	_map = map;

	// Build map
	for (int i = 0; i < img_height; i++)
	{
		for (int j = 0; j < img_width; j++)
		{
			(*map)(i, j)->cost = MapWithConstantCost;
		}
	}

	// Make planner
	for each (position4D startGoal in startGoalPositions)
	{
		_planner.push_back(new Planner(_map, (*_map)(startGoal.first.second, startGoal.first.first), (*_map)(startGoal.second.second, startGoal.second.first)));
		
		// Save Robot Positions
		currentPositions.push_back(startGoal.first);
	}
}

/**
 * Deconstructor.
 */
DStarLite::~DStarLite()
{
	delete _map;
}

void DStarLite::setNewStart(position2D start)
{
	_planner[0]->start((*_map)(start.second, start.first));
}
void DStarLite::setObstacle(position2D obstacle)
{
	_planner[0]->update((*_map)(obstacle.second, obstacle.first), Map::Cell::COST_UNWALKABLE);
}

void DStarLite::deleteObstacleFromMap(position2D obstacle)
{
	_planner[0]->update((*_map)(obstacle.second, obstacle.first), 1.0);
}

void DStarLite::setNewStart(position2D start, int plannerIndex)
{
	_planner[plannerIndex]->start((*_map)(start.second, start.first));
}

void DStarLite::setObstacle(position2D obstacle, int plannerIndex, double cost, int width, int height)
{
	// Incorporates Minkowski Difference (Rectangle)
	for (int h = -height; h < height; h++)
		for (int w = -width; w < width; w++)
			if(_map->has(obstacle.second + h, obstacle.first + w))
				_planner[plannerIndex]->update((*_map)(obstacle.second + h, obstacle.first + w), cost);
}

void DStarLite::deleteObstacleFromMap(position2D obstacle, int plannerIndex, double cost, int width, int height)
{
	// Incorporates Minkowski Difference (Rectangle)
	for (int h = -height; h < height; h++)
		for (int w = -width; w < width; w++)
			if (_map->has(obstacle.second + h, obstacle.first + w))
				_planner[plannerIndex]->update((*_map)(obstacle.second + h, obstacle.first + w), cost);
}

void DStarLite::printMap()
{
	for (unsigned int y = _map->rows(); y > 0; y--)
	{
		for (unsigned int x = 0; x < _map->cols(); x++)
		{
			std::cout << "(" << x << ", " << y - 1 << "): " << (*_map)(y - 1, x)->cost << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
}

std::pair<unsigned int, unsigned int> DStarLite::step(position2D current, position2D obstacle)
{
	setObstacle(obstacle);
	setNewStart(current);
	list<Map::Cell*> tmpPath = replan();
	deleteObstacleFromMap(obstacle);

	std::pair<unsigned int, unsigned int> currentPath = std::make_pair(-99, -99);
	// Step
	if (tmpPath.size() != 1)
	{
		tmpPath.pop_front();
		currentPath = std::make_pair(tmpPath.front()->x(), tmpPath.front()->y());
	}

	return currentPath;
}

std::vector<position2D> DStarLite::step(std::vector<position2D> currentPositions)
{
	std::vector<position2D> newPositions;

	setNewStart(currentPositions[0], 0);
	std::vector<list<Map::Cell*>> paths;
	paths.push_back(replan(0));

	position2D newPos = currentPositions[0];
	// Step
	if (paths[0].size() != 1)
	{
		paths[0].pop_front();
		newPos = std::make_pair(paths[0].front()->x(), paths[0].front()->y());
	}
	newPositions.push_back(newPos);

	for (unsigned int index = 1; index < currentPositions.size(); index ++) 
	{
		// Set Obstacles
		int obsIndex = index - 1;
		int cost = (5 * 5 + 3) * 10.0;
		for (list<Map::Cell*>::iterator cell = paths[obsIndex].begin(); cell != paths[obsIndex].end(); ++cell)
			for (unsigned int pi = index; pi < _planner.size(); pi++)
				setObstacle(std::make_pair((*cell)->x(), (*cell)->y()), pi, cost--);
		// Object self
		for (unsigned int pi2 = index; pi2 < _planner.size(); pi2++)
			setObstacle(std::make_pair(paths[obsIndex].front()->x(), paths[obsIndex].front()->y()), pi2, 99);
			
		setNewStart(currentPositions[index], index);
		paths.push_back(replan(index));

		newPos = currentPositions[index];
		// Step
		if (paths[index].size() != 1)
		{
			paths[index].pop_front();
			newPos = std::make_pair(paths[index].front()->x(), paths[index].front()->y());
		}
		newPositions.push_back(newPos);
	}

	// Delete Obstacles
	for (unsigned int robot = 1; robot < currentPositions.size(); robot++)
		for (list<Map::Cell*>::iterator cell = paths[robot - 1].begin(); cell != paths[robot - 1].end(); ++cell)
			for (unsigned int pi = robot; pi < _planner.size(); pi++)
				deleteObstacleFromMap(std::make_pair((*cell)->x(), (*cell)->y()), pi);

	return newPositions;
}

std::vector<std::vector<position2D>> DStarLite::calculatePaths()
{
	std::vector<std::vector<position2D>> paths;
	std::vector<position2D> startPos;
	for (unsigned int robot = 0; robot < currentPositions.size(); robot++)
		startPos.push_back(currentPositions[robot]);
	paths.push_back(startPos);

	while (true) 
	{
		bool bFinished = true;
		for (unsigned int robot = 0; robot < currentPositions.size(); robot++)
		{
			Map::Cell* goal = _planner[robot]->goal();
			if (currentPositions[robot] != std::make_pair(goal->x(), goal->y()))
				bFinished = false;
		}
		if (bFinished == true)
		{
			return paths;
			break;
		}
		// ToDo: Prioritize List
		currentPositions = step(currentPositions);
		paths.push_back(currentPositions);
	}
}