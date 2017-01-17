/**
 * Simulator.
 *
 * @package		DStarLite
 * @author		Aaron Zampaglione <azampagl@gmail.com>
 * @copyright	Copyright (C) 2011 Aaron Zampaglione
 * @license		MIT
 */
#ifndef DSTARLITE_DSTARLITE_H
#define DSTARLITE_DSTARLITE_H

#include "planner.h"
#include "map.h"

using namespace DSTARLITE;

namespace DSTARLITE
{
	typedef std::pair<unsigned int, unsigned int> position2D;
	typedef std::pair<position2D, position2D> position4D;

	class DStarLite
	{
		public:
			/**
			 * Constructor.
			 * 
			 * @param  char*    name of the simulator
			 * @param  Config   config options
			 */
			DStarLite(position2D start, position2D goal, unsigned int width, unsigned int height, int inGoalCostObjects = 17);
			DStarLite(Map* map, std::vector<position4D> startGoalPositions, int inGoalCostObjects = 17);
			DStarLite(int MapWithConstantCost, int mapSizeX, int mapSizeY, std::vector<position4D> startGoalPositions, int inGoalCostObjects = 17);

			/**
			 * Deconstructor.
			 */
			~DStarLite();

			/**
			 * Main execution method.
			 *
			 * @return  int  successfull
			 */
			list<Map::Cell*> replan();
			list<Map::Cell*> replan(unsigned int plannerIndex);

			void setNewStart(position2D start);
			void setObstacle(position2D start);
			void deleteObstacleFromMap(position2D obstacle);

			void setNewStart(position2D start, int plannerIndex);
			void setObstacle(position2D obstacle, int plannerIndex, int cost = Map::Cell::COST_UNWALKABLE);
			void deleteObstacleFromMap(position2D obstacle, int plannerIndex, int cost = 1.0);
			void printMap();
			position2D step(std::pair<unsigned int, unsigned int> current, std::pair<unsigned int, unsigned int> obstacle);
			std::vector<position2D> step(std::vector<position2D> currentPositions);

			std::vector<std::vector<position2D>> calculatePaths();


		protected:

			/**
			 * @var  Map*  real map, with all obstacles
			 */
			Map* _map;

			/**
			 * @var  Planner*  planner
			 */
			std::vector<Planner*> _planner;

			std::vector<position2D> currentPositions;

			int goalCostObjects;
	};
};

#endif // DSTARLITE_DSTARLITE_H
