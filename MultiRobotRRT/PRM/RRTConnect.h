#pragma once
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include "cell.h"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
namespace bgm = boost::geometry::model;

template<typename _ROBOT_TYPE>
class RRTConnect
{
public:
	RRTConnect();
	RRTConnect(std::vector<std::vector<Eigen::VectorXd>> inCoordinationDiagramPaths);

	// RRTConnect
	std::vector<Eigen::VectorXd> doRRTConnect(Eigen::VectorXd qStart, Eigen::VectorXd qGoal);

private:
	// Coordination Diagram
	bool bIsCoordinationDiagram;

	// RTree
	typedef bgi::rtree<std::pair<bgm::segment<_ROBOT_TYPE>, edge_t>, bgi::quadratic<16>> MultiRobotRtree;

	const float stepsize = .025f;

	Cell<_ROBOT_TYPE, MyCell> cell;

	// Get Closest Point on Line (A,B) to Point P
	// Projection of P onto Line (A, B) 
	double GetClosestPoint(Eigen::VectorXd A, Eigen::VectorXd B, Eigen::VectorXd P);

	// Add Edge to Graph and Rtree
	void addEdge(int firstIndex, int secondIndex, graph_t& g, MultiRobotRtree& rtree);

	// Add node and return index
	int addNode(Eigen::VectorXd node, graph_t& g);

	// Split the Edge and add all the new Nodes/Edges
	int addSplitNode(Eigen::VectorXd splitNode, int source, int target, graph_t& g, MultiRobotRtree& rtree);

	// Calculate Nearest Edge/Node to qrand
	std::pair<bgm::segment<_ROBOT_TYPE>, edge_t> RRTConnect::calculateNearestEdge(Eigen::VectorXd q, MultiRobotRtree& rtree);
	Eigen::VectorXd getPointOnSegment(Eigen::VectorXd a, Eigen::VectorXd b, double t);

	// Path
	std::vector<Eigen::VectorXd> RRTConnect::calculateShortestPath(int startIndex, int goalIndex, graph_t& g);
	void refinePath(std::vector<Eigen::VectorXd>& path);
	std::vector<Eigen::VectorXd> convertCDPath(std::vector<Eigen::VectorXd> path);
};

#include "RRTConnect.inl"

typedef RRTConnect<MultiRobot2x2> RRTConnect2x2;
typedef RRTConnect<Robot2> RRTConnect2;
