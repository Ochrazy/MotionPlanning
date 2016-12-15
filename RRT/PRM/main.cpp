#include "stdafx.h"
#include <iostream>
#include "cell.h"

#include <limits>

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>

using namespace std;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::segment<point_type> segment_type;

typedef std::pair<segment_type, edge_t> rtree_value_edge;
typedef boost::geometry::index::rtree<rtree_value_edge, boost::geometry::index::quadratic<16>> knn_rtree_edge_t;

point_type convertEigenTo2DPoint(Eigen::VectorXd inPoint)
{
	point_type point;
	point.set<0>(inPoint[0]);
	point.set<1>(inPoint[1]);
	return point;
}

double GetClosestPoint(point_type A, point_type B, point_type P)
{
	point_type AP = P;
	boost::geometry::subtract_point(AP, A);
	point_type AB = B;
	boost::geometry::subtract_point(AB, A);

	double ab2 = AB.x() * AB.x() + AB.y() * AB.y();
	double ap_ab = AP.x() * AB.x() + AP.y() * AB.y();
	double t = ap_ab / ab2;

	return t;
}

void addEdge(int firstIndex, int secondIndex, graph_t& g, knn_rtree_edge_t& rtreeEdge)
{
	// To graph
	boost::add_edge(firstIndex, secondIndex, g);
	// To R-Tree
	segment_type edge(convertEigenTo2DPoint(g[firstIndex].q_), convertEigenTo2DPoint(g[secondIndex].q_));
	rtreeEdge.insert(make_pair(edge, boost::edge(firstIndex, secondIndex, g).first));
}

// Add node and return index
int addNode(Eigen::VectorXd node, graph_t& g)
{
	int index = boost::num_vertices(g);
	boost::add_vertex(g);
	g[index].q_ = node;
	return index;
}

int addSplitNode(Eigen::VectorXd splitNode, int source, int target, graph_t& g, knn_rtree_edge_t& rtreeEdge)
{
	// Add Split Node (qn)
	int splitNodeIndex = addNode(splitNode, g);

	// Remove and add edges
	boost::remove_edge(source, target, g);
	segment_type edgeSegment(convertEigenTo2DPoint(g[source].q_), convertEigenTo2DPoint(g[target].q_));
	rtreeEdge.remove(make_pair(edgeSegment, boost::edge(source, target, g).first));
	addEdge(splitNodeIndex, source, g, rtreeEdge);
	addEdge(splitNodeIndex, target, g, rtreeEdge);

	return splitNodeIndex;
}

struct NearestEdgeQN
{
	int index_source;
	int index_target;
	double t;
	Eigen::VectorXd qn;
};

NearestEdgeQN calculateQNearest(point_type qrand, knn_rtree_edge_t& rtreeEdge, graph_t& g)
{
	// Find closest Edge to qrand
	std::vector<std::pair<segment_type, edge_t>> resultEdge;
	rtreeEdge.query(boost::geometry::index::nearest(qrand, 1), std::back_inserter(resultEdge));
	int index_source = resultEdge.back().second.m_source;
	int index_target = resultEdge.back().second.m_target;

	// Calculate qn
	Eigen::VectorXd qn;
	double t = GetClosestPoint(resultEdge.back().first.first, resultEdge.back().first.second, qrand);
	if (t < 0.0) qn = g[index_source].q_;
	else if (t > 1.0) qn = g[index_target].q_;
	else  qn = g[index_source].q_ + (g[index_target].q_ - g[index_source].q_) * t;

	NearestEdgeQN result = { index_source, index_target, t, qn };
	return result;
}

vector<Eigen::VectorXd> calculateShortestPath(graph_t& g, int startIndex, int goalIndex, WormCell& cell)
{
	vector<Eigen::VectorXd> path;
	typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_descriptor;
	vertex_descriptor start = boost::vertex(startIndex, g);
	std::vector<vertex_descriptor> p(num_vertices(g));
	boost::dijkstra_shortest_paths(g, start, boost::predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))));

	// Write the path: reverse search through predecessor map
	int currentVertex = goalIndex; // end point
	while ((currentVertex != startIndex)) // start point
	{
		path.push_back(g[currentVertex].q_);
		currentVertex = p[currentVertex];

		if (path.size() > num_vertices(g))
		{
			std::cout << "Failure: Could not find a path from start to goal!";
			return path;
		}
	}
	path.push_back(g[currentVertex].q_);

	reverse(path.begin(), path.end());

	return path;
}

void refinePath(vector<Eigen::VectorXd>& path, WormCell& cell)
{
	// Path Refinement
	// Go through path
	for (unsigned int current = 0; current < path.size(); current++)
	{
		// Find last successor
		int lastSuccessor = current + 1;
		for (unsigned int successor = current + 2; successor < path.size(); successor++)
		{
			if (cell.CheckMotion(path[current], path[successor], 0.025f))
				lastSuccessor = successor;
		}
		if (lastSuccessor != current + 1)
			path.erase(path.begin() + current + 1, path.begin() + lastSuccessor);
	}
}

/***********************************************************************************************************************************/
int _tmain(int argc, _TCHAR* argv[])
{
	WormCell cell;
	Eigen::VectorXd qStart(5), qGoal(5), q(5);
	vector<Eigen::VectorXd> path; // create a point vector for storing the path
	graph_t g;
	graph_t gb;
	int solutionIndex = -1;
	int solutionIndexB = -1;
	const float stepsize = .025f;

#define TEST_CASE 1
#ifdef TEST_CASE
#if TEST_CASE == 0

	//qStart << 0.5, 0.5, 0., 0., 0.;
	qStart << -.17, -.17, 0., 0., 0.;
	qGoal << .6, .9, DEG2RAD(-90.), DEG2RAD(-180.), DEG2RAD(180.);
	//qGoal << .5, .46, DEG2RAD(-180.f), 0., 0.;
	//qGoal << .17, -.17, 0, 0., 0.;

#elif TEST_CASE == 1
	cout << "Test case 1" << endl;
	qStart << .6, .1, 0., 0., 0.;
	qGoal << .1, .8, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 2
	cout << "Test case 2" << endl;
	qStart << .1, .8, DEG2RAD(-90.f), DEG2RAD(-180.f), DEG2RAD(180.f);
	qGoal << .9, .4, DEG2RAD(-90.f), DEG2RAD(-180.f), DEG2RAD(180.f);
#elif TEST_CASE == 3
	cout << "Test case 3" << endl;
	qStart << .9, .4, DEG2RAD(-90.f), DEG2RAD(-180.f), DEG2RAD(180.f);
	qGoal << .9, .75, DEG2RAD(-180.f), 0., 0.;
#elif TEST_CASE == 4
	cout << "Test case 4" << endl;
	qStart << .9, .75, DEG2RAD(-180.f), 0., 0.;
	qGoal << .5, .45, DEG2RAD(-180.f), 0., 0.;
#elif TEST_CASE == 5
	cout << "Test case 5" << endl;
	qStart << .5, .45, DEG2RAD(-180.f), 0., 0.;
	qGoal << .6, .95, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 6
	cout << "Test case 6" << endl;
	qStart << .5, .45, DEG2RAD(-180.f), 0., 0.;
	qGoal << .6, .95, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 7
	cout << "Test case 7 / colliding goal" << endl;
	qStart << .6, .95, DEG2RAD(-90.f), 0., 0.;
	qGoal << .7, .95, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 8
	cout << "Test case 8 / colliding start" << endl;
	qStart << .7, .95, DEG2RAD(-90.f), 0., 0.;
	qGoal << .6, .95, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 9
	cout << "Test case 9 / unreachable goal" << endl;
	qStart << .6, .95, DEG2RAD(-90.f), 0., 0.;
	qGoal << .6, 1.05, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 10
	cout << "Test case 10 / unreachable start" << endl;
	qStart << .6, 1.05, DEG2RAD(-90.f), 0., 0.;
	qGoal << .6, .95, DEG2RAD(-90.f), 0., 0.;
#endif
#endif

#define TASK 2
#ifdef TASK
#if TASK == 0
	// First Task
	knn_rtree_edge_t rtreeEdge;
	const int nNodes = 50;
	std::cout << "RRT whitout goal" << std::endl;
	cout << "RRT with number of Nodes:" << nNodes << endl;

	// Add Start and Goal nodes
	addNode(qStart, g);

	for (int i = 1; i < nNodes; ++i)
	{
		// Add first edge
		if (boost::num_edges(g) == 0)
			addEdge(0, addNode(cell.NextRandomCspace(), g), g, rtreeEdge);
		else
		{
			// Find closest Edge to qrand
			Eigen::VectorXd qrand = cell.NextRandomCspace();
			NearestEdgeQN neqn = calculateQNearest(convertEigenTo2DPoint(qrand), rtreeEdge, g);

			// Add Sample (qrand) Node
			int currentSampleIndex = addNode(qrand, g);

			// Add Edges
			if (neqn.t < 0.0f)
				addEdge(neqn.index_source, currentSampleIndex, g, rtreeEdge);
			else if (neqn.t > 1.0f)
				addEdge(neqn.index_target, currentSampleIndex, g, rtreeEdge);
			else
			{
				// Add Split Node (qn)
				int splitNodeIndex = addSplitNode(neqn.qn, neqn.index_source, neqn.index_target, g, rtreeEdge);
				addEdge(splitNodeIndex, currentSampleIndex, g, rtreeEdge);
			}
		}
	}

#elif TASK == 1
	// Second Task
	knn_rtree_edge_t rtreeEdge;
	const int nNodes = 10000;
	std::cout << "RRT with goal bias" << std::endl;
	std::cout << "RRT with number of Nodes:" << nNodes << endl;

	bool bGoToGoal = false;
	// Add Start and Goal nodes
	addNode(qStart, g);

	for (int i = 1; i < nNodes; ++i)
	{
		// Add first edge
		if (boost::num_edges(g) == 0)
		{
			// Calculate qs
			Eigen::VectorXd qs, cObstacle, qn, qrand = cell.NextRandomCspace();
			bool hitObs = cell.FirstContact(qs, cObstacle, g[0].q_, qrand, stepsize);
			if ((qs - g[0].q_).norm() > stepsize) //qs != qStart)
			{
				// Add first edge and node
				addEdge(0, addNode(qs, g), g, rtreeEdge);
				bGoToGoal = true;
			}
		}
		else
		{
			// Try to connect to goal (20%)
			Eigen::VectorXd sample = cell.NextRandomCspace();
			if ((i % (int)(nNodes / 20) == 0) || bGoToGoal == true)
			{
				sample = qGoal;
				bGoToGoal = false;
			}

			// Stopping Configuration
			Eigen::VectorXd qs, cObstacle, qrand = sample;

			// Find closest Edge to qrand
			NearestEdgeQN neqn = calculateQNearest(convertEigenTo2DPoint(qrand), rtreeEdge, g);

			// Calculate qs
			bool hitObs = cell.FirstContact(qs, cObstacle, neqn.qn, qrand, stepsize);

			if (cell.CheckMotion(qs, qGoal, stepsize))
			{
				std::cout << "Success!!!" << std::endl;
				int currentSampleIndex = addNode(qs, g);
				if (neqn.t < 0.0f)
					addEdge(neqn.index_source, currentSampleIndex, g, rtreeEdge);
				else if (neqn.t > 1.0f)
					addEdge(neqn.index_target, currentSampleIndex, g, rtreeEdge);
				else
				{
					int splitNodeIndex = addSplitNode(neqn.qn, neqn.index_source, neqn.index_target, g, rtreeEdge);
					addEdge(splitNodeIndex, currentSampleIndex, g, rtreeEdge);
				}

				solutionIndex = addNode(qGoal, g);
				addEdge(currentSampleIndex, solutionIndex, g, rtreeEdge);
				break;
			}

			// New sample
			if ((qs - neqn.qn).norm() > stepsize)  //qs != qn)
			{
				// Add Sample (qs) Node
				int currentSampleIndex = addNode(qs, g);

				// Add Edges
				if (neqn.t < 0.0f)
					addEdge(neqn.index_source, currentSampleIndex, g, rtreeEdge);
				else if (neqn.t > 1.0f)
					addEdge(neqn.index_target, currentSampleIndex, g, rtreeEdge);
				else
				{
					// Add Split Node (qn)
					int splitNodeIndex = addSplitNode(neqn.qn, neqn.index_source, neqn.index_target, g, rtreeEdge);
					addEdge(splitNodeIndex, currentSampleIndex, g, rtreeEdge);
				}
			}
		}
	}
#elif TASK == 2
	// Bonus Task
	knn_rtree_edge_t rtreeEdge;
	knn_rtree_edge_t rtreeEdgeB;
	const int nNodes = 10000;
	std::cout << "Bidirectional balanced RRT" << std::endl;
	std::cout << "RRT with number of Nodes:" << nNodes << endl;

	// Add Start and Goal nodes
	addNode(qStart, g);
	addNode(qGoal, gb);

	for (int i = 1; i < nNodes; ++i)
	{
		// Add first edge
		if (boost::num_edges(g) == 0)
		{
			// Calculate qs
			Eigen::VectorXd qs, cObstacle, qn, qrand = cell.NextRandomCspace();
			bool hitObs = cell.FirstContact(qs, cObstacle, g[0].q_, qrand, stepsize);
			if ((qs - g[0].q_).norm() > stepsize) //qs != qStart)
			{
				// Add first edge and node
				addEdge(0, addNode(qs, g), g, rtreeEdge);
			}
		}
		else
		{
			// Stopping Configuration
			Eigen::VectorXd qs, cObstacle, qrand = cell.NextRandomCspace();

			// Find closest Edge to qrand
			NearestEdgeQN neqn = calculateQNearest(convertEigenTo2DPoint(qrand), rtreeEdge, g);

			// Calculate qs
			bool hitObs = cell.FirstContact(qs, cObstacle, neqn.qn, qrand, stepsize);

			// New sample
			if ((qs - neqn.qn).norm() > stepsize)  //qs != qn)
			{
				// Add Sample (qs) Node
				int currentSampleIndex = addNode(qs, g);

				// Add Edges
				if (neqn.t < 0.0f)
					addEdge(neqn.index_source, currentSampleIndex, g, rtreeEdge);
				else if (neqn.t > 1.0f)
					addEdge(neqn.index_target, currentSampleIndex, g, rtreeEdge);
				else
				{
					// Add Split Node (qn)
					int splitNodeIndex = addSplitNode(neqn.qn, neqn.index_source, neqn.index_target, g, rtreeEdge);
					addEdge(splitNodeIndex, currentSampleIndex, g, rtreeEdge);
				}

				// ----------------------------------------------------------------------------
				// Second Tree 
				// Add first edge
				if (boost::num_edges(gb) == 0)
				{
					// Calculate qs
					Eigen::VectorXd qsB, cObstacleB, qnB, qrandB = cell.NextRandomCspace();
					bool hitObs = cell.FirstContact(qsB, cObstacleB, gb[0].q_, qrandB, stepsize);
					if ((qsB - gb[0].q_).norm() > stepsize) //qs != qStart)
					{
						// Add first edge and node
						addEdge(0, addNode(qsB, gb), gb, rtreeEdgeB);
					}
				}
				else
				{
					// Find closest Edge to qrand
					qrand = qs;
					NearestEdgeQN neqnB = calculateQNearest(convertEigenTo2DPoint(qrand), rtreeEdgeB, gb);

					// Calculate qsB
					Eigen::VectorXd qsB, cObstacleB;
					bool hitObs = cell.FirstContact(qsB, cObstacle, neqnB.qn, qrand, stepsize);
					int qsBIndex = -1;
					// New sample
					if ((qsB - neqnB.qn).norm() > stepsize)  //qs != qn)
					{
						// Add Sample (qs) Node
						qsBIndex = addNode(qsB, gb);

						// Add Edges
						if (neqnB.t < 0.0f)
							addEdge(neqnB.index_source, qsBIndex, gb, rtreeEdgeB);
						else if (neqnB.t > 1.0f)
							addEdge(neqnB.index_target, qsBIndex, gb, rtreeEdgeB);
						else
						{
							// Add Split Node (qn)
							int splitNodeIndex = addSplitNode(neqnB.qn, neqnB.index_source, neqnB.index_target, gb, rtreeEdgeB);
							addEdge(splitNodeIndex, qsBIndex, gb, rtreeEdgeB);
						}
					}
					if ((qsB - qs).norm() < stepsize && cell.CheckMotion(qsB, qs, 0.01f)) //qsB == qs)
					{
						std::cout << "Success!!!" << std::endl;
						solutionIndex = currentSampleIndex;
						solutionIndexB = qsBIndex;
						break;
					}
				}
				if (boost::num_vertices(gb) < boost::num_vertices(g))
				{
					g.swap(gb);
					rtreeEdge.swap(rtreeEdgeB);
				}
			}
		}
	}

#endif
#endif

	// Calculate shortest Paths
	if (solutionIndex != -1)
	{
		// Connect paths 
		vector<Eigen::VectorXd> pathG = calculateShortestPath(g, 0, solutionIndex, cell);
		std::cout << "Path size of G: " << pathG.size() << std::endl;

		if (solutionIndexB != -1)
		{
			vector<Eigen::VectorXd> pathGB = calculateShortestPath(gb, 0, solutionIndexB, cell);
			std::cout << "Path size of GB: " << pathGB.size() << std::endl;
			reverse(pathGB.begin(), pathGB.end());
			pathG.insert(pathG.end(), pathGB.begin(), pathGB.end());
			std::cout << "Path size of Complete Graph: " << pathG.size() << std::endl;
		}
		// Refine Path
		refinePath(pathG, cell);
		reverse(pathG.begin(), pathG.end());
		std::cout << "Path size of Refined Graph: " << pathG.size() << std::endl;

		write_easyrob_program_file(pathG, "rrt.prg");
	}

	// Print Graphs
	std::cout << "Number of Nodes in g: " << boost::num_vertices(g) << std::endl;
	write_gnuplot_file(g, "VisibilityGraph.dat");

	std::cout << "Number of Nodes in gb: " << boost::num_vertices(gb) << std::endl;
	write_gnuplot_file(gb, "VisibilityGraphB.dat");

	return EXIT_SUCCESS;
}
