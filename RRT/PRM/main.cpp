#include "stdafx.h"
#include <iostream>
#include "cell.h"
#include <iostream>
#include <random>
#include <chrono>

using namespace std;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// test connection
bool testConnection(Eigen::VectorXd firstNode, Eigen::VectorXd secondNode, WormCell& cell, float stepsize = 0.025f)
{
	Eigen::VectorXd segment(secondNode - firstNode), delta(5);
	delta = segment.normalized() * stepsize;
	int steps = int(segment.norm() / stepsize);
	Eigen::VectorXd tmpNode = firstNode;

	do
	{
		if (!cell.CheckPosition(tmpNode))
		{
			return false;
		}
		else
		{
			tmpNode += delta;
		}
	} while (--steps > 0);

	return true;
}

// Generate Random gaussian distributed Vectors
std::default_random_engine generator;
Eigen::VectorXd nextGaussianVector(Eigen::VectorXd baseVector)
{
	std::normal_distribution<float> x(baseVector[0], 0.01f);
	std::normal_distribution<float> y(baseVector[1], 0.01f);
	std::normal_distribution<float> a(RAD2DEG(baseVector[2]), 5);
	std::normal_distribution<float> b(RAD2DEG(baseVector[3]), 5);
	std::normal_distribution<float> c(RAD2DEG(baseVector[4]), 5);

	Eigen::VectorXd randomVector(5);

	randomVector << x(generator), y(generator), DEG2RAD(a(generator)), DEG2RAD(b(generator)), DEG2RAD(c(generator));

	return randomVector;
}

/***********************************************************************************************************************************/
int _tmain(int argc, _TCHAR* argv[])
{
    WormCell cell;
    Eigen::VectorXd qStart(5), qGoal(5), q(5);
    vector<Eigen::VectorXd> path; // create a point vector for storing the path
    knn_rtree_t rtree;
    const float stepsize = .025f;
	int number_of_nearest_neighbour = 6;

#define TEST_CASE 0
#ifdef TEST_CASE
#if TEST_CASE == 0
	cout << "Test case 0" << endl;

	// super Test 0
	//qStart << .0, -.2, DEG2RAD(0.), DEG2RAD(0.), DEG2RAD(0.);
	//qGoal << .6, .6, DEG2RAD(90.f), DEG2RAD(0.f), DEG2RAD(0.f);

	qStart << 0., 0., 0., 0., 0.;
	qGoal << .6, .9, DEG2RAD(-90.), DEG2RAD(-180.), DEG2RAD(180.);
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
	qGoal << .6, 1.05, DEG2RAD(-90.f), 0., 0.; // Why would this be unreachable ?
#elif TEST_CASE == 10
	cout << "Test case 10 / unreachable start" << endl;
	qStart << .6, 1.05, DEG2RAD(-90.f), 0., 0.; // Why would this be unreachable ?
	qGoal << .6, .95, DEG2RAD(-90.f), 0., 0.;
#endif
#endif
	
	auto start_time = std::chrono::high_resolution_clock::now();
	graph_t g; // the graph
	
	// 1. step: building up a graph g consisting of nNodes vertices
	cout << "1. Step: building nodes for the graph" << endl;
	
#define SAMPLING_STRATEGY 0
#ifdef SAMPLING_STRATEGY
#if SAMPLING_STRATEGY == 0
	std::cout << "Gaussian Sampling Strategy" << std::endl;
	int nNodes = 120000; // for gaussian distribution
	int numberNodes = 0;

	// Create random Samples first to cover free space better
	for (int i = 0; i<1000; ++i) {
		Eigen::VectorXd sample = cell.NextRandomCfree();
		rtree.insert(make_pair(MyWorm(sample), i));
		boost::add_vertex(g);
		g[i].q_ = sample;
		numberNodes++;
	}
	
	// Gaussian Sampling Strategy
	for (int i = 0; i<nNodes; ++i) {
		// Get random sample in the space
		Eigen::VectorXd sample = cell.NextRandomCspace();

		// create a second sample gaussian distribution
		Eigen::VectorXd sample2 = nextGaussianVector(sample);

		if (cell.CheckPosition(sample) && !cell.CheckPosition(sample2))
		{
			rtree.insert(make_pair(MyWorm(sample), numberNodes));
			boost::add_vertex(g);
			g[numberNodes].q_ = sample;
			numberNodes++;
		}
		else if (!cell.CheckPosition(sample) && cell.CheckPosition(sample2))
		{
			rtree.insert(make_pair(MyWorm(sample2), numberNodes));
			boost::add_vertex(g);
			g[numberNodes].q_ = sample2;
			numberNodes++;
		}
	}
	nNodes = numberNodes;
	

#elif SAMPLING_STRATEGY == 1
	std::cout << "Basic Sampling Strategy" << std::endl;
	int nNodes = 12000; // basic strategy

	// NextRandomCfree Sampling Strategy
	for (int i = 0; i<nNodes; ++i) {
		Eigen::VectorXd sample = cell.NextRandomCfree();
		rtree.insert(make_pair(MyWorm(sample), i));
		boost::add_vertex(g);
		g[i].q_ = sample;
	}
#endif
#endif
	
	std::cout << "Number Of Nodes: " << nNodes << std::endl;

    // 2. step: building edges for the graph, if the connection of 2 nodes are in free space
    cout << "2. Step: buildung edges for the graph" << endl;
	std::vector<std::pair<MyWorm, int>> result;
	unsigned int numberEdges = 0;
	for (int i = 0; i < nNodes; ++i) {
		rtree.query(boost::geometry::index::nearest(MyWorm(g[i].q_), number_of_nearest_neighbour), std::back_inserter(result));
		for each (std::pair<MyWorm, int> worm in result)
		{
			if (testConnection(worm.first.q(), g[i].q_, cell))
			{
				boost::add_edge(i, worm.second, g);
				numberEdges++;
			}
		}
		result.clear();
	}
	std::cout << "Number Of Edges: " << numberEdges << std::endl;

    // 3. Step: connecting start configuration to graph
    cout << "3. Step: connecting start configuration to graph" << endl;
	result.clear();
	boost::add_vertex(g);
	g[nNodes].q_ = qStart;
	rtree.query(boost::geometry::index::nearest(MyWorm(qStart), 100), std::back_inserter(result));
	int bFoundEdge = 0;
	for each (std::pair<MyWorm, int> worm in result)
	{
		if (testConnection(worm.first.q(), qStart, cell))
		{
			boost::add_edge(nNodes, worm.second, g);
			bFoundEdge++;
			if (bFoundEdge > number_of_nearest_neighbour) break;
		}
	}
	result.clear();
	if (bFoundEdge == 0)
	{
		std::cout << "Failure: Could not connect Start Configuration" << std::endl;
		return EXIT_FAILURE;
	}

    // 4. Step: connecting goal configuration to graph
    cout << "4. Step: connecting goal configuration to graph" << endl;
	result.clear();
	boost::add_vertex(g);
	g[nNodes + 1].q_ = qGoal;
	rtree.query(boost::geometry::index::nearest(MyWorm(qGoal), 100), std::back_inserter(result));
	bFoundEdge = 0;
	for each (std::pair<MyWorm, int> worm in result)
	{
		if (testConnection(worm.first.q(), qGoal, cell))
		{
			boost::add_edge(nNodes + 1, worm.second, g);
			bFoundEdge++;
			if (bFoundEdge > number_of_nearest_neighbour) break;
		}
	}
	result.clear();
	if (bFoundEdge == 0)
	{
		std::cout << "Failure: Could not connect Goal Configuration" << std::endl;
		return EXIT_FAILURE;
	}
	
    // 5. Step: searching for shortest path
    cout << "5. Step: searching for shortest path" << endl;

	typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_descriptor;
	vertex_descriptor start = boost::vertex(nNodes, g);
	std::vector<vertex_descriptor> p(num_vertices(g));
	boost::dijkstra_shortest_paths(g, start, boost::predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))));

	// Write the path: reverse search through predecessor map
	int currentVertex = nNodes + 1; // end point
	while ((currentVertex != nNodes)) // start point
	{
		path.push_back(g[currentVertex].q_);
		currentVertex = p[currentVertex];

		if (path.size() > nNodes)
		{
			std::cout << "Failure: Could not find a path from start to goal!";
			return EXIT_FAILURE;
		}
	}
	path.push_back(g[currentVertex].q_);

	std::cout << "Path size: " << path.size() << std::endl;
	std::cout << "Path Refinement:" << std::endl;

	// Path Refinement
	// Go through path
	reverse(path.begin(), path.end());
	for (unsigned int current = 0; current < path.size(); current++)
	{
		// Find last successor
		int lastSuccessor = current + 1;
		for (unsigned int successor = current + 2; successor < path.size(); successor++)
		{
			if (testConnection(path[current], path[successor], cell))
				lastSuccessor = successor;
		}
		if (lastSuccessor != current + 1)
			path.erase(path.begin() + current + 1, path.begin() + lastSuccessor);
	}
	reverse(path.begin(), path.end());
	
	std::cout << "Path size: " << path.size() << std::endl;

	auto end_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> diff = end_time - start_time;

	std::cout << "Time to find a path of size " << path.size() << " for " << nNodes << " nodes and " << numberEdges << " edges: " << diff.count() << " s\n";

	std::vector<int> component(boost::num_vertices(g));
	int num = boost::connected_components(g, &component[0]);
	std::cout << "Number of connected Components: " << num << std::endl;
	std::cout << "Number of Nearest Neighbours: " << number_of_nearest_neighbour << std::endl;

	write_gnuplot_file(g, "VisibilityGraph.dat");
	write_easyrob_program_file(path, "prm.prg");
	
    return EXIT_SUCCESS;
}
