#include "stdafx.h"
#include <iostream>
#include "cell.h"
#include <iostream>
#include <random>

using namespace std;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// test connection
bool testConnection(Eigen::VectorXd firstNode, Eigen::VectorXd secondNode, WormCell& cell, float stepsize = 0.025f)
{
	Eigen::VectorXd segment(secondNode - firstNode), delta(5);
	delta = segment.normalized() * stepsize; // stepsize .025
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

std::default_random_engine generator;

Eigen::VectorXd nextGaussianVector(Eigen::VectorXd baseVector)
{
	std::normal_distribution<float> x(baseVector[0], 0.001f);
	std::normal_distribution<float> y(baseVector[1], 0.001f);
	std::normal_distribution<float> a(RAD2DEG(baseVector[2]), 1);
	std::normal_distribution<float> b(RAD2DEG(baseVector[3]), 1);
	std::normal_distribution<float> c(RAD2DEG(baseVector[4]), 1);

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

#define TEST_CASE 0
#ifdef TEST_CASE
#if TEST_CASE == 0
	// Example
	//cout << "Example" << endl;
	qStart << .0, -.2, DEG2RAD(0.), DEG2RAD(0.), DEG2RAD(0.);
	qGoal << .6, .6, DEG2RAD(90.f), DEG2RAD(0.f), DEG2RAD(0.f);

	/*
	//qStart << 0.986542f, 0.92289f, DEG2RAD(132.256f), DEG2RAD(-11.9016f), DEG2RAD (-107.666f);
	//qGoal << 0.788461f, 0.56383f, DEG2RAD(141.03f), DEG2RAD(-4.15792f), DEG2RAD(-116.924f);
	 
	Eigen::VectorXd segment(qGoal - qStart), delta(5);
	delta = segment.normalized() * stepsize;
	int steps = int(segment.norm() / stepsize);

	do
	{
		if (!cell.CheckPosition(qStart))
		{
			for (int i = 0; i < 100; ++i)
			{
				path.push_back(qStart);
				qStart += delta * .01f;
			}
		}
		else
		{
			path.push_back(qStart);
			qStart += delta;
		}
	} while (--steps > 0);

	path.push_back(qGoal);
	reverse(path.begin(), path.end());
	write_easyrob_program_file(path, "example.prg", false);
	path.clear();
	*/
	// !Example
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
	
	int nNodes = 25000;
	graph_t g; //(nNodes + 2);
	/*Eigen::VectorXd v(5);
	Eigen::VectorXd v2(5);

	v << 0.986542f, 0.92289f, DEG2RAD(132.256f), DEG2RAD(-11.9016f), DEG2RAD(-107.666f);
	v2 << 0.788461f, 0.56383f, DEG2RAD(141.03f), DEG2RAD(-4.15792f), DEG2RAD(-116.924f);

	bool what = testConnection( v, v2, cell);*/

	
	// 1. step: building up a graph g consisting of nNodes vertices
	cout << "1. Step: building " << nNodes << " nodes for the graph" << endl;

	/*
	// Gaussian Sampling Strategy
	int numberNodes = 0;
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
	*/

	
	// NextRandomCfree Sampling Strategy
	for (int i = 0; i<nNodes; ++i) {
		Eigen::VectorXd sample = cell.NextRandomCfree();
		rtree.insert(make_pair(MyWorm(sample), i));
		boost::add_vertex(g);
		g[i].q_ = sample;
	}
	
	std::cout << "Number Of Nodes: " << nNodes << std::endl;

    // 2. step: building edges for the graph, if the connection of 2 nodes are in free space
    cout << "2. Step: buildung edges for the graph" << endl;
	std::vector<std::pair<MyWorm, int>> result;
	int numberEdges = 0;
	for (int i = 0; i < nNodes; ++i) {
		rtree.query(boost::geometry::index::nearest(MyWorm(g[i].q_), 5), std::back_inserter(result));
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
	//rtree.insert(make_pair(MyWorm(qStart), nNodes));
	result.clear();
	boost::add_vertex(g);
	g[nNodes].q_ = qStart;
	rtree.query(boost::geometry::index::nearest(MyWorm(qStart), 100), std::back_inserter(result));
	bool bFoundEdge = false;
	for each (std::pair<MyWorm, int> worm in result)
	{
		if (testConnection(worm.first.q(), qStart, cell))
		{
			boost::add_edge(nNodes, worm.second, g);
			bFoundEdge = true;
			break;
		}
	}
	result.clear();
	if (!bFoundEdge)
		std::cout << "Could not connect Start Configuration";

    // 4. Step: connecting goal configuration to graph
    cout << "4. Step: connecting goal configuration to graph" << endl;
	//rtree.insert(make_pair(MyWorm(qGoal), nNodes + 1));
	result.clear();
	boost::add_vertex(g);
	g[nNodes + 1].q_ = qGoal;
	rtree.query(boost::geometry::index::nearest(MyWorm(qGoal), 100), std::back_inserter(result));
	bFoundEdge = false;
	for each (std::pair<MyWorm, int> worm in result)
	{
		if (testConnection(worm.first.q(), qGoal, cell))
		{
			boost::add_edge(nNodes + 1, worm.second, g);
			bFoundEdge = true;
			break;
		}
	}
	result.clear();
	if (!bFoundEdge)
		std::cout << "Could not connect Goal Configuration";

	write_gnuplot_file(g, "VisibilityGraph.dat");
	
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

		if (path.size() > 100000)
			return EXIT_FAILURE;
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

	write_easyrob_program_file(path, "prm.prg");
	
    return EXIT_SUCCESS;
}
