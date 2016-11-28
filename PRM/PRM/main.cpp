#include "stdafx.h"
#include <iostream>
#include "cell.h"

using namespace std;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

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
	cout << "Example" << endl;
	 qStart << .0, -.2, DEG2RAD(0.), DEG2RAD(0.), DEG2RAD(0.);
	 qGoal << .6, .6, DEG2RAD(90.f), DEG2RAD(0.f), DEG2RAD(0.f);

	 /*
	Eigen::VectorXd segment(qGoal - qStart), delta(5);
	delta = segment.normalized() * stepsize;
	int steps = int(segment.norm() / stepsize);

	do
	{
		if (!cell.CheckPosition(qStart))
		{
			for (int i = 0; i < 10; ++i)
			{
				path.push_back(qStart);
				qStart += delta * .1f;
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

	const int nNodes = 25000;
	graph_t g(nNodes+2);

	// 1. step: building up a graph g consisting of nNodes vertices
	cout << "1. Step: building " << nNodes << " nodes for the graph" << endl;

	std::default_random_engine generator;
	std::normal_distribution<double> distribution(.5, .25);

	for (int i = 0; i<nNodes; ++i) {
		Eigen::VectorXd sample = cell.NextRandomCfree();
		rtree.insert(make_pair(MyWorm(sample), i));
		g[i].q_ = sample;
	}

    // 2. step: building edges for the graph, if the connection of 2 nodes are in free space
    cout << "2. Step: buildung edges for the graph" << endl;
	std::vector<std::pair<MyWorm, int>> result;
	MyWorm currentWorm;
	
	for (int i = 0; i < nNodes; ++i) {
		currentWorm = MyWorm(g[i].q_);
		rtree.query(boost::geometry::index::nearest(currentWorm, 5), std::back_inserter(result));
		for each (std::pair<MyWorm, int> worm in result)
		{
			// test connection
			currentWorm = MyWorm(g[i].q_);
			Eigen::VectorXd segment(worm.first.q() - currentWorm.q()), delta(5);
			delta = segment.normalized() * stepsize;
			int steps = int(segment.norm() / stepsize);

			bool bAddEdge = true;
			do
			{
				if (!cell.CheckPosition(currentWorm.q()))
				{
					bAddEdge = false;
					break;
				}
				else
				{
					currentWorm.q() += delta;
				}
			} while (--steps > 0);

			if (bAddEdge == true) boost::add_edge(i, worm.second, g);
		}
		result.clear();
	}
	
    // 3. Step: connecting start configuration to graph
    cout << "3. Step: connecting start configuration to graph" << endl;
	//rtree.insert(make_pair(MyWorm(qStart), nNodes));
	result.clear();
	g[nNodes].q_ = qStart;
	currentWorm = MyWorm(qStart);
		rtree.query(boost::geometry::index::nearest(currentWorm, 5), std::back_inserter(result));
		for each (std::pair<MyWorm, int> worm in result)
		{
			// test connection
			currentWorm = MyWorm(qStart);
			Eigen::VectorXd segment(worm.first.q() - currentWorm.q()), delta(5);
			delta = segment.normalized() * stepsize;
			int steps = int(segment.norm() / stepsize);

			bool bAddEdge = true;
			do
			{
				if (!cell.CheckPosition(currentWorm.q()))
				{
					bAddEdge = false;
					break;
				}
				else
				{
					currentWorm.q() += delta;
				}
			} while (--steps > 0);

			if (bAddEdge == true)
			{
				boost::add_edge(nNodes, worm.second, g);
				break;
			}
		}
		result.clear();

    // 4. Step: connecting goal configuration to graph
    cout << "4. Step: connecting goal configuration to graph" << endl;
	//rtree.insert(make_pair(MyWorm(qGoal), nNodes + 1));
	result.clear();
	g[nNodes + 1].q_ = qGoal;
	currentWorm = MyWorm(qGoal);
	rtree.query(boost::geometry::index::nearest(currentWorm, 5), std::back_inserter(result));
	for each (std::pair<MyWorm, int> worm in result)
	{
		// test connection
		currentWorm = MyWorm(qGoal);
		Eigen::VectorXd segment(worm.first.q() - currentWorm.q()), delta(5);
		delta = segment.normalized() * stepsize;
		int steps = int(segment.norm() / stepsize);

		bool bAddEdge = true;
		do
		{
			if (!cell.CheckPosition(currentWorm.q()))
			{
				bAddEdge = false;
				break;
			}
			else
			{
				currentWorm.q() += delta;
			}
		} while (--steps > 0);

		if (bAddEdge == true) 			
		{
			boost::add_edge(nNodes + 1, worm.second, g);
			break;
		}
	}
	result.clear();
	
    // 5. Step: searching for shortest path
    cout << "5. Step: searching for shortest path" << endl;

	typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_descriptor;
	vertex_descriptor start = boost::vertex(nNodes, g);
	std::vector<vertex_descriptor> p(num_vertices(g));
	boost::dijkstra_shortest_paths(g, start, boost::predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))));

	// Write the path: reverse search through predecessor map
	int currentVertex = nNodes + 1; // end point
	while ((currentVertex != nNodes) && (path.size() < 1000)) // start point
	{
		path.push_back(g[currentVertex].q_);
		currentVertex = p[currentVertex];
	}
	path.push_back(g[currentVertex].q_);

	write_easyrob_program_file(path, "prm.prg");

	write_gnuplot_file(g, "VisibilityGraph.dat");

    return EXIT_SUCCESS;
}
