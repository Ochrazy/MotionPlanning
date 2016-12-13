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

typedef bg::model::point<double, 5, bg::cs::cartesian> point_5d;
typedef boost::geometry::model::segment<point_5d> segment_5d;
typedef std::pair<segment_5d, edge_t> rtree_value_edge;
typedef boost::geometry::index::rtree<rtree_value_edge, boost::geometry::index::quadratic<16>> knn_rtree_edge_t;

Eigen::VectorXd convertPointToEigen(point_5d inPoint)
{
	Eigen::VectorXd point(5);
	point << inPoint.get<0>(), inPoint.get<1>(), inPoint.get<2>(), inPoint.get<3>(), inPoint.get<4>();
	return point;
}

point_5d convertEigenToPoint(Eigen::VectorXd inPoint)
{
	point_5d point;
	point.set<0>(inPoint[0]);
	point.set<1>(inPoint[1]);
	point.set<2>(inPoint[2]);
	point.set<3>(inPoint[3]);
	point.set<4>(inPoint[4]);
	return point;
}

double GetClosestPoint(Eigen::VectorXd A, Eigen::VectorXd B, Eigen::VectorXd P)
{
	Eigen::VectorXd AP = P - A;
	Eigen::VectorXd AB = B - A;
	double ab2 = AB[0] * AB[0] + AB[1] * AB[1];
	double ap_ab = AP[0] * AB[0] + AP[1] * AB[1];
	double t = ap_ab / ab2;

	return t;
}


/***********************************************************************************************************************************/
int _tmain(int argc, _TCHAR* argv[])
{
	WormCell cell;
	Eigen::VectorXd qStart(5), qGoal(5), q(5);
	vector<Eigen::VectorXd> path; // create a point vector for storing the path
	graph_t g;
	knn_rtree_t rtree;
	knn_rtree_edge_t rtreeEdge;
	const float stepsize = .025f;

#define TEST_CASE 0
#ifdef TEST_CASE
#if TEST_CASE == 0

	// Example
	//cout << "Example" << endl;
	//qStart << 0.5, 0.5, 0., 0., 0.;
	qStart << .0, .0, 0., 0., 0.;
	// qGoal << .6, .9, DEG2RAD(-90.), DEG2RAD(-180.), DEG2RAD(180.);
	qGoal << .5, .46, DEG2RAD(-180.f), 0., 0.;
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
	// !Example
	*/
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

	const int nNodes = 250;
	int additionalNodes = 0;
	// 1. step: building up a graph g consisting of nNodes vertices
	cout << "1. Step: building " << nNodes << " nodes for the graph" << endl;

	std::vector<std::pair<MyWorm, int>> result;
	std::vector<std::pair<segment_5d, edge_t>> resultEdge;
	rtree.insert(make_pair(MyWorm(qStart), 0));
	boost::add_vertex(g);
	g[0].q_ = qStart;

	for (int i = 1; i < nNodes; ++i) {
		// First Task
		Eigen::VectorXd sample = cell.NextRandomCspace();
		sample[2] = 0.0;
		sample[3] = 0.0;
		sample[4] = 0.0;

		/*
		// Second Task
		Eigen::VectorXd sample = cell.NextRandomCspace();
		if ((i % (int)(nNodes / 5) == 0) || i == 1)
		sample = qGoal;

		// Stopping Configuration
		Eigen::VectorXd Cfree, cObstacle, qn, alpha;
		rtree.query(boost::geometry::index::nearest(MyWorm(sample), 1), std::back_inserter(result));
		qn = result.back().first.q();
		alpha = sample;
		bool hitObs = cell.FirstContact(Cfree, cObstacle, qn, alpha, stepsize);

		// Check if we can reach goal
		if (qGoal == sample && cell.CheckMotion(qn, qGoal, stepsize))
		{
		++additionalNodes;
		boost::add_vertex(g);
		g[additionalNodes].q_ = qGoal;
		boost::add_edge(additionalNodes, result.back().second, g);
		break;
		}
		// No new sample
		else if (qn == Cfree)
		{
		result.clear();
		continue;
		}
		else // New Sample
		sample = Cfree;
		*/
		++additionalNodes;
		int currentSampleIndex = additionalNodes;
		boost::add_vertex(g);
		g[currentSampleIndex].q_ = sample;
		rtree.query(boost::geometry::index::nearest(MyWorm(g[currentSampleIndex].q_), 1), std::back_inserter(result));
		rtree.insert(make_pair(MyWorm(sample), currentSampleIndex));

		if (i == 1)
		{
			boost::add_edge(0, 1, g);
			point_5d A_tmp = convertEigenToPoint(g[0].q_);
			point_5d B_tmp = convertEigenToPoint(g[1].q_);
			segment_5d edge(A_tmp, B_tmp);
			rtreeEdge.insert(make_pair(edge, boost::edge(0, 1, g).first));
			continue;
		}

		// Find closest Edge
		rtreeEdge.query(boost::geometry::index::nearest(MyWorm(g[currentSampleIndex].q_), 1), std::back_inserter(resultEdge));
		Eigen::VectorXd nearest_A = convertPointToEigen(resultEdge.back().first.first);
		Eigen::VectorXd nearest_B = convertPointToEigen(resultEdge.back().first.second);
		int index_source = resultEdge.back().second.m_source;
		int index_target = resultEdge.back().second.m_target;

		// Add Edges
		double t = GetClosestPoint(nearest_A, nearest_B, sample);
		Eigen::VectorXd projected_point = nearest_A + (nearest_B - nearest_A) * t;
		if (t < 0.0f) {// && !cell.CheckMotion(nearest_A, sample)){
			boost::add_edge(currentSampleIndex, index_source, g);
			segment_5d edge(convertEigenToPoint(g[currentSampleIndex].q_), convertEigenToPoint(g[index_source].q_));
			rtreeEdge.insert(make_pair(edge, boost::edge(currentSampleIndex, index_source, g).first));
		}
		else if (t > 1.0f){
			boost::add_edge(currentSampleIndex, index_target, g);
			segment_5d edge(convertEigenToPoint(g[currentSampleIndex].q_), convertEigenToPoint(g[index_target].q_));
			rtreeEdge.insert(make_pair(edge, boost::edge(currentSampleIndex, index_target, g).first));
		}
		else
		{
			++additionalNodes;
			int splitNodeIndex = additionalNodes;
			boost::add_vertex(g);
			rtree.insert(make_pair(MyWorm(projected_point), splitNodeIndex));
			g[splitNodeIndex].q_ = projected_point;

			boost::remove_edge(index_source, index_target, g);
			rtreeEdge.remove(resultEdge.back());

			boost::add_edge(splitNodeIndex, index_source, g);
			segment_5d edge(convertEigenToPoint(g[splitNodeIndex].q_), convertEigenToPoint(g[index_source].q_));
			rtreeEdge.insert(make_pair(edge, boost::edge(splitNodeIndex, index_source, g).first));

			boost::add_edge(splitNodeIndex, index_target, g);
			edge = segment_5d(convertEigenToPoint(g[splitNodeIndex].q_), convertEigenToPoint(g[index_target].q_));
			rtreeEdge.insert(make_pair(edge, boost::edge(splitNodeIndex, index_target, g).first));

			boost::add_edge(splitNodeIndex, currentSampleIndex, g);
			edge = segment_5d(convertEigenToPoint(g[splitNodeIndex].q_), convertEigenToPoint(g[currentSampleIndex].q_));
			rtreeEdge.insert(make_pair(edge, boost::edge(splitNodeIndex, currentSampleIndex, g).first));
		}


		result.clear();
	}
	std::cout << "Number of Nodes: " << additionalNodes + 1 << std::endl;
	write_gnuplot_file(g, "VisibilityGraph.dat");

	return EXIT_SUCCESS;
}
