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

double GetClosestPoint(Eigen::VectorXd A, Eigen::VectorXd B, Eigen::VectorXd P)
{
    Eigen::VectorXd AP = P - A;
    Eigen::VectorXd AB = B - A;
    double ab2 = AB[0] * AB[0] + AB[1] * AB[1];
    double ap_ab = AP[0] * AB[0] + AP[1] * AB[1];
    double t = ap_ab / ab2;
    Eigen::VectorXd Closest = A + AB * t;
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
    const float stepsize = .025f;

#define TEST_CASE 0
#ifdef TEST_CASE
#if TEST_CASE == 0
	
    // Example
    cout << "Example" << endl;
	qStart << 0.5, 0.5, 0., 0., 0.;
    qGoal << .6, .9, DEG2RAD(-90.), DEG2RAD(-180.), DEG2RAD(180.);
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

    const int nNodes = 100;
    int additionalNodes = 0;
    // 1. step: building up a graph g consisting of nNodes vertices
    cout << "1. Step: building " << nNodes << " nodes for the graph" << endl;

    std::vector<std::pair<MyWorm, int>> result;
    rtree.insert(make_pair(MyWorm(qStart), 0));
    boost::add_vertex(g);
    g[0].q_ = qStart;

    for (int i = 1; i< nNodes; ++i) {
		Eigen::VectorXd sample = cell.NextRandomCspace();
		sample[2] = 0.0;
		sample[3] = 0.0;
		sample[4] = 0.0;

		++additionalNodes;
		int currentSampleIndex = additionalNodes;
        boost::add_vertex(g);
		g[currentSampleIndex].q_ = sample;
		rtree.query(boost::geometry::index::nearest(MyWorm(g[currentSampleIndex].q_), 1), std::back_inserter(result));
		rtree.insert(make_pair(MyWorm(sample), currentSampleIndex));

       
        boost::graph_traits<graph_t>::vertex_descriptor tmp_v = boost::vertex(result.back().second,g);

        double min_dist = std::numeric_limits<double>::infinity();
        Eigen::VectorXd nearest_A;
        Eigen::VectorXd nearest_B;
        int index_source = -1;
        int index_target = -1;

        boost::graph_traits < graph_t >::out_edge_iterator ei, ei_end;
        for (boost::tie(ei, ei_end) = out_edges(tmp_v, g); ei != ei_end; ++ei) {
            auto source = boost::source(*ei, g);
            auto target = boost::target(*ei, g);
            Eigen::VectorXd A = g[source].q_;
            Eigen::VectorXd B = g[target].q_;

            point_type p(sample[0], sample[1]);
            point_type A_tmp(A[0], A[1]);
            point_type B_tmp(B[0], B[1]);
            segment_type s(A_tmp, B_tmp);
            double dist = boost::geometry::distance(p, s);

            if (dist < min_dist){
                min_dist = dist;
                nearest_A = A;
                nearest_B = B;
                index_source = source;
                index_target = target;
            }
        }

		if (index_source == -1)
		{
			boost::add_edge(currentSampleIndex, result.back().second, g);
		}
		else
		{
			double t = GetClosestPoint(nearest_A, nearest_B, sample);
			if (t < 0.0f){
				boost::add_edge(currentSampleIndex, index_source, g);
			}
			else {
				Eigen::VectorXd projected_point = nearest_A + (nearest_B - nearest_A) * t;

				++additionalNodes;
				int splitNodeIndex = additionalNodes;
				boost::add_vertex(g);
				rtree.insert(make_pair(MyWorm(projected_point), splitNodeIndex));
				g[splitNodeIndex].q_ = projected_point;

				boost::remove_edge(index_source, index_target, g);
				boost::add_edge(splitNodeIndex, index_source, g);
				boost::add_edge(splitNodeIndex, index_target, g);
				boost::add_edge(splitNodeIndex, currentSampleIndex, g);
			}
		}

        result.clear();
    }
	std::cout << "Number of Nodes: " << additionalNodes + 1 << std::endl;
	write_gnuplot_file(g, "VisibilityGraph.dat");

    return EXIT_SUCCESS;
}
