/******************************************************************************
    file:      VisibilityGraph.cpp
    created:   2016-10-23
    author:    Thomas Horsch

    description: it is a brute force algorithm O(n^3), testing the visibility
    of each pair of edges
******************************************************************************/

#include <iostream>
#include <fstream>
#include "VisibilityGraph.h"
#include <boost/geometry/geometries/segment.hpp> 
#include <boost/geometry/algorithms/intersection.hpp>

#include <boost/geometry/geometries/point_xy.hpp>
typedef boost::geometry::model::d2::point_xy<double> Point2D;
typedef boost::geometry::model::segment<Point2D> Segment;

#define SOLUTION

using namespace std;

bool intersectionLineLine(Point p1, Point p2, Point p3, Point p4, Point *intersection, double *t1, double *t2)
{
	// Store the values for fast access and easy
	// equations-to-code conversion
	// intersection = NULL;
	double x1 = p1.x, x2 = p2.x, x3 = p3.x, x4 = p4.x;
	double y1 = p1.y, y2 = p2.y, y3 = p3.y, y4 = p4.y;

	double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
	// If d is zero, there is no intersection
	if (d == 0)
		return false;

	// Get the x and y
	double pre = (x1*y2 - y1*x2), post = (x3*y4 - y3*x4);
	double x = (pre * (x3 - x4) - (x1 - x2) * post) / d;
	double y = (pre * (y3 - y4) - (y1 - y2) * post) / d;

	// Check if the x and y coordinates are within both lines
	if (x < min(x1, x2) || x > max(x1, x2) || x < min(x3, x4) || x > max(x3, x4))
		return false;
	if (y < min(y1, y2) || y > max(y1, y2) || y < min(y3, y4) || y > max(y3, y4))
		return false;

	// Return the point of intersection
	intersection->x = x;
	intersection->y = y;
	intersection->z = 0.0;

	double par_t = intersection->SquareDistance(p1) / p2.SquareDistance(p1);
	*t1 = sqrt(par_t);
	par_t = intersection->SquareDistance(p3) / p4.SquareDistance(p3);
	*t2 = sqrt(par_t);
	return true;
}

vector<Point> VisibilityGraph(Graph g, const int nHind)
{
	typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
	typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor;
	typedef std::pair<int, int> Edge;
	vector<Edge> edge_vector;

	vector<Point> path; // create a point vector for storing the path

	// Example for access to the coordinates of the vertices

	for (int i = 0; i < nHind * 4; i += 4)
	{

		edge_vector.push_back(Edge(i, i + 1));
		edge_vector.push_back(Edge(i + 1, i + 2));
		edge_vector.push_back(Edge(i + 2, i + 3));
		edge_vector.push_back(Edge(i + 3, i));

		//        cout << g[i].pt.x << " " << g[i].pt.y << endl;
	}

#ifdef SOLUTION

	/*
	Point tmpPoint;
	double tmp1;
	double tmp2;

	// No Intersection !!!
	if (intersectionLineLine(Point(0.3f, 0.1f, 0.0f), Point(0.4f, 0.6f, 0.0f), Point(0.2f, 0.4f, 0.0f), Point(0.4f, 0.4f, 0.0f), &tmpPoint, &tmp1, &tmp2))
		int sfd = 0;

	// Intersection !!!
	Segment AB(Point2D(0.3f, 0.1f), Point2D(0.4f, 0.6f));
	Segment CD(Point2D(0.2f, 0.4f), Point2D(0.4f, 0.4f));
	bool result = boost::geometry::intersects(AB, CD);
	*/

	for (int i = 0; i < nHind * 4 + 2; ++i){

		for (int j = 0; j < nHind * 4 + 2; ++j){
			if (i == j) continue;
			if (i / 4 == j / 4) continue;

			bool status = false;
			for (int k = 0; k < nHind * 4; k += 4)
			{
				if (i != k && j != k && i != k + 1 && j != k + 1)
					status |= boost::geometry::intersects(Segment(Point2D(g[i].pt.x, g[i].pt.y), Point2D(g[j].pt.x, g[j].pt.y)), Segment(Point2D(g[k].pt.x, g[k].pt.y), Point2D(g[k + 1].pt.x, g[k + 1].pt.y)));
				if (i != k + 1 && j != k + 1 && i != k + 2 && j != k + 2)
					status |= boost::geometry::intersects(Segment(Point2D(g[i].pt.x, g[i].pt.y), Point2D(g[j].pt.x, g[j].pt.y)), Segment(Point2D(g[k + 1].pt.x, g[k + 1].pt.y), Point2D(g[k + 2].pt.x, g[k + 2].pt.y)));
				if (i != k + 2 && j != k + 2 && i != k + 3 && j != k + 3)
					status |= boost::geometry::intersects(Segment(Point2D(g[i].pt.x, g[i].pt.y), Point2D(g[j].pt.x, g[j].pt.y)), Segment(Point2D(g[k + 2].pt.x, g[k + 2].pt.y), Point2D(g[k + 3].pt.x, g[k + 3].pt.y)));
				if (i != k + 3 && j != k + 3 && i != k && j != k)
					status |= boost::geometry::intersects(Segment(Point2D(g[i].pt.x, g[i].pt.y), Point2D(g[j].pt.x, g[j].pt.y)), Segment(Point2D(g[k + 3].pt.x, g[k + 3].pt.y), Point2D(g[k].pt.x, g[k].pt.y)));

			}
			if (!status)
				edge_vector.push_back(Edge(i, j));
		}
	}


	const int num_edges = edge_vector.size();

	// add the edges to the graph object
	for (int i = 0; i < num_edges; ++i)
		//dirty hack because of Edge(1, 6)! somehow the linelineinteresction doen't find an intersection
		//if (i != 14)
			add_edge(edge_vector[i].first, edge_vector[i].second, g);

#endif SOLUTION

	write_gnuplot_file(g, "VisibilityGraph.dat");

	return path;
}

/**************************************************************************/
// Ausgabe einer Plotdatei für gnuplot:
// Aufruf in gnuplot: plot 'visibilitygraph.data' using 1:2 with lines
void write_gnuplot_file(Graph g, string filename)
{
    ofstream myfile;
    myfile.open(filename);

    // Iterate through the edges and print them out
    typedef boost::graph_traits<Graph>::edge_iterator edge_iter;
    std::pair<edge_iter, edge_iter> ep;
    edge_iter ei, ei_end;

    int cnt = 0; // edge counter

    for (tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
    {
        myfile << g[ei->m_source].pt.x << " " << g[ei->m_source].pt.y << endl;
        myfile << g[ei->m_target].pt.x << " " << g[ei->m_target].pt.y << endl << endl;
        cnt++;
    }

    cout << "Number of edges: " << cnt <<  endl;
    myfile.close();
}
