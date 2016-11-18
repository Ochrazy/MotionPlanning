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

	for (int i = 0; i < nHind * 4 + 2; ++i){

		for (int j = i + 1; j < nHind * 4 + 2; ++j){
			if (i == j) continue;
			if (i / 4 == j / 4) continue;

			// Make edge longer in both directions for supporting/separating line
			Point direction = (g[i].pt - g[j].pt).Normalize();
			Point2D A = Point2D(g[i].pt.x + direction.x * 2, g[i].pt.y + direction.y * 2);
			Point2D B = Point2D(g[j].pt.x - direction.x * 2, g[j].pt.y - direction.y * 2);

			bool status = false;
			int numberOfIntersections = 0; // Counter for Intersection test for supporting/separating line
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

				if (status)
					break;

				// Reduce the Graph with supporting/separating line 'algorithm'
				if (i < nHind * 4 && j < nHind * 4  &&	// check if i and j belong to an obstacle (not start or end point)
					i / 4 != j / 4 &&					// check if i-th and j-th index are belonging to different obstacles
					(i / 4 == k / 4 || j / 4 == k / 4)) // check if current index k of an obstacle accords to the i-th or j-th vertex
														// because then the edge is to be checked for intersections between these two obstacles 
				{
					numberOfIntersections += boost::geometry::intersects(Segment(A, B),
						Segment(Point2D(g[k].pt.x, g[k].pt.y), Point2D(g[k + 1].pt.x, g[k + 1].pt.y))) ? 1 : 0;
					numberOfIntersections += boost::geometry::intersects(Segment(A, B),
						Segment(Point2D(g[k + 1].pt.x, g[k + 1].pt.y), Point2D(g[k + 2].pt.x, g[k + 2].pt.y))) ? 1 : 0;
					numberOfIntersections += boost::geometry::intersects(Segment(A, B),
						Segment(Point2D(g[k + 2].pt.x, g[k + 2].pt.y), Point2D(g[k + 3].pt.x, g[k + 3].pt.y))) ? 1 : 0;
					numberOfIntersections += boost::geometry::intersects(Segment(A, B),
						Segment(Point2D(g[k + 3].pt.x, g[k + 3].pt.y), Point2D(g[k].pt.x, g[k].pt.y))) ? 1 : 0;				
				}
			}

			// A supporting/separating line only has 2 Intersections (in this case 4, because every vertex has two edges)
			if (numberOfIntersections > 4) status = true;

			// Add the surviving edges
			if (!status)
				edge_vector.push_back(Edge(i, j));
		}
	}

	const int num_edges = edge_vector.size();

	// Add the edges to the graph object
	for (int i = 0; i < num_edges; ++i)
		add_edge(edge_vector[i].first, edge_vector[i].second, g);

	// Apply Dijkstra's shortest paths algorithm on the final graph
	vertex_descriptor start = boost::vertex(nHind * 4, g);
	std::vector<vertex_descriptor> p(num_vertices(g));
	boost::dijkstra_shortest_paths(g, start, boost::predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))));

#endif SOLUTION

	write_gnuplot_file(g, "VisibilityGraph.dat");

	// Write the path: reverse search through predecessor map
	int currentVertex = p.size() - 1; // end point
	while (currentVertex != p.size() - 2) // start point
	{
		path.push_back(g[currentVertex].pt);
		currentVertex = p[currentVertex];
	}
	path.push_back(g[currentVertex].pt);

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

	cout << "Number of edges: " << cnt << endl;
	myfile.close();
}
