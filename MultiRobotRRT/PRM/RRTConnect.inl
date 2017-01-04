namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

template<typename _ROBOT_TYPE>
RRTConnect<_ROBOT_TYPE>::RRTConnect()
	: bIsCoordinationDiagram(false)
{
}

template<typename _ROBOT_TYPE>
RRTConnect<_ROBOT_TYPE>::RRTConnect(std::vector<std::vector<Eigen::VectorXd>> inCoordinationDiagramPaths)
	: cell(inCoordinationDiagramPaths), bIsCoordinationDiagram(true)
{
}

// Get Closest Point on Line (A,B) to Point P
// Projection of P onto Line (A, B) 
template<typename _ROBOT_TYPE>
double RRTConnect<_ROBOT_TYPE>::GetClosestPoint(Eigen::VectorXd A, Eigen::VectorXd B, Eigen::VectorXd P)
{
	Eigen::VectorXd AP = P - A;
	Eigen::VectorXd AB = B - A;

	double ab2 = AB.dot(AB);
	double ap_ab = AP.dot(AB);

	double t = ap_ab / ab2;
	return t;
}

// Add Edge to Graph and Rtree
template<typename _ROBOT_TYPE>
void RRTConnect<_ROBOT_TYPE>::addEdge(int firstIndex, int secondIndex, graph_t& g, MultiRobotRtree& rtree)
{
	// To graph
	boost::add_edge(firstIndex, secondIndex, g);
	// To R-Tree
	bgm::segment<_ROBOT_TYPE> edge(_ROBOT_TYPE(g[firstIndex].q_), _ROBOT_TYPE(g[secondIndex].q_));
	edge_t ed = boost::edge(firstIndex, secondIndex, g).first;
	rtree.insert(make_pair(edge, ed));
}

// Add node and return index
template<typename _ROBOT_TYPE>
int RRTConnect<_ROBOT_TYPE>::addNode(Eigen::VectorXd node, graph_t& g)
{
	int index = (int)boost::num_vertices(g);
	boost::add_vertex(g);
	g[index].q_ = node;
	return index;
}

// Split the Edge and add all the new Nodes/Edges
template<typename _ROBOT_TYPE>
int RRTConnect<_ROBOT_TYPE>::addSplitNode(Eigen::VectorXd splitNode, int source, int target, graph_t& g, MultiRobotRtree& rtree)
{
	// Add Split Node (qn)
	int splitNodeIndex = addNode(splitNode, g);

	// Remove and add edges
	boost::remove_edge(source, target, g);
	rtree.remove(std::make_pair(bgm::segment<_ROBOT_TYPE>(_ROBOT_TYPE(g[source].q_), _ROBOT_TYPE(g[target].q_)),
		boost::edge(source, target, g).first));
	addEdge(splitNodeIndex, source, g, rtree);
	addEdge(splitNodeIndex, target, g, rtree);

	return splitNodeIndex;
}

// Calculate Nearest Edge/Node to qrand
template<typename _ROBOT_TYPE>
std::pair<bgm::segment<_ROBOT_TYPE>, edge_t> RRTConnect<_ROBOT_TYPE>::calculateNearestEdge(Eigen::VectorXd q, MultiRobotRtree& rtree)
{
	// Find closest Edge to qrand
	std::vector<std::pair<bgm::segment<_ROBOT_TYPE>, edge_t>> resultEdge;
	rtree.query(boost::geometry::index::nearest(_ROBOT_TYPE(q), 1), std::back_inserter(resultEdge));
	return resultEdge.back();
}

template<typename _ROBOT_TYPE>
Eigen::VectorXd RRTConnect<_ROBOT_TYPE>::getPointOnSegment(Eigen::VectorXd a, Eigen::VectorXd b, double t)
{
	Eigen::VectorXd q;
	if (t < 0.0) q = a;
	else if (t > 1.0) q = b;
	else  q = a + (b - a) * t;

	return q;
}

template<typename _ROBOT_TYPE>
std::vector<Eigen::VectorXd> RRTConnect<_ROBOT_TYPE>::calculateShortestPath(int startIndex, int goalIndex, graph_t& g)
{
	std::vector<Eigen::VectorXd> path;
	typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_descriptor;
	vertex_descriptor start = boost::vertex(startIndex, g);
	std::vector<vertex_descriptor> p(num_vertices(g));
	boost::dijkstra_shortest_paths(g, start, boost::predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))));

	// Write the path: reverse search through predecessor map
	size_t currentVertex = goalIndex; // end point
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

template<typename _ROBOT_TYPE>
void RRTConnect<_ROBOT_TYPE>::refinePath(std::vector<Eigen::VectorXd>& path)
{
	// Path Refinement
	// Go through path
	for (unsigned int current = 0; current < path.size(); current++)
	{
		// Find last successor
		int lastSuccessor = current + 1;
		for (unsigned int successor = current + 2; successor < path.size(); successor++)
		{
			if (cell.CheckMotion(path[current], path[successor], stepsize))
				lastSuccessor = successor;
		}
		if (lastSuccessor != current + 1)
			path.erase(path.begin() + current + 1, path.begin() + lastSuccessor);
	}
}

template<typename _ROBOT_TYPE>
std::vector<Eigen::VectorXd>  RRTConnect<_ROBOT_TYPE>::convertCDPath(std::vector<Eigen::VectorXd> path)
{
	std::vector<Eigen::VectorXd> newPath;
	for (unsigned int current = 0; current < path.size(); current++)
	{
		std::vector<Eigen::VectorXd> realPath = cell.convertToRealPosition(path[current]);
		size_t sizeOfVector = realPath[0].size() + realPath.size();
		Eigen::VectorXd q(sizeOfVector);

		int index = 0;
		for (size_t i = 0; i < realPath.size(); i++)
			for (int x = 0; x < realPath[i].size(); x++)
			{
				q[index] = realPath[i][x];
				index++;
			}

		newPath.push_back(q);
	}
	return newPath;
}

template<typename _ROBOT_TYPE>
std::vector<Eigen::VectorXd> RRTConnect<_ROBOT_TYPE>::doRRTConnect(Eigen::VectorXd qStart, Eigen::VectorXd qGoal)
{
	MultiRobotRtree rtree;
	MultiRobotRtree rtreeB;

	graph_t g;
	graph_t gb;

	int solutionIndex = -1;
	int solutionIndexB = -1;

	int swapCounter = 0;

	// Maximum number of Nodes
	const int nNodes = 10000;
	std::cout << "Algorithm: Bidirectional balanced RRT" << std::endl;
	std::cout << "Number of maximum Nodes:" << nNodes << std::endl;

	// Add Start and Goal nodes
	addNode(qStart, g);
	addNode(qGoal, gb);

	for (int i = 1; i < nNodes; ++i)
	{
		// Add first edge
		if (boost::num_edges(g) == 0)
		{
			// Calculate qs
			Eigen::VectorXd qs, cObstacle, qrand = cell.NextRandomCspace();
			bool hitObs = cell.FirstContact(qs, cObstacle, g[0].q_, qrand, stepsize);
			if ((qs - g[0].q_).squaredNorm() >(0.001*0.001)) //qs != qStart)
			{
				// Add first edge and node
				addEdge(0, addNode(qs, g), g, rtree);
			}
		}
		else
		{
			// Stopping Configuration
			Eigen::VectorXd qs, cObstacle, qrand = cell.NextRandomCspace();

			// Find closest Edge to qrand
			std::pair<bgm::segment<_ROBOT_TYPE>, edge_t> edge;
			edge = calculateNearestEdge(qrand, rtree);
			// Find Closest Point
			double t = GetClosestPoint(edge.first.first.q(), edge.first.second.q(), qrand);
			Eigen::VectorXd qn = getPointOnSegment(edge.first.first.q(), edge.first.second.q(), t);

			// Calculate qs
			bool hitObs = cell.FirstContact(qs, cObstacle, qn, qrand, stepsize);

			// New sample
			if ((qs - qn).squaredNorm() > (0.001*0.001))  //qs != qn)
			{
				// Add Sample (qs) Node
				int currentSampleIndex = addNode(qs, g);

				// Add Edges
				if (t < 0.0f)
					addEdge((int)edge.second.m_source, currentSampleIndex, g, rtree);
				else if (t > 1.0f)
					addEdge((int)edge.second.m_target, currentSampleIndex, g, rtree);
				else
				{
					// Add Split Node (qn)
					int splitNodeIndex = addSplitNode(qn, (int)edge.second.m_source, (int)edge.second.m_target, g, rtree);
					addEdge(splitNodeIndex, currentSampleIndex, g, rtree);
				}

				// ----------------------------------------------------------------------------
				// Second Tree 
				// Add first edge
				if (boost::num_edges(gb) == 0)
				{
					// Calculate qs
					Eigen::VectorXd qsB, cObstacleB, qrandB = cell.NextRandomCspace();
					bool hitObs = cell.FirstContact(qsB, cObstacleB, gb[0].q_, qrandB, stepsize);
					if ((qsB - gb[0].q_).squaredNorm() > (0.001*0.001)) //qs != qStart)
					{
						// Add first edge and node
						addEdge(0, addNode(qsB, gb), gb, rtreeB);
					}
				}
				else
				{
					// Find closest Edge to qrand
					qrand = qs;
					//NearestEdgeQN neqnB = calculateQNearest(qrand, rtreeB, gb);
					// Find closest Edge to qrand
					edge = calculateNearestEdge(qrand, rtreeB);
					// Find Closest Point
					t = GetClosestPoint(edge.first.first.q(), edge.first.second.q(), qrand);
					qn = getPointOnSegment(edge.first.first.q(), edge.first.second.q(), t);

					// Calculate qsB
					Eigen::VectorXd qsB, cObstacleB;
					bool hitObs = cell.FirstContact(qsB, cObstacle, qn, qrand, stepsize);
					int qsBIndex = -1;
					// New sample
					if ((qsB - qn).squaredNorm() > (0.001*0.001))  //qs != qn)
					{
						// Add Sample (qs) Node
						qsBIndex = addNode(qsB, gb);

						// Add Edges
						if (t < 0.0f)
							addEdge((int)edge.second.m_source, qsBIndex, gb, rtreeB);
						else if (t > 1.0f)
							addEdge((int)edge.second.m_target, qsBIndex, gb, rtreeB);
						else
						{
							// Add Split Node (qn)
							int splitNodeIndex = addSplitNode(qn, (int)edge.second.m_source, (int)edge.second.m_target, gb, rtreeB);
							addEdge(splitNodeIndex, qsBIndex, gb, rtreeB);
						}
					}
					if ((qsB - qs).squaredNorm() < (0.01*0.01) && cell.CheckMotion(qsB, qs)) //qsB == qs)
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
					rtree.swap(rtreeB);
					swapCounter++;
				}
			}
		}
	}

	// Calculate shortest Paths
	std::vector<Eigen::VectorXd> path;
	if (solutionIndex != -1)
	{
		// Connect paths 
		path = calculateShortestPath(0, solutionIndex, g);
		std::cout << "Path size of G: " << path.size() << std::endl;

		if (solutionIndexB != -1)
		{
			std::vector<Eigen::VectorXd> pathGB = calculateShortestPath(0, solutionIndexB, gb);
			std::cout << "Path size of GB: " << pathGB.size() << std::endl;
			reverse(pathGB.begin(), pathGB.end());
			path.insert(path.end(), pathGB.begin(), pathGB.end());
			std::cout << "Path size of Complete Graph: " << path.size() << std::endl;
			reverse(path.begin(), path.end());
		}
		// Refine Path
		refinePath(path);
		if(bIsCoordinationDiagram)
			path = convertCDPath(path);

		std::cout << "Path size of Refined Graph: " << path.size() << std::endl;
	}
	else std::cout << "No solution found!" << std::endl;

	// Print Graphs
	std::cout << "Number of Nodes in g: " << boost::num_vertices(g) << std::endl;

	/*
	write_gnuplot_file(g, "VisibilityGraph.dat");

	if (solutionIndexB != -1)
	{
	std::cout << "Number of Nodes in gb: " << boost::num_vertices(gb) << std::endl;
	write_gnuplot_file(gb, "VisibilityGraphB.dat");
	}
	*/

	return path;
}
