#ifndef CELL_H_INCLUDED
#define CELL_H_INCLUDED

#include <string>
#include <random>
#include <memory>
#include <chrono>

#include <Eigen/Eigen>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/math/transform.h>
#include <fcl/narrowphase/narrowphase.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/connected_components.hpp>

// includes used for nearest neighbour retrieval
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/index/rtree.hpp>

#include "MultiRobot2x2.h"
#include "Robot2.h"

#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.0)

typedef std::vector<std::shared_ptr<fcl::Box>> boxv_t;
typedef std::vector<fcl::Transform3f> t3fv_t;

class MyCell
{
public:
    void operator()(boxv_t &obj_obstacle, t3fv_t &tf_obstacle);
};

// point coordinates of vertex
typedef struct { Eigen::VectorXd q_; } vertex_prop_t;

typedef boost::property<boost::edge_weight_t, float> edge_weight_prop_t;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, vertex_prop_t, edge_weight_prop_t> graph_t;

// Some typedefs for simplicity
typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_t;
typedef boost::graph_traits<graph_t>::edge_descriptor edge_t;

typedef boost::property_map<graph_t, boost::vertex_index_t>::type index_map_t;
typedef boost::graph_traits<graph_t>::vertex_iterator vertex_iter;
typedef boost::graph_traits<graph_t>::edge_iterator edge_iter;

void write_nodes_file(graph_t, std::string, bool jump_to = true);
void write_easyrob_program_file(std::vector<Eigen::VectorXd>, std::string, bool jump_to = false);
void write_gnuplot_file(graph_t, std::string);

template<typename _ROBOT_TYPE, typename _CELL_TYPE>
class Cell
{
public:
    Cell();
	Cell(std::vector<std::vector<Eigen::VectorXd>> inCoordinationDiagramPaths);
    virtual ~Cell() { }

    bool JumpTo(const Eigen::VectorXd &q);
    bool CheckPosition(const Eigen::VectorXd &q);
    bool CheckMotion(const Eigen::VectorXd &from, const Eigen::VectorXd &to, float dx = 1e-2);
    bool FirstContact(Eigen::VectorXd &Cfree, Eigen::VectorXd &Cobstacle, Eigen::VectorXd &from, const Eigen::VectorXd &to, float dx = 1e-2);
    bool LastContact(Eigen::VectorXd &Cfree, Eigen::VectorXd &Cobstacle, Eigen::VectorXd &from, const Eigen::VectorXd &to, float dx = 1e-2);

    Eigen::VectorXd NextRandomCspace();
    Eigen::VectorXd NextRandomCfree();
    void ResetRNG();

	std::vector<Eigen::VectorXd>  convertToRealPosition(Eigen::VectorXd q);

    _ROBOT_TYPE& Robot() { return robot_; }

protected:
    Cell(const Cell&);
    const Cell& operator=(const Cell&);

    std::vector<std::shared_ptr<fcl::Box>> obj_obstacle_;
    std::vector<std::shared_ptr<fcl::Box>> obj_robot_;
    std::vector<fcl::Transform3f> tf_obstacle_;
    std::vector<fcl::Transform3f> tf_robot_;
    fcl::GJKSolver_libccd solver_;

	// Coordination Diagram
	std::vector<std::vector<Eigen::VectorXd>> cdPaths;
	std::vector<std::vector<double>> cdPathLengths;

    std::mt19937_64 rng_; // Random Number Generator
    std::uniform_real_distribution<double> unif_;
    _ROBOT_TYPE robot_;
};

#include "cell.inl"

typedef Cell<MultiRobot2x2, MyCell> MultiRobotCell;

#endif
