#include <iostream>
#include <fstream>
#include "cell.h"
//
//std::vector<fcl::Transform3f> MultiRobot::ForwardKinematic() const
//{
//	return ForwardKinematic(q_);
//}
//
//std::vector<fcl::Transform3f> MultiRobot::ForwardKinematic(const Eigen::VectorMR &q)
//{
//	std::vector<fcl::Transform3f> transforms(2);
//	fcl::Transform3f T;
//	fcl::Vec3f z(0.f, 0.f, 1.f);
//
//	T = fcl::Transform3f(fcl::Vec3f(q(0), q(1), 0.f));
//	transforms[0] = T;
//
//	T = fcl::Transform3f(fcl::Vec3f(q(2), q(3), 0.f));
//	transforms[1] = T;
//
//	return transforms;
//}
//
//Eigen::VectorMR MultiRobot::Random(std::mt19937_64 &rng, std::uniform_real_distribution<double>& unif)
//{
//	Eigen::VectorMR q;
//
//	q(0) = unif(rng);
//	q(1) = unif(rng);
//	q(2) = unif(rng);
//	q(3) = unif(rng);
//
//	return q;
//}
//
//bool MultiRobot::IsInsideRange(const Eigen::VectorMR &q)
//{
//	return q(0) >= 0. && q(0) <= 1.
//		&& q(1) >= 0. && q(1) <= 1.
//		&& q(2) >= 0. && q(2) <= 1.
//		&& q(3) >= 0. && q(3) <= 1.;
//}
//
//void MultiRobot::operator()(boxv_t &obj_robot)
//{
//	obj_robot.push_back(std::make_shared<fcl::Box>(0.1, 0.05, 0.05));
//	obj_robot.push_back(std::make_shared<fcl::Box>(0.1, 0.05, 0.05));
//}
//
//
//void MyCell::operator()(boxv_t &obj_obstacle, t3fv_t &tf_obstacle)
//{
//    tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.45, 0.3, 0)));
//    tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.75, 0.3, 0)));
//    tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.3, 0.6, 0)));
//    //tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.6, 0.6, 0)));
//    tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.9, 0.6, 0)));
//    tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.45, 0.9, 0)));
//    tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.75, 0.9, 0)));
//
//    for (auto i : tf_obstacle)
//        obj_obstacle.push_back(std::make_shared<fcl::Box>(0.2, 0.2, 0.05));
//}
//
//void write_nodes_file(graph_t g, std::string filename, bool jump_to)
//{
//    std::ofstream myfile;
//
//    myfile.open(filename);
//    myfile << "ProgramFile" << std::endl;
//
//    std::cout << "Output vertices to " << filename << std::endl;
//
//    std::pair<vertex_iter, vertex_iter> vp;
//    for (vp = vertices(g); vp.first != vp.second; ++vp.first)
//    {
//        vertex_t v = *vp.first;
//        if (jump_to)
//            myfile << "JUMP_TO_AX ";
//        else
//            myfile << "PTP_AX ";
//
//        myfile << g[v].q_(0) << "  " << g[v].q_(1) << "  " << RAD2DEG(g[v].q_(2)) << " " << RAD2DEG(g[v].q_(3)) << " " << RAD2DEG(g[v].q_(4)) << std::endl;
//    }
//    myfile << "EndProgramFile" << std::endl;
//    myfile.close();
//}
//
//void write_easyrob_program_file(std::vector<Eigen::VectorXd> path, std::string filename, bool jump_to)
//{
//    std::ofstream myfile;
//
//    myfile.open(filename);
//    myfile << "ProgramFile" << std::endl;
//
//    for (int i = path.size() - 1; i >= 0; --i)
//    {
//        if (jump_to)
//            myfile << "JUMP_TO_AX ";
//        else
//            myfile << "PTP_AX ";
//
//        myfile << path[i](0) << " " << path[i](1) << " " << path[i](2) << " " << path[i](3) << std::endl;
//    }
//    myfile << "EndProgramFile" << std::endl;
//    myfile.close();
//}
//
//void write_gnuplot_file(graph_t g, std::string filename)
//{
//    int nEdge = 0;
//    std::ofstream myfile;
//
//    myfile.open(filename);
//
//    // Iterate through all edges and print them
//    std::pair<edge_iter, edge_iter> ep;
//    edge_iter ei, ei_end;
//
//    for (tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
//    {
//        myfile << g[ei->m_source].q_(0) << " " << g[ei->m_source].q_(1) << std::endl;
//        myfile << g[ei->m_target].q_(0) << " " << g[ei->m_target].q_(1) << std::endl << std::endl;
//        nEdge++;
//    }
//    myfile.close();
//
//    std::cout << "Number of edges: " << nEdge << std::endl;
//}
