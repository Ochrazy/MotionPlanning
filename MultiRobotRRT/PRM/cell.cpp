#include "stdafx.h"
#include <iostream>
#include <fstream>
#include "cell.h"

void MyCell::operator()(boxv_t &obj_obstacle, t3fv_t &tf_obstacle)
{
	/*
    tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.45, 0.3, 0)));
    tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.75, 0.3, 0)));
    tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.3, 0.6, 0)));
    //tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.6, 0.6, 0)));
    tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.9, 0.6, 0)));
    tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.45, 0.9, 0)));
    tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.75, 0.9, 0)));
	*/

	/*tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.1 + 0.0, 0.2 + 0.53, 0)));
	tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.1, 0.27, 0)));*/
	tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.4, 0.0, 0))); 

 /*   for (auto i : tf_obstacle)
        obj_obstacle.push_back(std::make_shared<fcl::Box>(0.2, 0.4, 0.05));*/
	for (auto i : tf_obstacle)
		obj_obstacle.push_back(std::make_shared<fcl::Box>(0.1, 0.4, 0.05));
}

void write_nodes_file(graph_t g, std::string filename, bool jump_to)
{
    std::ofstream myfile;

    myfile.open(filename);
    myfile << "ProgramFile" << std::endl;

    std::cout << "Output vertices to " << filename << std::endl;

    std::pair<vertex_iter, vertex_iter> vp;
    for (vp = vertices(g); vp.first != vp.second; ++vp.first)
    {
        vertex_t v = *vp.first;
        if (jump_to)
            myfile << "JUMP_TO_AX ";
        else
            myfile << "PTP_AX ";

        myfile << g[v].q_(0) << "  " << g[v].q_(1) << "  " << RAD2DEG(g[v].q_(2)) << " " << RAD2DEG(g[v].q_(3)) << " " << RAD2DEG(g[v].q_(4)) << std::endl;
    }
    myfile << "EndProgramFile" << std::endl;
    myfile.close();
}

void write_easyrob_program_file(std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> path, std::string filename, bool jump_to)
{
    std::ofstream myfile;

    myfile.open(filename);
    myfile << "ProgramFile" << std::endl;

    for (int i = 0; i < (int)path.size(); i++)
    {
        if (jump_to)
            myfile << "JUMP_TO_AX ";
        else
            myfile << "PTP_AX ";

		myfile << path[i](0) << " ";
		for (int x = 1; x < (int)path[i].size() - 1; x++)
		{
			myfile << path[i](x) << " ";
		}
		myfile << path[i](path[i].size() - 1) << std::endl;
    }
    myfile << "EndProgramFile" << std::endl;
    myfile.close();
}

void write_gnuplot_file(graph_t g, std::string filename)
{
    int nEdge = 0;
    std::ofstream myfile;

    myfile.open(filename);

    // Iterate through all edges and print them
    std::pair<edge_iter, edge_iter> ep;
    edge_iter ei, ei_end;

    for (tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
    {
        myfile << g[ei->m_source].q_(0) << " " << g[ei->m_source].q_(1) << std::endl;
        myfile << g[ei->m_target].q_(0) << " " << g[ei->m_target].q_(1) << std::endl << std::endl;
        nEdge++;
    }
    myfile.close();

    std::cout << "Number of edges: " << nEdge << std::endl;
}
