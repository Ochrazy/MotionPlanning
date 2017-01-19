#include "stdafx.h"
#include <iostream>
#include "RRTConnect.h"
#include "MultiRobot2x2.h"

using namespace std;

/***********************************************************************************************************************************/
int _tmain(int argc, _TCHAR* argv[])
{
	Eigen::VectorXd qStart(4), qGoal(4);

#define TEST_CASE 0 // change Test Case
#ifdef TEST_CASE
#if TEST_CASE == 0

	qStart << 0.0, 0.25, 0.25, 0.5;
	qGoal << 0.5, 0.25, 0.25, 0.0;

	/*MultiRobotCell cell;
	bool bo = cell.CheckPosition(qStart);*/

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

	RRTConnect2x2 rrt;
	std::vector<Eigen::VectorXd> path = rrt.doRRTConnect(qStart, qGoal);
	write_easyrob_program_file(path, "rrt.prg");

	/*std::vector<std::vector<Eigen::VectorXd>> paths;
	Eigen::VectorXd qStart2(2), qGoal2(2);
	qStart2 << 1., 0.;
	qGoal2 << 0.0, .1;
	RRTConnect2 rrt;
	paths.push_back(rrt.doRRTConnect(qStart2, qGoal2));
	write_easyrob_program_file(paths[0], "rrt1.prg");

	qStart2 << 0., 0.;
	qGoal2 << 1., 0.0;
	paths.push_back(rrt.doRRTConnect(qStart2, qGoal2));

	qStart2 << 0., 0.;
	qGoal2 << 1., 1.;
	RRTConnect2 rrtCD(paths);
	std::vector<Eigen::VectorXd> path = rrtCD.doRRTConnect(qStart2, qGoal2);
	write_easyrob_program_file(path, "rrt.prg");*/

	return EXIT_SUCCESS;
}
