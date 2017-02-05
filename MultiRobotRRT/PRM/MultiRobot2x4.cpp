#include "stdafx.h"
#include "MultiRobot2x4.h"

std::vector<fcl::Transform3f> MultiRobot2x4::ForwardKinematic() const
{
	return ForwardKinematic(q_);
}

std::vector<fcl::Transform3f> MultiRobot2x4::ForwardKinematic(const Eigen::VectorXd &q)
{
	std::vector<fcl::Transform3f> transforms(4);
	fcl::Transform3f T;

	T = fcl::Transform3f(fcl::Vec3f(q(0), q(1), 0.f));
	transforms[0] = T;

	T = fcl::Transform3f(fcl::Vec3f(q(2), q(3), 0.f));
	transforms[1] = T;

	T = fcl::Transform3f(fcl::Vec3f(q(4), q(5), 0.f));
	transforms[2] = T;

	T = fcl::Transform3f(fcl::Vec3f(q(6), q(7), 0.f));
	transforms[3] = T;

	return transforms;
}

Eigen::VectorXd MultiRobot2x4::Random(std::mt19937_64 &rng, std::uniform_real_distribution<double>& unif)
{
	Eigen::VectorXd q(8);

	q(0) = unif(rng);
	q(1) = unif(rng);
	q(2) = unif(rng);
	q(3) = unif(rng);
	q(4) = unif(rng);
	q(5) = unif(rng);
	q(6) = unif(rng);
	q(7) = unif(rng);

	return q;
}

void MultiRobot2x4::operator()(std::vector<std::shared_ptr<fcl::Box>> &obj_robot)
{
	obj_robot.push_back(std::make_shared<fcl::Box>(0.1, 0.05, 0.05));
	obj_robot.push_back(std::make_shared<fcl::Box>(0.1, 0.05, 0.05));
	obj_robot.push_back(std::make_shared<fcl::Box>(0.1, 0.05, 0.05));
	obj_robot.push_back(std::make_shared<fcl::Box>(0.1, 0.05, 0.05));
}

