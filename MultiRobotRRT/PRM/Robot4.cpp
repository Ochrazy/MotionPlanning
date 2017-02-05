#include "stdafx.h"
#include "Robot4.h"

std::vector<fcl::Transform3f> Robot4::ForwardKinematic() const
{
	return ForwardKinematic(q_);
}

std::vector<fcl::Transform3f> Robot4::ForwardKinematic(const Eigen::VectorXd &q)
{
	std::vector<fcl::Transform3f> transforms(1);
	fcl::Transform3f T;

	T = fcl::Transform3f(fcl::Vec3f(q(0), q(1), 0.0));
	transforms[0] = T;

	return transforms;
}

Eigen::VectorXd Robot4::Random(std::mt19937_64 &rng, std::uniform_real_distribution<double>& unif)
{
	Eigen::VectorXd q(4);

	q(0) = unif(rng);
	q(1) = unif(rng);
	q(2) = unif(rng);
	q(3) = unif(rng);

	return q;
}

void Robot4::operator()(std::vector<std::shared_ptr<fcl::Box>> &obj_robot)
{
	obj_robot.push_back(std::make_shared<fcl::Box>(0.1, 0.05, 0.05));
}

