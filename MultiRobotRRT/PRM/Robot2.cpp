#include "stdafx.h"
#include "Robot2.h"

std::vector<fcl::Transform3f> Robot2::ForwardKinematic() const
{
	return ForwardKinematic(q_);
}

std::vector<fcl::Transform3f> Robot2::ForwardKinematic(const Eigen::VectorXd &q)
{
	std::vector<fcl::Transform3f> transforms(1);
	fcl::Transform3f T;

	T = fcl::Transform3f(fcl::Vec3f(q(0), q(1), 0.f));
	transforms[0] = T;

	return transforms;
}

Eigen::VectorXd Robot2::Random(std::mt19937_64 &rng, std::uniform_real_distribution<double>& unif)
{
	Eigen::VectorXd q(2);

	q(0) = unif(rng);
	q(1) = unif(rng);

	return q;
}

void Robot2::operator()(std::vector<std::shared_ptr<fcl::Box>> &obj_robot)
{
	obj_robot.push_back(std::make_shared<fcl::Box>(0.1, 0.05, 0.05));
}

