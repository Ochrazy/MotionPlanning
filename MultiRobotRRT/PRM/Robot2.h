#pragma once
#include <random>
#include <Eigen/Eigen>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/math/transform.h>
#include <boost/geometry.hpp>

class Robot2
{
public:
	Robot2() { q_ = Eigen::VectorXd(2); q_ << 0., 0.; }
	Robot2(const Eigen::VectorXd &q) : q_(q) { }

	std::vector<fcl::Transform3f> ForwardKinematic() const;

	double &operator[](int i) { return q_(i); }
	double operator[](int i) const { return q_(i); }

	Eigen::VectorXd& q() { return q_; }

	static std::vector<fcl::Transform3f> ForwardKinematic(const Eigen::VectorXd &q);
	static Eigen::VectorXd Random(std::mt19937_64 &rng, std::uniform_real_distribution<double> &unif);

	void operator()(std::vector<std::shared_ptr<fcl::Box>> &obj_robot);

protected:
	Eigen::VectorXd q_;
};


namespace boost {
	namespace geometry {
		namespace traits
		{
			// Adapt MultiRobot to Boost.Geometry
			template<> struct tag<Robot2> { typedef point_tag type; };
			template<> struct coordinate_type<Robot2> { typedef double type; };
			template<> struct coordinate_system<Robot2> { typedef cs::cartesian type; };
			template<> struct dimension<Robot2> : boost::mpl::int_<2> {};
			template<> struct access<Robot2, 0>
			{
				static double get(Robot2 const& p) { return p[0]; }
				static void set(Robot2& p, double const& value) { p[0] = value; }
			};
			template<> struct access<Robot2, 1>
			{
				static double get(Robot2 const& p) { return p[1]; }
				static void set(Robot2& p, double const& value) { p[1] = value; }
			};
		}
	}
} // namespace boost::geometry::traits


