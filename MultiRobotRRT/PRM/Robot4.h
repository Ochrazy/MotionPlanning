#pragma once
#include <random>
#include <Eigen/Eigen>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/math/transform.h>
#include <boost/geometry.hpp>

class Robot4
{
public:
	Robot4() { q_ = Eigen::VectorXd(4); q_ << 0., 0., 0., 0.; }
	Robot4(const Eigen::VectorXd &q) : q_(q) { }

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
			template<> struct tag<Robot4> { typedef point_tag type; };
			template<> struct coordinate_type<Robot4> { typedef double type; };
			template<> struct coordinate_system<Robot4> { typedef cs::cartesian type; };
			template<> struct dimension<Robot4> : boost::mpl::int_<4> {};
			template<> struct access<Robot4, 0>
			{
				static double get(Robot4 const& p) { return p[0]; }
				static void set(Robot4& p, double const& value) { p[0] = value; }
			};
			template<> struct access<Robot4, 1>
			{
				static double get(Robot4 const& p) { return p[1]; }
				static void set(Robot4& p, double const& value) { p[1] = value; }
			};
			template<> struct access<Robot4, 2>
			{
				static double get(Robot4 const& p) { return p[2]; }
				static void set(Robot4& p, double const& value) { p[2] = value; }
			};
			template<> struct access<Robot4, 3>
			{
				static double get(Robot4 const& p) { return p[3]; }
				static void set(Robot4& p, double const& value) { p[3] = value; }
			};
		}
	}
} // namespace boost::geometry::traits


