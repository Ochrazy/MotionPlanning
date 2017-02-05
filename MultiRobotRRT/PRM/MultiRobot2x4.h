#pragma once
#include <random>
#include <Eigen/Eigen>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/math/transform.h>
#include <boost/geometry.hpp>

class MultiRobot2x4
{
public:
	MultiRobot2x4() { q_ = Eigen::VectorXd(8); q_ << 0., 0., 0., 0., 0., 0., 0., 0.; }
	MultiRobot2x4(const Eigen::VectorXd &q) : q_(q) { }

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
			template<> struct tag<MultiRobot2x4> { typedef point_tag type; };
			template<> struct coordinate_type<MultiRobot2x4> { typedef double type; };
			template<> struct coordinate_system<MultiRobot2x4> { typedef cs::cartesian type; };
			template<> struct dimension<MultiRobot2x4> : boost::mpl::int_<8> {};
			template<> struct access<MultiRobot2x4, 0>
			{
				static double get(MultiRobot2x4 const& p) { return p[0]; }
				static void set(MultiRobot2x4& p, double const& value) { p[0] = value; }
			};
			template<> struct access<MultiRobot2x4, 1>
			{
				static double get(MultiRobot2x4 const& p) { return p[1]; }
				static void set(MultiRobot2x4& p, double const& value) { p[1] = value; }
			};
			template<> struct access<MultiRobot2x4, 2>
			{
				static double get(MultiRobot2x4 const& p) { return p[2]; }
				static void set(MultiRobot2x4& p, double const& value) { p[2] = value; }
			};
			template<> struct access<MultiRobot2x4, 3>
			{
				static double get(MultiRobot2x4 const& p) { return p[3]; }
				static void set(MultiRobot2x4& p, double const& value) { p[3] = value; }
			};
			template<> struct access<MultiRobot2x4, 4>
			{
				static double get(MultiRobot2x4 const& p) { return p[4]; }
				static void set(MultiRobot2x4& p, double const& value) { p[4] = value; }
			};
			template<> struct access<MultiRobot2x4, 5>
			{
				static double get(MultiRobot2x4 const& p) { return p[5]; }
				static void set(MultiRobot2x4& p, double const& value) { p[5] = value; }
			};
			template<> struct access<MultiRobot2x4, 6>
			{
				static double get(MultiRobot2x4 const& p) { return p[6]; }
				static void set(MultiRobot2x4& p, double const& value) { p[6] = value; }
			};
			template<> struct access<MultiRobot2x4, 7>
			{
				static double get(MultiRobot2x4 const& p) { return p[7]; }
				static void set(MultiRobot2x4& p, double const& value) { p[7] = value; }
			};
		}
	}
} // namespace boost::geometry::traits


