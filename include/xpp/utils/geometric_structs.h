/*!
 * \file   geom_to_sl.h
 * \author Alexander Winkler
 * \date   Jul 4, 2014
 * \brief  Structures to hold the pose (position + orientation) of an object
 */

#ifndef _XPP_UTILS_GEOMETRICSTRUCTS_H_
#define _XPP_UTILS_GEOMETRICSTRUCTS_H_

#include "orientation.h" /// Orientations::QuaternionToRPY()
#include <Eigen/Dense>
#include <array>

namespace xpp {

/**
@brief Utilities of dynamic locomotion library

Helpful geometric representations, conversion functions, logging facilities are
located in this namespace
 */
namespace utils {

// for easily import X,Y,Z,AX,... into other namespaces
namespace coords_wrapper {
/// To be used with 6D vectors. 'A' stands for angular, 'L' for linear.
enum Coords3D { X=0, Y, Z};
enum Coords6D { AX=0, AY, AZ, LX, LY, LZ };
static const Coords3D Coords3DArray[] = { X, Y, Z };
}
using namespace coords_wrapper;

typedef Eigen::Vector2d Vec2d; /// X,Y
typedef Eigen::Vector3d Vec3d; /// X,Y,Z


struct Point2d {
  Eigen::Vector2d p;
  Eigen::Vector2d v;
  Eigen::Vector2d a;
  explicit Point2d(Eigen::Vector2d _p = Eigen::Vector2d::Zero(),
                   Eigen::Vector2d _v = Eigen::Vector2d::Zero(),
                   Eigen::Vector2d _a = Eigen::Vector2d::Zero())
      : p(_p), v(_v), a(_a) {}
};


struct Point3d {
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  explicit Point3d(Eigen::Vector3d _p = Eigen::Vector3d::Zero(),
                   Eigen::Vector3d _v = Eigen::Vector3d::Zero(),
                   Eigen::Vector3d _a = Eigen::Vector3d::Zero())
      : p(_p), v(_v), a(_a) {}
};


struct Ori {
  Eigen::Quaterniond q;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  explicit Ori(Eigen::Quaterniond _q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
               Eigen::Vector3d _v    = Eigen::Vector3d::Zero(),
               Eigen::Vector3d _a    = Eigen::Vector3d::Zero())
  : q(_q), v(_v), a(_a) {}

};


struct Pose {
  Point3d pos;
  Ori ori;
};


/** p*x + q*y + r = 0 */
struct LineCoeff2d {
  double p;
  double q;
  double r;
};




// overloading operator<< for more elegant priting of above values
inline std::ostream& operator<<(std::ostream& out, const LineCoeff2d& lc)
{
  out  << "p=" << lc.p << ", q=" << lc.q << ", r=" << lc.r;
  return out;
}


inline std::ostream& operator<<(std::ostream& out, const Point2d& pos)
{
  out << "p=" << pos.p.transpose() << "  "
      << "v=" << pos.v.transpose() << "  "
      << "a=" << pos.a.transpose();
  return out;
}


inline std::ostream& operator<<(std::ostream& out, const Point3d& pos)
{
  out << "p=" << pos.p.transpose() << "  "
      << "v=" << pos.v.transpose() << "  "
      << "a=" << pos.a.transpose();
  return out;
}


inline std::ostream& operator<<(std::ostream& out, const Ori& ori)
{
  Eigen::Vector3d rpy_rad, rpy_deg;
  Orientation::QuaternionToRPY(ori.q, rpy_rad);
  rpy_deg = rpy_rad * (180.0 / 3.14);
  out << "rpy=" << rpy_deg.transpose() << "  "
      << "v=" << ori.v.transpose() << "  "
      << "a=" << ori.a.transpose();
  return out;
}


inline std::ostream& operator<<(std::ostream& out, const Pose& pose)
{
  out << "\tPos: " << pose.pos << "\n"
      << "\tOri: " << pose.ori;
  return out;
}

inline Point2d operator+(const Point2d& lhs, const Point2d& rhs)
{
  Point2d ret;
  ret.p = lhs.p + rhs.p;
  ret.v = lhs.v + rhs.v;
  ret.a = lhs.a + rhs.a;
  return ret;
}

inline Point2d operator*(double mult, const Point2d& rhs)
{
  Point2d ret;
  ret.p = mult * rhs.p;
  ret.v = mult * rhs.v;
  ret.a = mult * rhs.a;
  return ret;
}

} // namespace utils
} // namespace xpp

#endif // _XPP_UTILS_GEOMETRICSTRUCTS_H_
