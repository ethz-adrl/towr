/*!
 * \file   state.h
 * \author Alexander Winkler
 * \date   Jul 4, 2014
 * \brief  Structures to hold the pose (position + orientation) of an object
 */

#ifndef _XPP_UTILS_STATE_H_
#define _XPP_UTILS_STATE_H_

#include <iostream>
#include <Eigen/Dense>

#include <xpp/cartesian_declarations.h>

namespace xpp {

class StateLin1d;
class StateLin2d;
class StateLin3d;
class StateAng3d;
class State3d;

using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double,6,1>;
using VectorXd = Eigen::VectorXd;

/** @brief Interface to an object holding  x-dimensional values.
 *
 * These values include position, velocity and acceration.
 */
class StateLinXd {
public:
  VectorXd p_, v_, a_;
  StateLinXd();
  explicit StateLinXd(int dim);
  explicit StateLinXd(const VectorXd& p, const VectorXd& v, const VectorXd& a);
  StateLinXd(const VectorXd& p);
  virtual ~StateLinXd() {};

  const VectorXd GetByIndex(MotionDerivative deriv) const;
  VectorXd& GetByIndex(MotionDerivative deriv);

  StateLin1d GetDimension(int dim) const;
  void SetDimension(int dim, const StateLin1d& p1d);

  bool operator==(const StateLinXd& other) const;
  bool operator!=(const StateLinXd& other) const;

  int kNumDim = 0;
};


class StateLin1d  : public StateLinXd {
public:
  StateLin1d() : StateLinXd(1) {};
  StateLin1d(const StateLinXd&);
  virtual ~StateLin1d() {};
};

class StateLin2d  : public StateLinXd {
public:
  StateLin2d() : StateLinXd(2) {};
  virtual ~StateLin2d() {};
};

class StateLin3d : public StateLinXd {
public:
  StateLin3d() : StateLinXd(3) {};
//  StateLin3d(const VectorXd p) : StateLin3d() { p_ = p; };
  StateLin3d(const StateLinXd&);
  virtual ~StateLin3d() {};

  StateLin2d Get2D() const;
};


/** Angular state of an object in 3-dimensional space.
  */
class StateAng3d {
public:
  using Vector3d   = Eigen::Vector3d;
  using Quaternion = Eigen::Quaterniond;

  Quaternion q;  ///< mapping from base -> world
  Vector3d v, a; ///< angular vel/acc expressed in world frame

  explicit StateAng3d(Quaternion _q = Quaternion(1.0, 0.0, 0.0, 0.0),
                      Vector3d   _v = Vector3d::Zero(),
                      Vector3d   _a = Vector3d::Zero())
  : q(_q), v(_v), a(_a) {}
};

/** The complete state (linear+angular) of an object in 3-dimensional space
  */
class State3d {
public:
  StateLin3d lin; ///< linear position, velocity and acceleration
  StateAng3d ang; ///< angular position, velocity and acceleration

  Vector6d Get6dVel() const;
  Vector6d Get6dAcc() const;
};

/** The complete state (linear+angular) of an object in 3-dimensional space
  */
class State3dEuler {
public:
  StateLin3d lin; ///< linear position, velocity and acceleration
  StateLin3d ang; ///< rpy euler angles, euler rates, euler accelerations
};

inline std::ostream& operator<<(std::ostream& out, const StateLinXd& pos)
{
  out << "p=" << pos.p_.transpose() << "  "
      << "v=" << pos.v_.transpose() << "  "
      << "a=" << pos.a_.transpose();
  return out;
}

inline StateLinXd operator+(const StateLinXd& lhs, const StateLinXd& rhs)
{
  StateLinXd ret(lhs.kNumDim);
  ret.p_ = lhs.p_ + rhs.p_;
  ret.v_ = lhs.v_ + rhs.v_;
  ret.a_ = lhs.a_ + rhs.a_;
  return ret;
}

inline StateLinXd operator*(double mult, const StateLinXd& rhs)
{
  StateLinXd ret(rhs.kNumDim);
  ret.p_ = mult * rhs.p_;
  ret.v_ = mult * rhs.v_;
  ret.a_ = mult * rhs.a_;
  return ret;
}

inline bool
StateLinXd::operator==(const StateLinXd &other) const
{
  bool all_equal = (p_==other.p_
                 && v_==other.v_
                 && a_==other.a_);
  return all_equal;
}

inline bool
StateLinXd::operator!=(const StateLinXd &other) const
{
  return !(*this == other);
}

inline std::ostream& operator<<(std::ostream& out, const StateAng3d& ori)
{
  Eigen::Vector3d rpy_rad, rpy_deg;
  rpy_rad = ori.q.toRotationMatrix().eulerAngles(2,1,0);
  rpy_deg = rpy_rad * (180.0 / 3.14);
  out << "rpy=" << rpy_rad.transpose() << "  "
      << "v=" << ori.v.transpose() << "  "
      << "a=" << ori.a.transpose();
  return out;
}

inline std::ostream& operator<<(std::ostream& out, const State3d& pose)
{
  out << "\tPos: " << pose.lin << "\n"
      << "\tOri: " << pose.ang;
  return out;
}

} // namespace xpp

#endif // _XPP_UTILS_STATE_H_
