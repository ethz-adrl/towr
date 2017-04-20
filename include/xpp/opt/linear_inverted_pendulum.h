/**
 @file    linear_inverted_pendulum.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_LINEAR_INVERTED_PENDULUM_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_LINEAR_INVERTED_PENDULUM_H_

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/cartesian_declarations.h>
#include <xpp/endeffectors.h>

namespace xpp {
namespace opt {

class BaseMotion;

class LinearInvertedPendulum {
public:
  using JacobianRow = Eigen::SparseVector<double, Eigen::RowMajor>;

  using ComPos = Eigen::Vector2d;
  using ComAcc = Eigen::Vector2d;
  using Cop    = Eigen::Vector2d;

  using EELoad = Endeffectors<double>;
  using EEPos  = EndeffectorsPos;

  LinearInvertedPendulum ();
  virtual ~LinearInvertedPendulum ();

  void SetCurrent(const ComPos&, double height, const EELoad&, const EEPos&);

  ComAcc GetAcceleration() const;

  /** Approximates the acceleration with small angle assumption and calculates
    * Jacobian w.r.t. spline coefficients.
    */
  JacobianRow GetJacobianWrtBase(const BaseMotion&, double t_global, Coords3D dim) const;

  double GetDerivativeOfAccWrtLoad(EndeffectorID, d2::Coords dim) const;
  double GetDerivativeOfAccWrtEEPos(EndeffectorID) const; // same for x and y direction

private:
  ComPos pos_;
  double h_;
  EELoad ee_load_;
  EEPos ee_pos_;

  Cop CalculateCop() const;
  Cop GetDerivativeOfCopWrtLoad(EndeffectorID) const;
  double GetDerivativeOfCopWrtEEPos(EndeffectorID) const;
  double GetLoadSum() const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_LINEAR_INVERTED_PENDULUM_H_ */
