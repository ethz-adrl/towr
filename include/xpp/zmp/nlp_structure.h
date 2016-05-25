/*
 * nlp_definitions.h
 *
 *  Created on: Apr 12, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_STRUCTURE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_STRUCTURE_H_

#include <xpp/utils/geometric_structs.h>
#include <xpp/zmp/continuous_spline_container.h>
#include <Eigen/Dense>

namespace xpp {
namespace zmp {

/**
 * Holds what the optimization variables represent and in which order
 */
class NlpStructure {
public:
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef ContinuousSplineContainer::Coords Coords;
  typedef Eigen::VectorXd VectorXd;
  typedef double Number;

  enum Dim2d { kDim2d = xpp::utils::kDim2d };

  struct NlpVariables
  {
    NlpVariables() {};

    /** Initializes all variables to zero */
    NlpVariables(const NlpStructure& structure)
        :spline_coeff_(VectorXd::Zero(structure.n_spline_coeff_)),
         footholds_(StdVecEigen2d(structure.n_steps_, Eigen::Vector2d::Zero())) {}

    NlpVariables(const VectorXd& spline_coeff, const StdVecEigen2d& footholds)
        :spline_coeff_(spline_coeff), footholds_(footholds) {}

    VectorXd spline_coeff_;
    StdVecEigen2d footholds_;
  };

public:
  NlpStructure(int n_spline_coeff = 0, int n_steps = 0)
    :n_spline_coeff_(n_spline_coeff),
     n_steps_(n_steps)
  {}

  int GetOptimizationVariableCount() const { return n_spline_coeff_ + 2*n_steps_; };

  VectorXd ConvertToEigen(const Number* x) const
  {
    return Eigen::Map<const VectorXd>(x,GetOptimizationVariableCount());
  }

  VectorXd ExtractSplineCoefficients(const Number* x) const
  {
    return ExtractSplineCoefficients(ConvertToEigen(x));
  }

  VectorXd ExtractSplineCoefficients(const VectorXd& x_eig) const
  {
    return x_eig.head(n_spline_coeff_);
  }

  StdVecEigen2d ExtractFootholds(const Number* x) const
  {
    return ExtractFootholds(ConvertToEigen(x));
  }

  VectorXd ExtractFootholds(const StdVecEigen2d& footholds_xy) const
  {
    uint n_steps = footholds_xy.size();
    VectorXd vec(kDim2d*n_steps);
    int c=0;
    for (uint step=0; step<n_steps; ++step)
    {
      vec[c++] = footholds_xy.at(step).x();
      vec[c++] = footholds_xy.at(step).y();
    }
    return vec;
  }

  StdVecEigen2d ExtractFootholds(const VectorXd& x_eig) const
  {
    Eigen::VectorXd footholds_xy = x_eig.tail(n_steps_*kDim2d);
    StdVecEigen2d fooothold_vec(n_steps_);
    for (int i=0; i<n_steps_; ++i) {
      fooothold_vec.at(i) = footholds_xy.segment<kDim2d>(2*i);
    }

    return fooothold_vec;
  }

  static int Index(int spline, Coords dim, SplineCoeff coeff)
  {
    int idx = 0;
    idx += ContinuousSplineContainer::Index(spline, dim, coeff);
    return idx;
  }

  int Index(int step, Coords dim)
  {
    int idx = n_spline_coeff_;
    idx += step*kDim2d + dim;
    return idx;
  }

  int n_spline_coeff_;
  int n_steps_;
};

} // namespace zmp
} // namespace xpp

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_STRUCTURE_H_ */
