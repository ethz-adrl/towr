/*
 * nlp_definitions.h
 *
 *  Created on: Apr 12, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_STRUCTURE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_STRUCTURE_H_

#include <xpp/utils/geometric_structs.h>
// should not need to include anything more here!

namespace xpp {
namespace zmp {

/** @brief Knows about the internal structure of the optimization variables.
  *
  * This class can supply all the information, that is \b immediately extractable
  * from the values of the optimization variables, without any context knowledge
  * such as initialization values etc.
  */
class NlpStructure {
public:
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef xpp::utils::coords_wrapper::Coords3D Coords;
  typedef Eigen::VectorXd VectorXd;
  typedef double Number;

  enum Dim2d { kDim2d = xpp::utils::kDim2d };

public:
  NlpStructure(int n_spline_coeff = 0, int n_steps = 0)
  {
    Init(n_spline_coeff, n_steps);
  }

  void Init(int n_spline_coeff = 0, int n_steps = 0)
  {
    n_spline_coeff_ = n_spline_coeff;
    n_steps_ = n_steps;
    x_ = Eigen::VectorXd::Zero(GetOptimizationVariableCount());
  }

  int GetOptimizationVariableCount() const { return n_spline_coeff_ + 2*n_steps_; };

  VectorXd ConvertToEigen(const Number* x) const
  {
    return Eigen::Map<const VectorXd>(x,GetOptimizationVariableCount());
  }

//  /** Spline functions */
//  VectorXd ExtractSplineCoefficients(const Number* x) const
//  {
//    return ExtractSplineCoefficients(ConvertToEigen(x));
//  }

  VectorXd ExtractSplineCoefficients() const
  {
    return x_.head(n_spline_coeff_);
  }

  void SetAllVariables(const VectorXd& x_all) {
    x_ = x_all;
  }

  void SetAllVariables(const Number* x_all) {
    x_ = ConvertToEigen(x_all);
  }

  void SetSplineCoefficients(const VectorXd& x_abdc)
  {
    x_.head(n_spline_coeff_) = x_abdc;
  }

  /** Foothold functions */
  VectorXd ConvertStdToEig(const StdVecEigen2d& footholds_xy) const
  {
    assert(footholds_xy.size() == n_steps_);

    VectorXd vec(kDim2d*n_steps_);
    int c=0;
    for (int step=0; step<n_steps_; ++step)
    {
      vec[c++] = footholds_xy.at(step).x();
      vec[c++] = footholds_xy.at(step).y();
    }
    return vec;
  }

  void SetFootholds(const StdVecEigen2d& footholds_xy)
  {
    x_.middleRows(n_spline_coeff_, n_steps_*kDim2d) = ConvertStdToEig(footholds_xy);
  }

  VectorXd ExtractFootholdsToEig() const
  {
    return x_.middleRows(n_spline_coeff_, kDim2d*n_steps_);
  }

  StdVecEigen2d ExtractFootholdsToStd() const
  {
    Eigen::VectorXd footholds_xy = ExtractFootholdsToEig();
    StdVecEigen2d fooothold_vec(n_steps_);
    for (int i=0; i<n_steps_; ++i) {
      fooothold_vec.at(i) = footholds_xy.segment<kDim2d>(2*i);
    }

    return fooothold_vec;
  }

//  StdVecEigen2d ExtractFootholdsToStd(const Number* x) const
//  {
//    return ExtractFootholdsToStd(ConvertToEigen(x));
//  }

  /** miscellaneous helper functions */
//  static int Index(int spline, Coords dim, SplineCoeff coeff)
//  {
//    int idx = 0;
//    idx += ContinuousSplineContainer::Index(spline, dim, coeff);
//    return idx;
//  }
//
//  int Index(int step, Coords dim)
//  {
//    int idx = n_spline_coeff_;
//    idx += step*kDim2d + dim;
//    return idx;
//  }

private:
  VectorXd x_;  ///< optimization variables

  int n_spline_coeff_;
  int n_steps_;
};

} // namespace zmp
} // namespace xpp

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_STRUCTURE_H_ */
