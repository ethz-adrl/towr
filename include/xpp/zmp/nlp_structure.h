/*
 * nlp_definitions.h
 *
 *  Created on: Apr 12, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_STRUCTURE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_STRUCTURE_H_

#include <Eigen/Dense>

namespace xpp {
namespace zmp {

/**
 * Holds what the optimization variables represent and in which order
 */
class NlpStructure {
public:
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef Eigen::VectorXd VectorXd;
  typedef double Number;

  enum {
    X = 0,
    Y = 1
  };


public:
  NlpStructure(int n_spline_coeff = 0, int n_steps = 0)
    :n_spline_coeff_(n_spline_coeff),
     n_steps_(n_steps)
  {
    opt_all_.resize(GetOptimizationVariableCount());
    opt_all_.setZero();

    opt_footholds_.resize(n_steps);
    std::fill(opt_footholds_.begin(), opt_footholds_.end(), Eigen::Vector2d::Zero());

    opt_coeff_ = Eigen::VectorXd(n_spline_coeff_);
    opt_coeff_.setZero();
  }


  void UpdateOptimizationVariables(const Number* x)
  {
    opt_all_   = Eigen::Map<const VectorXd>(x,GetOptimizationVariableCount());
    opt_coeff_ = ExtractSplineCoefficients(opt_all_);

    for (uint i=0; i<opt_footholds_.size(); ++i) {
      Eigen::Vector2d& f = opt_footholds_.at(i);
      f.x() = x[n_spline_coeff_+2*i+X];
      f.y() = x[n_spline_coeff_+2*i+Y];
    }
  };

  Eigen::VectorXd ExtractSplineCoefficients(const Eigen::VectorXd& x) const
  {
    return x.head(n_spline_coeff_);
  }

  int n_spline_coeff_;
  int n_steps_;

  Eigen::VectorXd opt_coeff_;
  StdVecEigen2d opt_footholds_;
  Eigen::VectorXd opt_all_; /// combines the two above

  int GetOptimizationVariableCount() const { return n_spline_coeff_ + 2*n_steps_; };

};

} // namespace zmp
} // namespace xpp

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_STRUCTURE_H_ */
