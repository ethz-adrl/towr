/**
 @file    variable_set.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 8, 2016
 @brief   Holds the values of one type of optimization variables
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_NLP_VARIABLE_SET_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_NLP_VARIABLE_SET_H_

#include "a_constraint.h" // Bound
#include <Eigen/Dense>
#include <vector>

namespace xpp {
namespace opt {

/** Holds the values of one type of optmization variables
  */
class VariableSet {
public:
  using VectorXd = Eigen::VectorXd;
  using VecBound = AConstraint::VecBound;
  using Bound    = AConstraint::Bound;

  VariableSet(const VectorXd& values, std::string id, const Bound& = AConstraint::kNoBound_);
  virtual ~VariableSet();

  VectorXd GetVariables() const;
  VecBound GetBounds() const;
  std::string GetId() const;

  void SetVariables(const VectorXd& x);

private:
  VectorXd x_;
  VecBound bounds_;
  std::string id_;
};

} // namespace opt
} // namespace xpp

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_VARIABLE_SET_H_ */
