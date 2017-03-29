/**
 @file    a_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Abstract class representing a constraint for the NLP problem.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_A_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_A_CONSTRAINT_H_

#include "optimization_variables.h"
#include "parametrization.h"

#include <Eigen/Sparse>
#include <Eigen/Dense>

#include <memory>

namespace xpp {
namespace opt {

class Constraint {
public:
  using VectorXd = Eigen::VectorXd;
  using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  using ParametrizationPtr = std::shared_ptr<Parametrization>;
  using VarPair = std::pair<ParametrizationPtr,Jacobian>;

  Constraint ();
  virtual ~Constraint ();


  virtual void UpdateVariables(const OptimizationVariables*);

  /** The Jacobian of the constraints with respect to each decision variable set
    */
  // zmp_ remove one of these
  virtual Jacobian GetJacobianWithRespectTo (std::string var_set) const;
  virtual Jacobian& GetJacobianRefWithRespectTo (std::string var_set);

  /** A constraint always delivers a vector of constraint violations.
   */
  virtual VectorXd EvaluateConstraint () const = 0;

  /** For each returned constraint an upper and lower bound is given.
   */
  virtual VecBound GetBounds () const = 0;

  int GetNumberOfConstraints() const;

  void PrintStatus(double tol) const;

protected:
  void SetDependentVariables(const std::vector<ParametrizationPtr>&, int num_constraints);

  /** Implement in derived class if Jacobians change with opt. variables */
  virtual void UpdateJacobians() {};

  std::string name_;


  mutable std::vector<VarPair> variables_;
  mutable VectorXd g_;
  mutable VecBound bounds_;
  int num_constraints_ = 0;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_A_CONSTRAINT_H_ */
