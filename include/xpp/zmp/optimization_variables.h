/**
 @file    cost_function_functor.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 23, 2016
 @brief   Defines a class to hold the value of the optimization variables.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OPTIMIZATION_VARIABLES_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OPTIMIZATION_VARIABLES_H_

#include <xpp/zmp/a_subject.h>
#include <xpp/zmp/nlp_structure.h>
#include <xpp/utils/geometric_structs.h>

namespace xpp {
namespace zmp {

/** @brief hold the state of the optimization variables.
  *
  * This class is responsible for publishing the up-to-date values of the
  * optimization variables to all the observers (cost function,
  * constraints, visualizers,...) that depend on this state. It doesn't know about
  * internal structure of the variables, that is all handled by the owned member.
  * It is able to interpret the values of the optimization variables if they are
  * independent of the initialization values of the NLP.
  *
  * https://sourcemaking.com/design_patterns/observer
  */
class OptimizationVariables : public ASubject {
public:
  typedef Eigen::VectorXd VectorXd;
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d; // for footholds
  typedef Eigen::Vector2d Vector2d;

  OptimizationVariables ();
  virtual ~OptimizationVariables ();

  void Init (int n_spline_coeff, int n_steps);
  void Init (const VectorXd& x_coeff_abcd, const StdVecEigen2d& footholds);

  VectorXd GetOptimizationVariables() const;
  StdVecEigen2d GetFootholdsStd() const;
  VectorXd GetSplineCoefficients() const;
  int GetOptimizationVariableCount() const;

  void SetVariables(const double* x);
  void SetVariables(const VectorXd& x);

  void SetSplineCoefficients(const VectorXd& x);

private:
  NlpStructure nlp_structure_; ///< this class holds all the structural information of the NLP


  VectorXd ConvertStdToEig(const StdVecEigen2d& footholds_xy) const;

  bool initialized_ = false; // checks if the init() method has been called
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OPTIMIZATION_VARIABLES_H_ */
