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

namespace xpp {
namespace zmp {

/** @brief hold the state of the optimization variables.
  *
  * This class is responsible for keeping the up-to-date values of the
  * optimization variables and supplying it to all the observers (cost function,
  * constraints, visualizers,...) that depend on this state.
  *
  * @ref https://sourcemaking.com/design_patterns/observer
  */
class OptimizationVariables : public ASubject {
public:
  typedef Eigen::VectorXd VectorXd;
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d; // for footholds
  typedef Eigen::Vector2d Vector2d;

  // fixme think about removing default constructor and use pointers if this
  // is supposed to be a member variable.
  OptimizationVariables ();
  OptimizationVariables (int n_spline_coeff, int n_steps);
  virtual ~OptimizationVariables () {};

  void Init (int n_spline_coeff, int n_steps);

  StdVecEigen2d GetFootholdsStd() const;
  VectorXd GetFootholdsEig () const;

  VectorXd GetSplineCoefficients() const;

  VectorXd GetOptimizationVariables() const { return x_; };
  int GetOptimizationVariableCount() const;

  void SetVariables(const VectorXd& x);
  void SetVariables(const double* x);
  void SetFootholds (const StdVecEigen2d& footholds);

private:
  VectorXd x_;                 ///< optimization variables
  NlpStructure nlp_structure_; ///< this class holds all the structural information of the NLP

  bool initialized_ = false; // checks if the init() method has been called
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OPTIMIZATION_VARIABLES_H_ */
