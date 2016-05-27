/*
 * optimization_variables.h
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OPTIMIZATION_VARIABLES_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OPTIMIZATION_VARIABLES_H_

#include <xpp/zmp/a_subject.h>
#include <xpp/zmp/nlp_structure.h>

namespace xpp {
namespace zmp {

class OptimizationVariables : public ASubject {
public:
  typedef Eigen::VectorXd VectorXd;
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d; // for footholds

  OptimizationVariables (int n_spline_coeff, int n_steps);
  virtual ~OptimizationVariables () {};

  void NotifyObservers () const override;
  void RegisterObserver(IObserver* o) override;

  StdVecEigen2d GetFootholdsStd() const;
  VectorXd GetFootholdsEig () const;

  VectorXd GetSplineCoefficients() const;
  VectorXd GetOptimizationVariables() const { return x_; };
  int GetOptimizationVariableCount() const;

  void SetVariables(const VectorXd& x);
  void SetVariables(const double* x);
  void SetFootholds (const StdVecEigen2d& footholds);

//  // Singleton pattern: Ensure there is only one instance of this class in program
//  static OptimizationVariables& GetInstance(int n_spline_coeff, int n_steps);
//  // singletons are not allowed to be copied
//  OptimizationVariables(OptimizationVariables const&) = delete;
//  void operator=(OptimizationVariables const&)        = delete;
private:
  VectorXd x_;                ///< optimization variables
  NlpStructure nlp_structure_; // fixme this class seems redundant

};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OPTIMIZATION_VARIABLES_H_ */
