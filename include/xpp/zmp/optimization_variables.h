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
  virtual ~OptimizationVariables ();

  void NotifyObservers () override;
  void RegisterObserver(IObserver* o) override;

  void SetStructure(int n_spline_coeff, int n_steps);

  StdVecEigen2d GetFootholds() const;
  VectorXd GetSplineCoefficients() const;
  int GetOptimizationVariableCount() const;

  void SetVariables(const VectorXd& x);




private:
  VectorXd x_;                ///< optimization variables
  NlpStructure nlp_structure_; // fixme this class seems redundant
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OPTIMIZATION_VARIABLES_H_ */
