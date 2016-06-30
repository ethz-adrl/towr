/**
 @file    nlp_structure.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 8, 2016
 @brief   Declares the class NlpStructure
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
  typedef Eigen::VectorXd VectorXd;
  typedef double Number;

  NlpStructure(int n_spline_coeff = 0, int n_steps = 0);
  virtual ~NlpStructure();
  void Init(int n_spline_coeff = 0, int n_steps = 0);

  VectorXd GetOptimizationVariables() const;
  int GetOptimizationVariableCount() const;
  VectorXd GetSplineCoefficients() const;
  StdVecEigen2d GetFootholdsStd() const;

  void SetAllVariables(const VectorXd& x_all);
  void SetAllVariables(const Number* x_all);
  void SetSplineCoefficients(const VectorXd& x_abdc);
  void SetFootholds(const StdVecEigen2d& footholds_xy);

private:
  VectorXd x_;  ///< optimization variables

  int n_spline_coeff_;
  int n_steps_;

  VectorXd GetFootholdsEig() const;
  VectorXd ConvertToEigen(const Number* x) const;
  VectorXd ConvertStdToEig(const StdVecEigen2d& footholds_xy) const;
};

} // namespace zmp
} // namespace xpp

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_STRUCTURE_H_ */
