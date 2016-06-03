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
#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/hyq/foothold.h>

namespace xpp {
namespace zmp {

/** @brief hold the state of the optimization variables.
  *
  * This class is responsible for keeping the up-to-date values of the
  * optimization variables and supplying semantic information of how to interpret
  * the values of the optimization variables.
  */
class OptimizationVariables : public ASubject {
public:
  typedef Eigen::VectorXd VectorXd;
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d; // for footholds
  typedef std::vector<xpp::hyq::Foothold> VecFoothold;
  typedef SplineContainer::VecSpline VecSpline;
  typedef Eigen::Vector2d Vector2d;

  OptimizationVariables ();
  virtual ~OptimizationVariables () {};

  // todo, remove this method, it introduces to many dependencies
  void Init(const Vector2d& start_cog_p,
            const Vector2d& start_cog_v,
            const std::vector<xpp::hyq::LegID>& step_sequence,
            const SplineTimes& times);

  StdVecEigen2d GetFootholdsStd() const;
  VectorXd GetFootholdsEig () const;
//  VecFoothold GetFootholds() const;

  VectorXd GetSplineCoefficients() const;
//  VecSpline GetSplines();

  VectorXd GetOptimizationVariables() const { return x_; };
  int GetOptimizationVariableCount() const;

  void SetVariables(const VectorXd& x);
  void SetVariables(const double* x);
  void SetFootholds (const StdVecEigen2d& footholds);

private:
  VectorXd x_;                 ///< optimization variables
  NlpStructure nlp_structure_; ///< this class holds all the structural information of the NLP

  std::vector<xpp::hyq::LegID> step_sequence_;
  ContinuousSplineContainer spline_structure_;

  bool initialized_ = false; // checks if the init() method has been called
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OPTIMIZATION_VARIABLES_H_ */
