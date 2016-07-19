/**
 @file    constraint_container.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Provides a class to combine knowledge of individual constraints.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINT_CONTAINER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINT_CONTAINER_H_

#include <xpp/zmp/a_constraint.h>
#include <xpp/zmp/i_observer.h>
#include <xpp/zmp/optimization_variables.h>

#include <map>
#include <memory>

namespace xpp {
namespace zmp {

/** @brief Knows about all constraints and gives information about them.
  *
  * For every constraint that \c ConstraintContainer knows about, it will return
  * the constraint violations and the acceptable bounds.
  */
class ConstraintContainer : public IObserver {
public:
  typedef AConstraint::VectorXd VectorXd;
  typedef AConstraint::VecBound VecBound;
  typedef std::shared_ptr<AConstraint> ConstraintPtr;
  typedef xpp::utils::StdVecEigen2d FootholdsXY;

  ConstraintContainer (OptimizationVariables& subject);
  virtual ~ConstraintContainer ();

  void Update () override;

  void AddConstraint (ConstraintPtr constraint,
                      const std::string& name);

  AConstraint& GetConstraint(const std::string& name);

  VectorXd EvaluateConstraints () const;
  VecBound GetBounds () const;
  void RefreshBounds ();


  const FootholdsXY& GetFootholds() const;
  const VectorXd& GetSplineCoefficients() const;

private:
  std::map<std::string, ConstraintPtr > constraints_;
  VecBound bounds_;

  VectorXd spline_coeff_;
  FootholdsXY footholds_;

  bool bounds_up_to_date_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINT_CONTAINER_H_ */
