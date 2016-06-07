/**
 @file    joint_angles_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Brief description
 */

#include <xpp/zmp/joint_angles_constraint.h>
#include <xpp/zmp/spline_container.h>

#include <xpp/zmp/optimization_variables.h>

namespace xpp {
namespace zmp {

JointAnglesConstraint::JointAnglesConstraint (OptimizationVariables& subject)
    :IObserver(subject)
{
  // TODO Auto-generated constructor stub
}

JointAnglesConstraint::~JointAnglesConstraint ()
{
  // TODO Auto-generated destructor stub
}

void
JointAnglesConstraint::Update ()
{
  x_coeff_   = subject_->GetSplineCoefficients();
  x_footholds_ = subject_->GetFootholdsStd();

  // calculate interpreted values from the optimization values
  VecFoothold footholds = interpreter_.GetFootholds(x_footholds_);
  splines_ = interpreter_.GetSplines(x_coeff_);

  stance_feet_calc_.Update(start_stance_, footholds, splines_);
}

void
JointAnglesConstraint::AddInterpreter (const Interpreter& interpreter)
{
  interpreter_ = interpreter;
}

JointAnglesConstraint::VectorXd
JointAnglesConstraint::EvaluateConstraint() const
{

  for (double t : SplineContainer::GetDiscretizedGlobalTimes(splines_)) {
    VecFoothold stance_b = stance_feet_calc_.GetStanceFeetInBase(t);

    for (const xpp::hyq::Foothold& f : stance_b) {

      JointAngles3d q = inv_kin_->GetJointAngles(f.p, f.leg);
    }


  }


}


JointAnglesConstraint::VecBound
JointAnglesConstraint::GetBounds () const
{
}

} /* namespace zmp */
} /* namespace xpp */
