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
  inv_kin_ = nullptr;
  // TODO Auto-generated constructor stub
}

JointAnglesConstraint::~JointAnglesConstraint ()
{
  // TODO Auto-generated destructor stub
}

void
JointAnglesConstraint::Init (const Interpreter& interpreter,
                             const VecFoothold& start_stance)
{
  interpreter_ = interpreter;
  start_stance_ = start_stance;
}

void
JointAnglesConstraint::Update ()
{
  VectorXd x_coeff        = subject_->GetSplineCoefficients();
  FootholdsXY x_footholds = subject_->GetFootholdsStd();

  // calculate interpreted values from the optimization values
  VecFoothold footholds = interpreter_.GetFootholds(x_footholds);
  VecSpline splines     = interpreter_.GetSplines(x_coeff);

  vec_t_ = SplineContainer::GetDiscretizedGlobalTimes(splines);
  stance_feet_calc_.Update(start_stance_, footholds, splines);
}

JointAnglesConstraint::VectorXd
JointAnglesConstraint::EvaluateConstraint() const
{

  std::vector<double> g_vec;
  g_vec.reserve(vec_t_.size()*xpp::utils::kDim2d*4/*stance legs */);

  for (double t : vec_t_) {
    VecFoothold stance_b = stance_feet_calc_.GetStanceFeetInBase(t);

    for (const xpp::hyq::Foothold& f : stance_b) {

      JointAngles3d q = inv_kin_->GetJointAngles(f.p, f.leg);
      for (int i=0; i<q.rows(); ++i) {
        g_vec.push_back(q[i]);
      }
    }
  }

  return Eigen::Map<const VectorXd>(&g_vec.front(),g_vec.size());
}


JointAnglesConstraint::VecBound
JointAnglesConstraint::GetBounds () const
{


  std::vector<Bound> bounds;
  VectorXd g = EvaluateConstraint(); // only need the number of constraints

  for (int i=0; i<g.rows(); ++i)
    bounds.push_back(kInequalityBoundPositive_);

  return bounds;
}

} /* namespace zmp */
} /* namespace xpp */
