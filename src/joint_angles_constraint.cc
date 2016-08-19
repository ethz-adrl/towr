/**
 @file    joint_angles_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Brief description
 */

#include <xpp/zmp/joint_angles_constraint.h>
#include <xpp/zmp/optimization_variables.h>
#include <xpp/zmp/spline_container.h>

namespace xpp {
namespace zmp {

JointAnglesConstraint::JointAnglesConstraint ()
{
  inv_kin_ = nullptr;
}

JointAnglesConstraint::~JointAnglesConstraint ()
{
  // TODO Auto-generated destructor stub
}

void
JointAnglesConstraint::Init (const Interpreter& interpreter,
                             const InvKinPtr& inv_kin)
{
  interpreter_ = interpreter;
  inv_kin_ = inv_kin;
}

void
JointAnglesConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  VectorXd x_coeff      = opt_var->GetVariables(VariableNames::kSplineCoeff);
  VectorXd footholds_xy = opt_var->GetVariables(VariableNames::kFootholds);

  // calculate interpreted values from the optimization values
  VecFoothold footholds = interpreter_.GetFootholds(utils::ConvertEigToStd(footholds_xy));
  VecSpline splines     = interpreter_.GetSplines(x_coeff);

  vec_t_ = SplineContainer::GetDiscretizedGlobalTimes(splines);
  stance_feet_calc_.Update(interpreter_.GetStartStance(),
                           footholds, splines, interpreter_.GetRobotHeight());
}

JointAnglesConstraint::VectorXd
JointAnglesConstraint::EvaluateConstraint() const
{
  std::vector<double> g_vec;
  g_vec.reserve(vec_t_.size()*xpp::utils::kDim2d*4/*stance legs */);

  for (double t : vec_t_) {
    VecFoothold stance_b = stance_feet_calc_.GetStanceFeetInBase(t);

    for (const Foothold& f : stance_b) {

      JointAngles q;
//      try {
        q = inv_kin_->GetJointAngles(f.p, f.leg);
//      } catch (const std::runtime_error& e) {
//        // no joint angles can produce desired endeffector position
//        // ensure that it matches the size of constraint bounds to not mess up alignment
//        // todo make up some joint angles that violate constraint
//        std::cout << "violation for foothold: " << f << std::endl;
//        q = 500/*random number*/*inv_kin_->GetLowerJointLimits(f.leg);
//      }

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

  // fixme, duplicates code in Evaluate Constraint, use template pattern or similar
  for (double t : vec_t_) {
    VecFoothold stance_b = stance_feet_calc_.GetStanceFeetInBase(t);

    for (const Foothold& f : stance_b) {

      JointAngles q_min = inv_kin_->GetLowerJointLimits(f.leg);
      JointAngles q_max = inv_kin_->GetUpperJointLimits(f.leg);
      for (int i=0; i<q_min.rows(); ++i) {
        bounds.push_back(Bound(q_min[i], q_max[i]));
      }
    }
  }

  return bounds;
}

} /* namespace zmp */
} /* namespace xpp */
