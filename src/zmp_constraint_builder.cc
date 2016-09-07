/**
 @file    zmp_contraint_builder.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Defines the ZmpConstraintBuilder class
 */

#include <xpp/zmp/zmp_constraint_builder.h>

#include <xpp/zmp/com_motion.h>
#include <xpp/hyq/support_polygon_container.h>

#include <xpp/zmp/zero_moment_point.h>
#include <xpp/utils/line_equation.h>

namespace xpp {
namespace zmp {

using NodeConstraints = xpp::hyq::SupportPolygon::VecSuppLine;
using JacobianRow = Eigen::SparseVector<double, Eigen::RowMajor>;

using namespace xpp::utils::coords_wrapper; // X, Y

ZmpConstraintBuilder::ZmpConstraintBuilder()
{
  com_motion_   = nullptr;
  contacts_     = nullptr;
}

ZmpConstraintBuilder::~ZmpConstraintBuilder()
{
}

void
ZmpConstraintBuilder::Init(const MotionStructure& structure,
                           const ComMotion& com_motion,
                           const SupportPolygonContainer& supp,
                           double walking_height,
                           double dt)
{
  motion_structure_ = structure;
  motion_structure_.SetDisretization(dt);
  com_motion_ = com_motion.clone();
  contacts_ = SuppPolygonPtrU(new SupportPolygonContainer(supp));



  walking_height_ = walking_height;

  // refactor remove this, all info contained in motion_info_
  double t_switch = 0.2; // the timeframe at which the constraint is relaxed
  times_ = GetTimesForConstraitEvaluation(dt, t_switch);

  // set coefficients to zero, since that is where I am approximating the function
  //around. can only do this in initialization, because ZMP is linear, so
  // Jacobians and offset are independent of current coefficients.
  com_motion_->SetCoefficientsZero();
  ZeroMomentPoint zmp(*com_motion_, times_, walking_height);
  jac_zmpx_0_ = zmp.GetJacobianWrtCoeff(X);
  jac_zmpy_0_ = zmp.GetJacobianWrtCoeff(Y);

  int n_motion = com_motion_->GetTotalFreeCoeff();
  int n_contacts = contacts_->GetTotalFreeCoeff();

  n_constraints_ = GetNumberOfConstraints();
  jac_wrt_motion_   = Jacobian(n_constraints_, n_motion);
  jac_wrt_contacts_ = Jacobian(n_constraints_, n_contacts);

  variables_changed_ = true;
}

std::vector<double>
ZmpConstraintBuilder::GetTimesDisjointSwitches () const
{
  std::vector<double> t_disjoint_switches;
  double t_global = 0;

  auto phases = com_motion_->GetPhases();
  for(int i=0; i<phases.size()-1; ++i) {

    auto phase = phases.at(i);
    t_global += phase.duration_;

    bool curr_phase_is_step = phase.type_   == PhaseInfo::kStepPhase;
    bool next_phase_is_step = phases.at(i+1).type_ == PhaseInfo::kStepPhase;

    if (curr_phase_is_step && next_phase_is_step) {
      int step = phase.n_completed_steps_;
      auto curr_leg = contacts_->GetLegID(step);
      auto next_leg = contacts_->GetLegID(step+1);

      if (SupportPolygonContainer::DisJointSupportPolygons(curr_leg, next_leg))
        t_disjoint_switches.push_back(t_global);

    }
  }

  return t_disjoint_switches;
}

std::vector<double>
ZmpConstraintBuilder::GetTimesForConstraitEvaluation (double dt, double t_cross) const
{
  std::vector<double> t_switch = GetTimesDisjointSwitches();
  bool skip_timestep = false;

  double T_first_phase = com_motion_->GetPhases().front().duration_;

  std::vector<double> t_constraint;

  // allow the zmp to be outside of the support polygon the entire first
  // swingphase and then catch itself
  // refactor don't forget, ignoring ZMP for first step
  double t = t_cross/2.; //T_first_phase + dt; // t_cross
  double t_total = com_motion_->GetTotalTime();
  while (t <= t_total) {
    skip_timestep = false;
    for (auto ts : t_switch) {
      bool time_close_to_switch = std::abs(t - ts) < t_cross/2.0;
      if (time_close_to_switch) { // don't add point in vicinity of t_cross
        t += dt;
        skip_timestep = true;
      }
    }

    if (skip_timestep) continue;

    t_constraint.push_back(t);
    t += dt;
  }

  return t_constraint;
}

void
ZmpConstraintBuilder::Update (const VectorXd& motion_coeff,
                              const VectorXd& footholds)
{
  com_motion_->SetCoefficients(motion_coeff);
  contacts_->SetFootholdsXY(utils::ConvertEigToStd(footholds));

  variables_changed_ = true;
}

int
ZmpConstraintBuilder::GetNumberOfConstraints () const
{
//  return motion_structure_.GetTotalNumberOfDiscreteContacts();

  auto supp_polygons = contacts_->AssignSupportPolygonsToPhases(com_motion_->GetPhases());
  int n_constraints = 0;
  for (auto t : times_) {
    int phase_id  = com_motion_->GetCurrentPhase(t).id_;
    NodeConstraints supp_line = supp_polygons.at(phase_id).GetLines();
    n_constraints += supp_line.size();
  }

  return n_constraints;
}

void
ZmpConstraintBuilder::UpdateJacobians (Jacobian& jac_motion,
                                       Jacobian& jac_contacts) const
{
  int n_contacts = contacts_->GetTotalFreeCoeff();

  // know the lines of of each support polygon
  auto supp_polygon = contacts_->AssignSupportPolygonsToPhases(com_motion_->GetPhases());

  int n = 0; // node counter
  int c = 0; // inequality constraint counter


    for (auto t : times_) {
//  for (auto node_new : motion_structure_.GetContactInfoVec()) {

    // the current position of the zero moment point

    auto state = com_motion_->GetCom(t/*node_new.time_*/);
    auto zmp = ZeroMomentPoint::CalcZmp(state.Make3D(), walking_height_);


    int phase_id  = com_motion_->GetCurrentPhase(t/*node_new.time_*/).id_;
    NodeConstraints node = supp_polygon.at(phase_id).GetLines();


//    hyq::SupportPolygon::SortCounterclockWise(node_new.legs_);



    for (int i=0; i<node.size(); ++i) {

      auto f_from = node.at(i).from;
      auto f_to = node.at(i).to;


      // get the jacobian of the line coefficient of each line
      utils::LineEquation::JacobianRow jac_line;
      // refactor do this only when phase change -> efficiency
      utils::LineEquation line(f_from.p.segment<2>(X), f_to.p.segment<2>(X));
      jac_line = line.GetJacobianDistanceWrtPoints(zmp);

      JacobianRow jac_line_wrt_contacts(n_contacts);
      jac_line_wrt_contacts.reserve(4); // every line depends on 4 points

      // reason, sparsity structure changes between calls
//      std::cout << f_from.id << "->" << f_to.id << "  ,  ";

      // only if line is not fixed by start stance does it go into the jacobian
      if (f_from.id != hyq::Foothold::kFixedByStart) {
        jac_line_wrt_contacts.insert(contacts_->Index(f_from.id, X)) = jac_line(0);
        jac_line_wrt_contacts.insert(contacts_->Index(f_from.id, Y)) = jac_line(1);
      }

      if (f_to.id != hyq::Foothold::kFixedByStart) {
        jac_line_wrt_contacts.insert(contacts_->Index(f_to.id, X))   = jac_line(2);
        jac_line_wrt_contacts.insert(contacts_->Index(f_to.id, Y))   = jac_line(3);
      }

      auto coeff = line.GetCoeff();
      // this line really impacts performance
      jac_motion.row(c)   = coeff.p*jac_zmpx_0_.row(n) + coeff.q*jac_zmpy_0_.row(n);
      jac_contacts.row(c) = jac_line_wrt_contacts;
      c++;
    }

    n++;

  }
}

ZmpConstraintBuilder::VectorXd
ZmpConstraintBuilder::GetDistanceToLineMargin () const
{
  auto supp_polygons = contacts_->AssignSupportPolygonsToPhases(com_motion_->GetPhases());

  VectorXd distance = VectorXd::Zero(n_constraints_);

  // for every time t
  int c=0; // constraint counter
  for (const auto& t : times_) {

    // the current position of the zero moment point
    auto state = com_motion_->GetCom(t);
    auto zmp = ZeroMomentPoint::CalcZmp(state.Make3D(), walking_height_);

    int phase_id  = com_motion_->GetCurrentPhase(t).id_;
    NodeConstraints supp_line = supp_polygons.at(phase_id).GetLines();

    for (auto i=0; i<supp_line.size(); ++i)
      distance(c++) = supp_line.at(i).GetDistanceToPoint(zmp);
  }

  return distance;
}

ZmpConstraintBuilder::Jacobian
ZmpConstraintBuilder::GetJacobianWrtMotion () const
{
  CheckAndUpdateJacobians();
  return jac_wrt_motion_;
}

ZmpConstraintBuilder::Jacobian
ZmpConstraintBuilder::GetJacobianWrtContacts () const
{
  CheckAndUpdateJacobians();
  return jac_wrt_contacts_;
}

void
ZmpConstraintBuilder::CheckAndUpdateJacobians () const
{
  if (variables_changed_) {
    UpdateJacobians(jac_wrt_motion_, jac_wrt_contacts_);
    variables_changed_ = false;
  }
}

} /* namespace zmp */
} /* namespace xpp */
