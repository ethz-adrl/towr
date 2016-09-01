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
  supp_polygon_ = nullptr;
}

ZmpConstraintBuilder::ZmpConstraintBuilder(const ComMotion& com_motion,
                                           const SupportPolygonContainer& supp,
                                           double walking_height)
{
  Init(com_motion, supp, walking_height);
}

ZmpConstraintBuilder::~ZmpConstraintBuilder()
{
}

void
ZmpConstraintBuilder::Init(const ComMotion& com_motion,
                           const SupportPolygonContainer& supp,
                           double walking_height)
{
  com_motion_ = com_motion.clone();
  // set coefficients to zero, since that is where I am approximating the function
  //around. can only do this in initialization, because ZMP is linear, so
  // Jacobians and offset are independent of current coefficients.
  com_motion_->SetCoefficientsZero();

  supp_polygon_ = SuppPolygonPtrU(new SupportPolygonContainer(supp));
  walking_height_ = walking_height;







  // calculate the times at which support polygon constraint should be active
  auto phases = com_motion_->GetPhases();
  auto polygons = supp_polygon_->AssignSupportPolygonsToPhases(*com_motion_);

  std::vector<double> times;

//  std::iota(times.begin(), times.end(), 0);
//
//  for (auto phase : phases) {
//    if (
//
//
//  }


//  static constexpr double dt = 0.1; //discretization time [seconds]: needed for creating support triangle inequality constraints
//  static constexpr double eps = 1e-10; // maximum inaccuracy when adding double numbers
//
//  double t = 0.0;
//  while (t <= com_motion_->GetTotalTime()-dt+eps) { // still add the second to last time, even if rounding errors to to floating point arithmetics
//
//    auto phase = com_motion_->GetCurrentPhase(t);
//    auto poly = polygons.at(phase.id_);


  std::vector<double> t_disjoint_switches;
  for(int i=0; i<phases.size()-1; ++i) {

    if (phases.at(i).type_ == kStepPhase) {
      int step = phases.at(i).n_completed_steps_;
      auto curr_leg = supp_polygon_->GetLegID(step);

      if (phases.at(i+1).type_ == kStepPhase) {
        auto next_leg = supp_polygon_->GetLegID(step+1);

        if (SupportPolygonContainer::DisJointSupportPolygons(curr_leg, next_leg))
          t_disjoint_switches.push_back(phases.at(i).duration_);
      }
    }
  }





  ZeroMomentPoint zmp(*com_motion_, com_motion_->GetDiscretizedGlobalTimes(), walking_height);
  jac_zmpx_0_ = zmp.GetJacobianWrtCoeff(X);
  jac_zmpy_0_ = zmp.GetJacobianWrtCoeff(Y);

  int n_motion = com_motion_->GetTotalFreeCoeff();
  int n_contacts = supp_polygon_->GetTotalFreeCoeff();

  n_constraints_ = GetNumberOfConstraints();
  jac_wrt_motion_   = Jacobian(n_constraints_, n_motion);
  jac_wrt_contacts_ = Jacobian(n_constraints_, n_contacts);

  CalcJacobians();
}

std::vector<double>
ZmpConstraintBuilder::GetTimesDisjointSwitches () const
{
  auto phases = com_motion_->GetPhases();

  std::vector<double> t_disjoint_switches;
  double t_global = 0;
  for(int i=0; i<phases.size()-1; ++i) {

    t_global += phases.at(i).duration_;

    bool curr_phase_is_step = phases.at(i).type_   == kStepPhase;
    bool next_phase_is_step = phases.at(i+1).type_ == kStepPhase;

    if (curr_phase_is_step && next_phase_is_step) {
      int step = phases.at(i).n_completed_steps_;
      auto curr_leg = supp_polygon_->GetLegID(step);
      auto next_leg = supp_polygon_->GetLegID(step+1);

      if (SupportPolygonContainer::DisJointSupportPolygons(curr_leg, next_leg))
        t_disjoint_switches.push_back(t_global);

    }
  }

  return t_disjoint_switches;
}

void
ZmpConstraintBuilder::Update (const VectorXd& motion_coeff,
                              const VectorXd& footholds)
{
  com_motion_->SetCoefficients(motion_coeff);
  supp_polygon_->SetFootholdsXY(utils::ConvertEigToStd(footholds));

  CalcJacobians();
}

int
ZmpConstraintBuilder::GetNumberOfConstraints () const
{
  // refactor this discretized global times could specify where to evaluate zmp constraint
  auto vec_t = com_motion_->GetDiscretizedGlobalTimes();
  auto supp_polygons = supp_polygon_->AssignSupportPolygonsToPhases(*com_motion_);
  int n_constraints = 0;
  for (auto t : vec_t) {
    int phase_id  = com_motion_->GetCurrentPhase(t).id_;
    NodeConstraints supp_line = supp_polygons.at(phase_id).GetLines();
    n_constraints += supp_line.size();
  }

  return n_constraints;
}

void
ZmpConstraintBuilder::CalcJacobians ()
{
  // refactor this discretized global times could specify where to evaluate zmp constraint
  int n_contacts = supp_polygon_->GetTotalFreeCoeff();

  // know the lines of of each support polygon
  auto supp_polygon = supp_polygon_->AssignSupportPolygonsToPhases(*com_motion_);

  int n = 0; // node counter
  int c = 0; // inequality constraint counter

  for (const auto& t : com_motion_->GetDiscretizedGlobalTimes()) {

    // the current position of the zero moment point
    auto state = com_motion_->GetCom(t);
    auto zmp = ZeroMomentPoint::CalcZmp(state.Make3D(), walking_height_);

    int phase_id  = com_motion_->GetCurrentPhase(t).id_;
    NodeConstraints node = supp_polygon.at(phase_id).GetLines();

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
        jac_line_wrt_contacts.insert(supp_polygon_->Index(f_from.id, X)) = jac_line(0);
        jac_line_wrt_contacts.insert(supp_polygon_->Index(f_from.id, Y)) = jac_line(1);
      }

      if (f_to.id != hyq::Foothold::kFixedByStart) {
        jac_line_wrt_contacts.insert(supp_polygon_->Index(f_to.id, X))   = jac_line(2);
        jac_line_wrt_contacts.insert(supp_polygon_->Index(f_to.id, Y))   = jac_line(3);
      }

      auto coeff = line.GetCoeff();
      // this line really impacts performance
      jac_wrt_motion_.row(c)   = coeff.p*jac_zmpx_0_.row(n) + coeff.q*jac_zmpy_0_.row(n);
      jac_wrt_contacts_.row(c) = jac_line_wrt_contacts;
      c++;
    }

    n++;

  }
}

ZmpConstraintBuilder::VectorXd
ZmpConstraintBuilder::GetDistanceToLineMargin () const
{
  // refactor this discretized global times could specify where to evaluate zmp constraint
  auto vec_t = com_motion_->GetDiscretizedGlobalTimes();
  auto supp_polygons = supp_polygon_->AssignSupportPolygonsToPhases(*com_motion_);

  VectorXd distance = VectorXd::Zero(n_constraints_);

  // for every time t
  int c=0; // constraint counter
  // refactor exclude times where the splines transition between disjoint support polygons
  for (const auto& t : vec_t) {

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
  return jac_wrt_motion_;
}

ZmpConstraintBuilder::Jacobian
ZmpConstraintBuilder::GetJacobianWrtContacts () const
{
  return jac_wrt_contacts_;
}

//ZmpConstraintBuilder::MatVecVec
//ZmpConstraintBuilder::CalcZmpConstraints(const SupportPolygonContainer& s) const
//{
//  CheckIfInitialized();
//  return CalcZmpConstraints(jac_px_0_, jac_py_0_, s);
//};
//
//
//ZmpConstraintBuilder::MatVecVec
//ZmpConstraintBuilder::CalcZmpConstraints(const MatVec& jac_px_0, const MatVec& jac_py_0,
//                                  const SupportPolygonContainer& supp_polygon_container) const
//{
//  std::vector<NodeConstraint> supp_lines = supp_polygon_container.GetActiveConstraintsForEachPhase(*spline_structure_);
//
//  // if every spline is a four leg support spline with 4 line constraints
//  auto vec_t = spline_structure_->GetDiscretizedGlobalTimes();
//  const int max_num_constraints = vec_t.size()*SupportPolygon::kMaxSides;
//  int coeff = spline_structure_->GetTotalFreeCoeff();
//  MatVecVec ineq(max_num_constraints, coeff);
//
//  int n = 0; // node counter
//  int c = 0; // inequality constraint counter
//
//  for (double t_global : vec_t) {
//    int spline_id = spline_structure_->GetPolynomialID(t_global);
//    int phase_id  = spline_structure_->GetCurrentPhase(t_global).id_;
//
//    // splines switch and disregard constraint at these times.
//    if (DisjSuppSwitch(t_global, spline_structure_->GetPolynomial(spline_id), supp_polygon_container)) {
//      n++; // no constraints
//      continue;
//    }
//
//    GenerateNodeConstraint(supp_lines.at(phase_id), jac_px_0.GetRow(n), jac_py_0.GetRow(n), c, ineq);
//
//    n++;
//    c += SupportPolygon::kMaxSides;
//  }
//
//  assert(c <= max_num_constraints);
////  assert((n == x_zmp.M.rows()) && (n == y_zmp.M.rows())); // don't need constraint for every node
//  return ineq;
//}
//
//void
//ZmpConstraintBuilder::GenerateNodeConstraint(const NodeConstraint& node_constraints,
//                                      const VecScalar& x_zmp,
//                                      const VecScalar& y_zmp,
//                                      int row_start,
//                                      MatVecVec& ineq)
//{
//  // add three or four line constraints depending on if support triange/ support polygon etc
//  for (SupportPolygon::SuppLine l : node_constraints) {
//    VecScalarScalar constr = GenerateLineConstraint(l, x_zmp, y_zmp);
//    ineq.WriteRow(constr,row_start++);
//  }
//}
//
//ZmpConstraintBuilder::VecScalarScalar
//ZmpConstraintBuilder::GenerateLineConstraint(const SupportPolygon::SuppLine& l,
//                                      const VecScalar& x_zmp,
//                                      const VecScalar& y_zmp)
//{
//
//  utils::LineEquation line(l.from.p.segment<2>(X), l.to.p.segment<2>(X));
//  auto coeff = line.GetCoeff();
//
//  VecScalarScalar line_constr_sep;
//  // separate the constraints that depend only on the start stance with the ones
//  // that depend on the optimized footholds
//
//  line_constr_sep.vs.v  = coeff.p*x_zmp.v + coeff.q*y_zmp.v;
//  line_constr_sep.vs.s = 0;
//  line_constr_sep.constant = -l.s_margin;
//
//  double line_coeff_terms = coeff.p*x_zmp.s + coeff.q*y_zmp.s + coeff.r;
//  if (l.fixed_by_start_stance)
//    line_constr_sep.constant += line_coeff_terms;
//  else
//    line_constr_sep.vs.s += line_coeff_terms;
//
//  return line_constr_sep;
//}
//
//bool
//ZmpConstraintBuilder::DisjSuppSwitch (double t, const ComPolynomial& curr_spline,
//                                      const SupportPolygonContainer& supp_polygon_container) const
//{
//  if (!curr_spline.IsFourLegSupport()) {
//
//    int step = curr_spline.GetCurrStep();
//    double t_local = spline_structure_->GetLocalTime(t);
//    static const double t_stance = 0.2; // time to switch between disjoint support triangles
//    double t_start_local = curr_spline.GetDuration() - t_stance/2;
//
//    if (DisjointSuppPolygonsAtBeginning(step,supp_polygon_container) && t_local < t_stance/2.)
//      return true;
//    if (DisjointSuppPolygonsAtEnd(step,supp_polygon_container) && t_local > t_start_local)
//      return true;
//  }
//  return false;
//}
//
//bool
//ZmpConstraintBuilder::DisjointSuppPolygonsAtBeginning(
//    int step, const SupportPolygonContainer& supp_polygon_container) const
//{
//  LegID swing_leg = supp_polygon_container.GetLegID(step);
//  if (step == 0) {
//    return false; // don't allow initial zmp to violate constraint for first part of first step
//  } else {
//    LegID prev_swing_leg = supp_polygon_container.GetLegID(step-1);
//    return Insert4LSPhase(prev_swing_leg, swing_leg);
//  }
//}
//
//bool
//ZmpConstraintBuilder::DisjointSuppPolygonsAtEnd(
//    int step, const SupportPolygonContainer& supp_polygon_container) const
//{
//  LegID swing_leg = supp_polygon_container.GetLegID(step);
//  bool last_step = step == supp_polygon_container.GetNumberOfSteps()-1;
//  if (last_step) {
//    return true; // allow zmp to violate constraint at last part of last step
//  } else {
//    LegID next_swing_leg = supp_polygon_container.GetLegID(step+1);
//    return Insert4LSPhase(swing_leg, next_swing_leg);
//  }
//}
//

} /* namespace zmp */
} /* namespace xpp */
