/*
 * zmp_constraint.cc
 *
 *  Created on: Apr 4, 2016
 *      Author: winklera
 */

#include <xpp/zmp/zmp_constraint_builder.h>

namespace xpp {
namespace zmp {

ZmpConstraintBuilder::ZmpConstraintBuilder(const ComSplinePtr& spline_container, double walking_height)
{
  Init(spline_container, walking_height);
}

void
ZmpConstraintBuilder::Init(const ComSplinePtr& spline_container, double walking_height)
{
  spline_structure_ = spline_container->clone();
  // set coefficients to zero, since that is where I am approximating the function
  //around. can only do this in initialization, because ZMP is linear, so
  // Jacobians and offset are independent of current coefficients.
  spline_structure_->SetCoefficientsZero();

  using namespace xpp::utils::coords_wrapper;

  walking_height_ = walking_height;
  jac_px_0_ = ZeroMomentPoint::GetLinearApproxWrtMotionCoeff(*spline_structure_, walking_height, X);
  jac_py_0_ = ZeroMomentPoint::GetLinearApproxWrtMotionCoeff(*spline_structure_, walking_height, Y);

  initialized_ = true;
}

ZmpConstraintBuilder::MatVecVec
ZmpConstraintBuilder::CalcZmpConstraints(const SupportPolygonContainer& s) const
{
  CheckIfInitialized();
  return CalcZmpConstraints(jac_px_0_, jac_py_0_, s);
};

ZmpConstraintBuilder::MatVecVec
ZmpConstraintBuilder::CalcZmpConstraints(const MatVec& jac_px_0, const MatVec& jac_py_0,
                                  const SupportPolygonContainer& supp_polygon_container) const
{
  std::vector<NodeConstraint> supp_lines = supp_polygon_container.GetActiveConstraintsForEachPhase(*spline_structure_);

  // if every spline is a four leg support spline with 4 line constraints
  auto vec_t = spline_structure_->GetDiscretizedGlobalTimes();
  const int max_num_constraints = vec_t.size()*SupportPolygon::kMaxSides;
  int coeff = spline_structure_->GetTotalFreeCoeff();
  MatVecVec ineq(max_num_constraints, coeff);

  int n = 0; // node counter
  int c = 0; // inequality constraint counter

  for (double t_global : vec_t) {
    int spline_id = spline_structure_->GetPolynomialID(t_global);
    int phase_id  = spline_structure_->GetCurrentPhase(t_global).id_;

    // refactor have one function that knows at which global time the
    // splines switch and disregard constraint at these times.
    if (DisjSuppSwitch(t_global, spline_structure_->GetPolynomial(spline_id), supp_polygon_container)) {
      n++; // no constraints
      continue;
    }

    GenerateNodeConstraint(supp_lines.at(phase_id), jac_px_0.GetRow(n), jac_py_0.GetRow(n), c, ineq);

    n++;
    c += SupportPolygon::kMaxSides;
  }

  assert(c <= max_num_constraints);
//  assert((n == x_zmp.M.rows()) && (n == y_zmp.M.rows())); // don't need constraint for every node
  return ineq;
}

void
ZmpConstraintBuilder::GenerateNodeConstraint(const NodeConstraint& node_constraints,
                                      const VecScalar& x_zmp,
                                      const VecScalar& y_zmp,
                                      int row_start,
                                      MatVecVec& ineq)
{
  // add three or four line constraints depending on if support triange/ support polygon etc
  for (SupportPolygon::SuppLine l : node_constraints) {
    VecScalarScalar constr = GenerateLineConstraint(l, x_zmp, y_zmp);
    ineq.WriteRow(constr,row_start++);
  }
}

ZmpConstraintBuilder::VecScalarScalar
ZmpConstraintBuilder::GenerateLineConstraint(const SupportPolygon::SuppLine& l,
                                      const VecScalar& x_zmp,
                                      const VecScalar& y_zmp)
{

  // refactor shouldn't calculate this everytime
  utils::LineEquation line(l.from.p.segment<2>(X), l.to.p.segment<2>(X));
  auto coeff = line.GetCoeff();

  VecScalarScalar line_constr_sep;
  // separate the constraints that depend only on the start stance with the ones
  // that depend on the optimized footholds

  line_constr_sep.vs.v  = coeff.p*x_zmp.v + coeff.q*y_zmp.v;
  line_constr_sep.vs.s = 0;
  line_constr_sep.constant = -l.s_margin;

  double line_coeff_terms = coeff.p*x_zmp.s + coeff.q*y_zmp.s + coeff.r;
  if (l.fixed_by_start_stance)
    line_constr_sep.constant += line_coeff_terms;
  else
    line_constr_sep.vs.s += line_coeff_terms;

  return line_constr_sep;
}

bool
ZmpConstraintBuilder::DisjSuppSwitch (double t, const ComPolynomial& curr_spline,
                                      const SupportPolygonContainer& supp_polygon_container) const
{
  if (!curr_spline.IsFourLegSupport()) {

    int step = curr_spline.GetCurrStep();
    double t_local = spline_structure_->GetLocalTime(t);
    static const double t_stance = 0.2; // time to switch between disjoint support triangles
    double t_start_local = curr_spline.GetDuration() - t_stance/2;

    if (DisjointSuppPolygonsAtBeginning(step,supp_polygon_container) && t_local < t_stance/2.)
      return true;
    if (DisjointSuppPolygonsAtEnd(step,supp_polygon_container) && t_local > t_start_local)
      return true;
  }
  return false;
}

bool
ZmpConstraintBuilder::DisjointSuppPolygonsAtBeginning(
    int step, const SupportPolygonContainer& supp_polygon_container) const
{
  LegID swing_leg = supp_polygon_container.GetLegID(step);
  if (step == 0) {
    return false; // don't allow initial zmp to violate constraint for first part of first step
  } else {
    LegID prev_swing_leg = supp_polygon_container.GetLegID(step-1);
    return Insert4LSPhase(prev_swing_leg, swing_leg);
  }
}

bool
ZmpConstraintBuilder::DisjointSuppPolygonsAtEnd(
    int step, const SupportPolygonContainer& supp_polygon_container) const
{
  LegID swing_leg = supp_polygon_container.GetLegID(step);
  bool last_step = step == supp_polygon_container.GetNumberOfSteps()-1;
  if (last_step) {
    return true; // allow zmp to violate constraint at last part of last step
  } else {
    LegID next_swing_leg = supp_polygon_container.GetLegID(step+1);
    return Insert4LSPhase(swing_leg, next_swing_leg);
  }
}

bool
ZmpConstraintBuilder::Insert4LSPhase(LegID prev, LegID next)
{
  using namespace xpp::hyq;
  // check for switching between disjoint support triangles.
  // the direction the robot is moving between triangles does not matter.
  if ((prev==LF && next==RH) || (prev==RF && next==LH)) return true;
  std::swap(prev, next);
  if ((prev==LF && next==RH) || (prev==RF && next==LH)) return true;

  return false;
}

ZmpConstraintBuilder::MatrixXd
ZmpConstraintBuilder::GetJacobian (const SupportPolygonContainer& s) const
{
  // refactor this discretized global times could specify where to evaluate zmp constraint
  auto vec_t = spline_structure_->GetDiscretizedGlobalTimes();
  auto n_nodes = vec_t.size();

  int n_motion = spline_structure_->GetTotalFreeCoeff();
  int n_contacts = s.GetTotalFreeCoeff();

  int input_dim = n_motion + n_contacts;
  int output_dim = n_nodes*SupportPolygon::kMaxSides; // this is ugly

  // know the lines of of each support polygon
  auto supp_lines = s.GetActiveConstraintsForEachPhase(*spline_structure_);

  using JacobianRow = Eigen::RowVectorXd;
  using MatrixXd = Eigen::MatrixXd;

  JacobianRow jac_line_full = JacobianRow::Zero(n_contacts);
  MatrixXd jac = MatrixXd::Zero(output_dim, input_dim);

  int n = 0; // node counter
  int c = 0; // inequality constraint counter

  // for every time t
  for (const auto& t : vec_t) {

    // the current position of the zero moment point
    auto state = spline_structure_->GetCom(t);
    auto zmp = ZeroMomentPoint::CalcZmp(state.Make3D(), walking_height_);

    int phase_id  = spline_structure_->GetCurrentPhase(t).id_;
    NodeConstraint supp_line = supp_lines.at(phase_id);

    int num_lines = supp_line.size();
    Eigen::VectorXd lx(num_lines);
    Eigen::VectorXd ly(num_lines);
    MatrixXd jac_node_wrt_contacts = MatrixXd::Zero(num_lines, n_contacts);

    for (int i=0; i<num_lines; ++i) {


      auto f_from = supp_line.at(i).from;
      auto f_to = supp_line.at(i).to;

      // refactor do this only when phase change -> efficiency
      utils::LineEquation line(f_from.p.segment<2>(X), f_to.p.segment<2>(X));


      auto coeff = line.GetCoeff();
      lx(i) = coeff.p;
      ly(i) = coeff.q;

      // get the jacobian of the line coefficient of each line
      utils::LineEquation::JacobianRow jac_line;
      jac_line = line.GetJacobianDistanceWrtPoints(zmp);


      // only if line is not fixed by start stance does it go into the jacobian
      if (!f_from.fixed_by_start_stance) {
        jac_node_wrt_contacts(i,s.Index(f_from.id, X)) = jac_line(0);
        jac_node_wrt_contacts(i,s.Index(f_from.id, Y)) = jac_line(1);
      }

      if (!f_to.fixed_by_start_stance) {
        jac_node_wrt_contacts(i,s.Index(f_to.id, X))   = jac_line(2);
        jac_node_wrt_contacts(i,s.Index(f_to.id, Y))   = jac_line(3);
      }

    }

    JacobianRow jac_zmp_wrt_x_t = jac_px_0_.M.row(n);
    JacobianRow jac_zmp_wrt_y_t = jac_py_0_.M.row(n);


    MatrixXd jac_node = Eigen::MatrixXd::Zero(num_lines, input_dim);

    // this information should not actually be known
    jac_node.leftCols(n_motion)    = lx*jac_zmp_wrt_x_t + ly*jac_zmp_wrt_y_t;
    jac_node.rightCols(n_contacts) = jac_node_wrt_contacts;


    jac.middleRows(c,num_lines) = jac_node;
    c += SupportPolygon::kMaxSides; // will create blank spaces in polygons, but keep constraint affiliation same
    n++;







  }


  // build jacobian w.r.t x position;


  return jac;
}

ZmpConstraintBuilder::VectorXd
ZmpConstraintBuilder::GetDistanceToLineMargin (const SupportPolygonContainer& s) const
{
  // refactor this discretized global times could specify where to evaluate zmp constraint
  auto vec_t = spline_structure_->GetDiscretizedGlobalTimes();
  auto n_nodes = vec_t.size();
  int output_dim = n_nodes*SupportPolygon::kMaxSides; // this is ugly

  // know the lines of of each support polygon
  auto supp_lines = s.GetActiveConstraintsForEachPhase(*spline_structure_);

  VectorXd distance = VectorXd::Zero(output_dim);


  // for every time t
  int c=0; // constraint counter
  // refactor exclude times where the splines transition between disjoint support polygons
  for (const auto& t : vec_t) {

    // the current position of the zero moment point
    auto state = spline_structure_->GetCom(t);
    auto zmp = ZeroMomentPoint::CalcZmp(state.Make3D(), walking_height_);

    int phase_id  = spline_structure_->GetCurrentPhase(t).id_;
    NodeConstraint supp_line = supp_lines.at(phase_id);

    for (const auto& l : supp_line)
      distance(c++) = GetDistanceToLineMargin(zmp, l);
  }

  return distance;
}

double
ZmpConstraintBuilder::GetDistanceToLineMargin (const Vector2d& zmp, SuppLine supp_line) const
{
  auto f_from = supp_line.from;
  auto f_to   = supp_line.to;

  utils::LineEquation line(f_from.p.segment<2>(X), f_to.p.segment<2>(X));

  return line.GetDistanceFromLine(zmp) - supp_line.s_margin;
}

void
ZmpConstraintBuilder::CheckIfInitialized() const
{
  if (!initialized_) {
    throw std::runtime_error("ZmpConstraintBuilder not initialized. Call Init() first");
  }
}

} /* namespace zmp */
} /* namespace xpp */


