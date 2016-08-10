/*
 * zmp_constraint.cc
 *
 *  Created on: Apr 4, 2016
 *      Author: winklera
 */

#include <xpp/zmp/zmp_constraint_builder.h>

namespace xpp {
namespace zmp {

ZmpConstraintBuilder::ZmpConstraintBuilder(const ContinuousSplineContainer& spline_container, double walking_height)
{
  Init(spline_container, walking_height);
}

void
ZmpConstraintBuilder::Init(const ContinuousSplineContainer& spline_container, double walking_height)
{
  spline_structure_ = spline_container;

  using namespace xpp::utils::coords_wrapper;
  x_zmp_map_ = ZeroMomentPoint::ExpressZmpThroughCoefficients(spline_structure_, walking_height, X);
  y_zmp_map_ = ZeroMomentPoint::ExpressZmpThroughCoefficients(spline_structure_, walking_height, Y);

  initialized_ = true;
}

ZmpConstraintBuilder::MatVecVec
ZmpConstraintBuilder::CalcZmpConstraints(const SupportPolygonContainer& s) const
{
  CheckIfInitialized();
  return CalcZmpConstraints(x_zmp_map_, y_zmp_map_, s);
};

ZmpConstraintBuilder::MatVecVec
ZmpConstraintBuilder::CalcZmpConstraints(const MatVec& x_zmp, const MatVec& y_zmp,
                                  const SupportPolygonContainer& supp_polygon_container) const
{
  std::vector<NodeConstraint> supp_lines = supp_polygon_container.GetActiveConstraintsForEachStep(spline_structure_.GetSplines());

  // if every spline is a four leg support spline with 4 line constraints
  const int max_num_constraints = spline_structure_.GetTotalNodes()*SupportPolygon::kMaxSides;
  int coeff = spline_structure_.GetTotalFreeCoeff();
  MatVecVec ineq(max_num_constraints, coeff);

  int n = 0; // node counter
  int c = 0; // inequality constraint counter

  for (double t_global : spline_structure_.GetDiscretizedGlobalTimes()) {
    int id = spline_structure_.GetSplineID(t_global);

    if (DisjSuppSwitch(t_global, spline_structure_.GetSpline(id), supp_polygon_container)) {
      n++; // no constraints
      continue;
    }

    GenerateNodeConstraint(supp_lines.at(id), x_zmp.GetRow(n), y_zmp.GetRow(n), c, ineq);

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
  VecScalarScalar line_constr_sep;
  // separate the constraints that depend only on the start stance with the ones
  // that depend on the optimized footholds
  line_constr_sep.vs.v  = l.coeff.p*x_zmp.v + l.coeff.q*y_zmp.v;
  line_constr_sep.vs.s = 0;
  line_constr_sep.constant = -l.s_margin;

  double line_coeff_terms = l.coeff.p*x_zmp.s + l.coeff.q*y_zmp.s + l.coeff.r;
  if (l.fixed_by_start_stance)
    line_constr_sep.constant = line_coeff_terms;
  else
    line_constr_sep.vs.s = line_coeff_terms;

  return line_constr_sep;
}

bool
ZmpConstraintBuilder::DisjSuppSwitch (double t, const ZmpSpline& curr_spline,
                                      const SupportPolygonContainer& supp_polygon_container) const
{
  if (!curr_spline.IsFourLegSupport()) {

    int step = curr_spline.GetCurrStep();
    double t_local = spline_structure_.GetLocalTime(t);
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

void
ZmpConstraintBuilder::CheckIfInitialized() const
{
  if (!initialized_) {
    throw std::runtime_error("ZmpConstraintBuilder not initialized. Call Init() first");
  }
}

} /* namespace zmp */
} /* namespace xpp */

