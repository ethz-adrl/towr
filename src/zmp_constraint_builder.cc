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

ZmpConstraintBuilder::MatVec
ZmpConstraintBuilder::CalcZmpConstraints(const SupportPolygonContainer& s) const
{
  CheckIfInitialized();
  return CalcZmpConstraints(x_zmp_map_, y_zmp_map_, s);
};

ZmpConstraintBuilder::MatVec
ZmpConstraintBuilder::CalcZmpConstraints(const MatVec& x_zmp, const MatVec& y_zmp,
                                  const SupportPolygonContainer& supp_polygon_container) const
{
  std::vector<NodeConstraint> supp_lines = supp_polygon_container.GetActiveConstraintsForEachStep(spline_structure_.GetSplines());

  // if every spline is a four leg support spline with 4 line constraints
  const int max_num_constraints = spline_structure_.GetTotalNodes()*SupportPolygon::kMaxSides;
  int coeff = spline_structure_.GetTotalFreeCoeff();
  MatVec ineq(max_num_constraints, coeff);

  int n = 0; // node counter
  int c = 0; // inequality constraint counter
  for (double t_global : spline_structure_.GetDiscretizedGlobalTimes())
  {
    int id = spline_structure_.GetSplineID(t_global);




    // check if this spline needs a four leg support phase to go to next spline
    // skip constraint then
    if (id > 2 && id < spline_structure_.GetLastSpline().GetId()) { // not initial 4ls spline
      int step = spline_structure_.GetSpline(id).GetCurrStep();
      xpp::hyq::LegID swing_leg = supp_polygon_container.GetLegID(step);
      xpp::hyq::LegID swing_leg_prev = supp_polygon_container.GetLegID(step-1);

      bool insert_stance = xpp::hyq::SupportPolygonContainer::Insert4LSPhase(swing_leg_prev, swing_leg);
      double t_stance = 0.2; // this should replace the ros param "stance_time"
      if (insert_stance && spline_structure_.GetLocalTime(t_global) < t_stance) {
        n++;
        continue; // don't add constraint
      }
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
                                      MatVec& ineq)
{
  // add three or four line constraints depending on if support triange/ support polygon etc
  for (SupportPolygon::SuppLine l : node_constraints) {
    VecScalar constr = GenerateLineConstraint(l, x_zmp, y_zmp);
    ineq.WriteRow(constr,row_start++);
  }
}


ZmpConstraintBuilder::VecScalar
ZmpConstraintBuilder::GenerateLineConstraint(const SupportPolygon::SuppLine& l,
                                      const VecScalar& x_zmp,
                                      const VecScalar& y_zmp)
{
  VecScalar line_constr;

  line_constr.v  = l.coeff.p*x_zmp.v + l.coeff.q*y_zmp.v;
  line_constr.s  = l.coeff.p*x_zmp.s + l.coeff.q*y_zmp.s;
  line_constr.s += l.coeff.r - l.s_margin;

  return line_constr;
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


