/*
 * zmp_constraint.cc
 *
 *  Created on: Apr 4, 2016
 *      Author: winklera
 */

#include <xpp/zmp/zmp_constraint.h>


namespace xpp {
namespace zmp {

ZmpConstraint::ZmpConstraint()
{
}


ZmpConstraint::ZmpConstraint(const ContinuousSplineContainer& spline_container, double walking_height)
{
  Init(spline_container, walking_height);
}


void
ZmpConstraint::Init(const ContinuousSplineContainer& spline_container, double walking_height)
{
  spline_structure_ = spline_container;

  // these discretized zmp positions do not depend on current footholds
  using namespace xpp::utils::coords_wrapper;
  x_zmp_ = ZeroMomentPoint::ExpressZmpThroughCoefficients(spline_structure_, walking_height, X);
  y_zmp_ = ZeroMomentPoint::ExpressZmpThroughCoefficients(spline_structure_, walking_height, Y);

  initialized_ = true;
}


ZmpConstraint::MatVec
ZmpConstraint::CalcZmpConstraints(const MatVec& x_zmp, const MatVec& y_zmp,
                                  const SupportPolygonContainer& supp_polygon_container) const
{

  AktiveContraints supp_lines = supp_polygon_container.GetActiveConstraintsForEachStep(spline_structure_.GetSplines());


  // if every spline is a four leg support spline with 4 line constraints
  const int max_num_constraints = spline_structure_.GetTotalNodes()*SupportPolygon::kMaxSides;
  int coeff = spline_structure_.GetTotalFreeCoeff();
  MatVec ineq(max_num_constraints, coeff);


  int n = 0; // node counter
  int c = 0; // inequality constraint counter
  double t_global = 0.0;
  while (t_global < spline_structure_.GetTotalTime())
  {
    int spline = spline_structure_.GetSplineID(t_global);


    AddNodeConstraints(supp_lines.at(spline), x_zmp.GetRow(n), y_zmp.GetRow(n), c, ineq);


    n++;
    c += SupportPolygon::kMaxSides;
    t_global += spline_structure_.dt_; // fixme, this must be equal to x_zmp step
  }


  assert(c <= max_num_constraints);
  assert((n == x_zmp.M.rows()) && (n == y_zmp.M.rows()));
  return ineq;
}


void
ZmpConstraint::AddNodeConstraints(const NodeConstraints& node_constraints,
                                  const VecScalar& x_zmp,
                                  const VecScalar& y_zmp,
                                  int row_start,
                                  MatVec& ineq)
{
  // add three/four line constraints for each node
  for (SupportPolygon::SuppLine l : node_constraints) {
    VecScalar constr = GenerateLineConstraint(l, x_zmp, y_zmp);
    ineq.WriteRow(constr,row_start++);
  }
}


ZmpConstraint::VecScalar
ZmpConstraint::GenerateLineConstraint(const SupportPolygon::SuppLine& l,
                                      const VecScalar& x_zmp,
                                      const VecScalar& y_zmp)
{
  // the zero moment point must always lay on one side of triangle side:
  // p*x_zmp + q*y_zmp + r > stability_margin
  VecScalar line_constr;

  line_constr.v  = l.coeff.p*x_zmp.v + l.coeff.q*y_zmp.v;
  line_constr.s  = l.coeff.p*x_zmp.s + l.coeff.q*y_zmp.s;
  line_constr.s += l.coeff.r - l.s_margin;

  return line_constr;
}


void
ZmpConstraint::CheckIfInitialized() const
{
  if (!initialized_) {
    throw std::runtime_error("ZmpConstraint not initialized. Call Init() first");
  }
}




} /* namespace zmp */
} /* namespace xpp */


