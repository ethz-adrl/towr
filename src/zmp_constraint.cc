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
ZmpConstraint::CreateLineConstraints(const SupportPolygonContainer& supp_polygon_container) const
{
  CheckIfInitialized();
  return AddLineConstraints(x_zmp_, y_zmp_, supp_polygon_container);
}


ZmpConstraint::MatVec
ZmpConstraint::AddLineConstraints(const MatVec& x_zmp, const MatVec& y_zmp,
                                  const SupportPolygonContainer& supp_polygon_container) const
{
  auto CreateSuppPoly = &hyq::SupportPolygonContainer::CreateSupportPolygonsWith4LS; // alias
  int coeff = spline_structure_.GetTotalFreeCoeff();

  int num_nodes_no_4ls = spline_structure_.GetTotalNodesNo4ls();
  int num_nodes_4ls = spline_structure_.GetTotalNodes4ls();
  int num_ineq_constr = 3*num_nodes_no_4ls + 4*num_nodes_4ls; // upper limit, not accurate

  MatVec ineq(num_ineq_constr, coeff);

  int n = 0; // node counter
  int c = 0; // inequality constraint counter

  std::vector<SupportPolygon> supp = CreateSuppPoly(supp_polygon_container,
                                                    spline_structure_.GetSplines());

//  std::cout << "IN ZMPCONSTRAINT: Support polygons for the " << supp.size() << " steps:\n";
//  for (const hyq::SupportPolygon& s : supp) {
//    std::cout << s;
//  }

  for (const zmp::ZmpSpline& s : spline_structure_.GetSplines())
  {
    SupportPolygon::VecSuppLine lines = supp.at(s.GetId()).CalcLines();


//    Eigen::VectorXd lp(lines.size());
//    Eigen::VectorXd lq(lines.size());
//    Eigen::VectorXd lr(lines.size());
//    Eigen::VectorXd ls(lines.size());
//
//    for (int i=0; i<lines.size(); ++i){
//      lp[i] = lines.at(i).coeff.p;
//      lq[i] = lines.at(i).coeff.q;
//      lr[i] = lines.at(i).coeff.r;
//      ls[i] = lines.at(i).s_margin;
//    }

    for (double i=0; i < s.GetNodeCount(spline_structure_.dt_); ++i) {
      // FIXME: Optimize by performing one matrix*vector multiplication


//      const VecScalar& x_ = x_zmp.ExtractRow(n);
//      const VecScalar& y_ = y_zmp.ExtractRow(n);
//
//      ineq.M.middleRows(c,lines.size()) = lp*x_.v + lq*y_.v;
//      ineq.v.segment(c,lines.size())    = lp*x_.s - lq*y_.s + lr - ls;
//      c += lines.size();

      for (SupportPolygon::SuppLine l : lines) { // add three/four line constraints for each node
        VecScalar constr = GenerateLineConstraint(l, x_zmp.GetRow(n), y_zmp.GetRow(n));
        ineq.WriteRow(constr,c++);
      }


      n++;
    }
  }

  assert((n == x_zmp.M.rows()) && (n == y_zmp.M.rows()));
  assert(c <= num_ineq_constr);
  return ineq;
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


