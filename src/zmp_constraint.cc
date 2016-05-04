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




  std::vector<SupportPolygon> supp;
  supp = SupportPolygonContainer::CreateSupportPolygonsWith4LS(supp_polygon_container,
                                                               spline_structure_.GetSplines());

  std::vector<SupportPolygon::VecSuppLine> supp_lines;
  const int max_num_constraints = spline_structure_.GetTotalNodes()*SupportPolygon::kMaxSides;




  // FIXME this has to be exact number of constraints, otherwise solution is very different!!!!
  for (int s=0; s<spline_structure_.GetSplineCount(); ++s) {
    SupportPolygon::VecSuppLine lines = supp.at(s).CalcLines();
    supp_lines.push_back(lines);
    // add one, because that could be the maximum off for rounding errors
//    max_num_constraints += (spline_structure_.GetNodeCount(s))*4;//lines.size();
  }


//  std::cout << "num_constraint: " << num_constraints << std::endl;

  int coeff = spline_structure_.GetTotalFreeCoeff();
  MatVec ineq(max_num_constraints, coeff);





  int n = 0; // node counter
  int c = 0; // inequality constraint counter
  double t_global = 0.0;
  while (t_global < spline_structure_.GetTotalTime())
  {
    int spline = spline_structure_.GetSplineID(t_global);
//    std::cout << "spline: " << spline << std::endl;

    for (SupportPolygon::SuppLine l : supp_lines.at(spline)) { // add three/four line constraints for each node
//      std::cout << l << std::endl;
      VecScalar constr = GenerateLineConstraint(l, x_zmp.GetRow(n), y_zmp.GetRow(n));
      ineq.WriteRow(constr,c++);
    }
    // skip the remaining lines
    int n_lines = supp_lines.at(spline).size();
    c += SupportPolygon::kMaxSides-n_lines;


    t_global += spline_structure_.dt_; // fixme, this must be equal to x_zmp step
    n++;

  }


  assert(c <= max_num_constraints);
//  ineq.M.conservativeResize(c, Eigen::NoChange);
//  ineq.v.conservativeResize(c);

//  std::cout << "c = " << c << std::endl;

  assert((n == x_zmp.M.rows()) && (n == y_zmp.M.rows()));
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


