/*
 * zmp_constraint.cc
 *
 *  Created on: Apr 4, 2016
 *      Author: winklera
 */

#include <xpp/zmp/zmp_constraint.h>

namespace xpp {
namespace zmp {

ZmpConstraint::ZmpConstraint (xpp::zmp::ContinuousSplineContainer spline_container)
{
  spline_container_ = spline_container;
}


std::vector<hyq::SupportPolygon>
ZmpConstraint::CreateSupportPolygonsWith4LS(const xpp::hyq::SupportPolygonContainer& supp_polygon_container) const
{
  std::vector<SupportPolygon> supp;
  std::vector<SupportPolygon> supp_no_4l = supp_polygon_container.GetSupportPolygons();

  for (const xpp::zmp::ZmpSpline& s : spline_container_.splines_)
  {
    bool first_spline = (s.id_ == spline_container_.splines_.front().id_);
    bool last_spline  = (s.id_ == spline_container_.splines_.back().id_);

    if (first_spline)
      supp.push_back(supp_polygon_container.GetStartPolygon());
    else if (last_spline)
      supp.push_back(supp_polygon_container.GetFinalPolygon());
    else if (s.four_leg_supp_)
      supp.push_back((SupportPolygon::CombineSupportPolygons(supp_no_4l.at(s.step_), supp_no_4l.at(s.step_-1))));
    else
      supp.push_back(supp_no_4l.at(s.step_));

  }
  return supp;
}



ZmpConstraint::MatVec
ZmpConstraint::AddLineConstraints(const MatVec& x_zmp, const MatVec& y_zmp,
                                  const xpp::hyq::SupportPolygonContainer& supp_polygon_container) const
{
  int coeff = spline_container_.GetTotalFreeCoeff();

  int num_nodes_no_4ls = spline_container_.GetTotalNodesNo4ls();
  int num_nodes_4ls = spline_container_.GetTotalNodes4ls();
  int num_ineq_constr = 3*num_nodes_no_4ls + 4*num_nodes_4ls;

  MatVec ineq(num_ineq_constr, coeff);

  int n = 0; // node counter
  int c = 0; // inequality constraint counter

  std::vector<SupportPolygon> supp = CreateSupportPolygonsWith4LS(supp_polygon_container);

  for (const zmp::ZmpSpline& s : spline_container_.splines_)
  {
    SupportPolygon::VecSuppLine lines = supp.at(s.id_).CalcLines();
    for (double i=0; i < s.GetNodeCount(spline_container_.dt_); ++i) {
      for (SupportPolygon::SuppLine l : lines) { // add three line constraints for each node
        VecScalar constr = GenerateLineConstraint(l, x_zmp.ExtractRow(n), y_zmp.ExtractRow(n));
        ineq.AddVecScalar(constr,c++);
      }
      n++;
    }
  }
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
  line_constr.s  = l.coeff.p*x_zmp.s + l.coeff.q*x_zmp.s;
  line_constr.s += l.coeff.r - l.s_margin;

  return line_constr;
}




} /* namespace zmp */
} /* namespace xpp */


