/*
 * zmp_constraint.cc
 *
 *  Created on: Apr 4, 2016
 *      Author: winklera
 */

#include <xpp/zmp/zmp_constraint.h>

namespace xpp {
namespace zmp {

ZmpConstraint::ZmpConstraint (xpp::zmp::ContinuousSplineContainer spline_container,
                              xpp::hyq::SupportPolygonContainer supp_polygon_container)
{
  spline_container_ = spline_container;
  supp_polygon_container_ = supp_polygon_container;
}

ZmpConstraint::~ZmpConstraint ()
{
  // TODO Auto-generated destructor stub
}



std::vector<hyq::SupportPolygon>
ZmpConstraint::CreateSupportPolygonsWith4LS() const
{
  std::vector<SupportPolygon> supp;
  std::vector<SupportPolygon> supp_no_4l = supp_polygon_container_.CreateSupportPolygons();

  for (const xpp::zmp::ZmpSpline& s : spline_container_.splines_)
  {
    bool first_spline = (s.id_ == spline_container_.splines_.front().id_);
    bool last_spline  = (s.id_ == spline_container_.splines_.back().id_);

    // TODO make get first and last stance a function
    if (first_spline)
      supp.push_back(supp_polygon_container_.GetStartPolygon());
    else if (last_spline)
      supp.push_back(supp_polygon_container_.GetFinalPolygon());
    else if (s.four_leg_supp_)
      supp.push_back((SupportPolygon::CombineSupportPolygons(supp_no_4l.at(s.step_), supp_no_4l.at(s.step_-1))));
    else
      supp.push_back(supp_no_4l.at(s.step_));

  }
  return supp;
}



ZmpConstraint::MatVec
ZmpConstraint::AddLineConstraints(const MatVec& x_zmp, const MatVec& y_zmp) const
{

  int coeff = spline_container_.GetTotalFreeCoeff();

  int num_nodes_no_4ls = spline_container_.GetTotalNodesNo4ls();
  int num_nodes_4ls = spline_container_.GetTotalNodes4ls();
  int num_ineq_constr = 3*num_nodes_no_4ls + 4*num_nodes_4ls;

  MatVec ineq(num_ineq_constr, coeff);

  int n = 0; // node counter
  int c = 0; // inequality constraint counter

  std::vector<SupportPolygon> supp = CreateSupportPolygonsWith4LS();

  for (const zmp::ZmpSpline& s : spline_container_.splines_)
  {
    SupportPolygon::VecSuppLine lines = supp.at(s.id_).CalcLines();
    for (double i=0; i < s.GetNodeCount(spline_container_.dt_); ++i) {
      for (SupportPolygon::SuppLine l : lines) // add three line constraints for each node
        AddLineConstraint(l, x_zmp.M.row(n), y_zmp.M.row(n),x_zmp.v[n], y_zmp.v[n], c, ineq);
      n++;
    }
  }
  return ineq;
}


void ZmpConstraint::AddLineConstraint(const SupportPolygon::SuppLine& l,
                                    const Eigen::RowVectorXd& x_zmp_M,
                                    const Eigen::RowVectorXd& y_zmp_M,
                                    double x_zmp_v,
                                    double y_zmp_v,
                                    int& c, MatVec& ineq)
{
  // the zero moment point must always lay on one side of triangle side:
  // p*x_zmp + q*y_zmp + r > stability_margin
  ineq.M.row(c) = l.coeff.p*x_zmp_M + l.coeff.q*y_zmp_M;
  ineq.v[c]     = l.coeff.p*x_zmp_v + l.coeff.q*x_zmp_v;
  ineq.v[c]    += l.coeff.r - l.s_margin;

  c++;
}




} /* namespace zmp */
} /* namespace xpp */


