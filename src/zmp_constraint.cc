/*
 * zmp_constraint.cc
 *
 *  Created on: Apr 4, 2016
 *      Author: winklera
 */

#include <xpp/zmp/zmp_constraint.h>


namespace xpp {
namespace zmp {

ZmpConstraint::ZmpConstraint (ContinuousSplineContainer spline_container, double walking_height)
{
  spline_structure_ = spline_container;

  // these discretized zmp positions do not depend on current footholds
  x_zmp_ = ExpressZmpThroughCoefficients(walking_height, xpp::utils::X);
  y_zmp_ = ExpressZmpThroughCoefficients(walking_height, xpp::utils::Y);
}


ZmpConstraint::MatVec
ZmpConstraint::CreateLineConstraints(const SupportPolygonContainer& supp_polygon_container) const
{
  return AddLineConstraints(x_zmp_, y_zmp_, supp_polygon_container);
}


ZmpConstraint::Vector2d
ZmpConstraint::CalcZmp(const State3d& cog, double height)
{
  const double g = 9.81; // gravity acceleration
  double z_acc = cog.a.z();

  Vector2d zmp;
  zmp.x() = cog.p.x() - height/(g+z_acc) * cog.a.x();
  zmp.y() = cog.p.y() - height/(g+z_acc) * cog.a.y();

  return zmp;
}


ZmpConstraint::MatVec
ZmpConstraint::ExpressZmpThroughCoefficients(double walking_height, int dim) const
{
  int coeff = spline_structure_.GetTotalFreeCoeff();
  int num_nodes = spline_structure_.GetTotalNodes4ls() + spline_structure_.GetTotalNodesNo4ls();

  double h = walking_height;

  MatVec zmp(num_nodes, coeff);

  const double g = 9.81; // gravity acceleration
  const double z_acc = 0.0; // TODO: calculate z_acc based on foothold height

  int n = 0; // node counter
  for (const ZmpSpline& s : spline_structure_.GetSplines()) {

    // calculate e and f coefficients from previous values
    const int k = s.id_;
    VecScalar Ek = spline_structure_.GetCoefficient(k, dim, E);
    VecScalar Fk = spline_structure_.GetCoefficient(k, dim, F);
    int a = ContinuousSplineContainer::Index(k,dim,A);
    int b = ContinuousSplineContainer::Index(k,dim,B);
    int c = ContinuousSplineContainer::Index(k,dim,C);
    int d = ContinuousSplineContainer::Index(k,dim,D);

    for (double i=0; i < s.GetNodeCount(spline_structure_.dt_); ++i) {

      double time = i*spline_structure_.dt_;
      std::array<double,6> t = utils::cache_exponents<6>(time);

      //  x_zmp = x_pos - height/(g+z_acc) * x_acc
      //      with  x_pos = at^5 +   bt^4 +  ct^3 + dt*2 + et + f
      //            x_acc = 20at^3 + 12bt^2 + 6ct   + 2d
      zmp.M(n, a)   = t[5]        - h/(g+z_acc) * 20.0 * t[3];
      zmp.M(n, b)   = t[4]        - h/(g+z_acc) * 12.0 * t[2];
      zmp.M(n, c)   = t[3]        - h/(g+z_acc) *  6.0 * t[1];
      zmp.M(n, d)   = t[2]        - h/(g+z_acc) *  2.0;
      zmp.M.row(n) += t[1]*Ek.v;
      zmp.M.row(n) += t[0]*Fk.v;

      zmp.v[n] = Ek.s*t[0] + Fk.s;

      ++n;
    }
  }

  assert(n<=num_nodes); // check that Eigen matrix didn't overflow
  return zmp;
}


std::vector<hyq::SupportPolygon>
ZmpConstraint::CreateSupportPolygonsWith4LS(const SupportPolygonContainer& supp_polygon_container) const
{
  std::vector<SupportPolygon> supp;
  std::vector<SupportPolygon> supp_no_4l = supp_polygon_container.GetSupportPolygons();


  for (const xpp::zmp::ZmpSpline& s : spline_structure_.GetSplines())
  {
    bool first_spline = (s.id_ == spline_structure_.GetFirstSpline().id_);
    bool last_spline  = (s.id_ == spline_structure_.GetLastSpline().id_);

    if (s.four_leg_supp_)
      if (first_spline)
        supp.push_back(supp_polygon_container.GetStartPolygon());
      else if (last_spline)
        supp.push_back(supp_polygon_container.GetFinalPolygon());
      else
        supp.push_back((SupportPolygon::CombineSupportPolygons(supp_no_4l.at(s.step_), supp_no_4l.at(s.step_-1))));
    else
      supp.push_back(supp_no_4l.at(s.step_));
  }

  return supp;
}


ZmpConstraint::MatVec
ZmpConstraint::AddLineConstraints(const MatVec& x_zmp, const MatVec& y_zmp,
                                  const SupportPolygonContainer& supp_polygon_container) const
{
  int coeff = spline_structure_.GetTotalFreeCoeff();

  int num_nodes_no_4ls = spline_structure_.GetTotalNodesNo4ls();
  int num_nodes_4ls = spline_structure_.GetTotalNodes4ls();
  int num_ineq_constr = 3*num_nodes_no_4ls + 4*num_nodes_4ls; // upper limit, not accurate

  MatVec ineq(num_ineq_constr, coeff);

  int n = 0; // node counter
  int c = 0; // inequality constraint counter

  std::vector<SupportPolygon> supp = CreateSupportPolygonsWith4LS(supp_polygon_container);

  for (const zmp::ZmpSpline& s : spline_structure_.GetSplines())
  {
    SupportPolygon::VecSuppLine lines = supp.at(s.id_).CalcLines();


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
        VecScalar constr = GenerateLineConstraint(l, x_zmp.ExtractRow(n), y_zmp.ExtractRow(n));
        ineq.AddVecScalar(constr,c++);
      }


      n++;
    }
  }

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




} /* namespace zmp */
} /* namespace xpp */


