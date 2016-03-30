/*
 * supp_triangle_container.cc
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#include <xpp/hyq/supp_triangle_container.h>

namespace xpp {
namespace hyq {

SuppTriangleContainer::SuppTriangleContainer ()
{
  // TODO Auto-generated constructor stub

}

SuppTriangleContainer::~SuppTriangleContainer ()
{
  // TODO Auto-generated destructor stub
}


void SuppTriangleContainer::Init(LegDataMap<Foothold> start_stance,
                                 const Footholds& footholds,
                                 const MarginValues& margins)
{
  start_stance_ = start_stance;
  footholds_    = footholds;
  margins_      = margins;

  initialized_ = true;

}


SuppTriangleContainer::SuppTriangles
SuppTriangleContainer::GetSupportTriangles(const Footholds& footholds) const
{
  CheckIfInitialized();

  SuppTriangles tr;
  LegDataMap<Foothold> curr_stance = start_stance_;

  for (std::size_t s = 0; s < footholds.size(); s++) {
    LegID swingleg = footholds[s].leg;
    ArrayF3 non_swing_legs;
    // extract the 3 non-swinglegs from stance
    for (LegID l : LegIDArray)
      if(curr_stance[l].leg != swingleg)
        non_swing_legs.push_back(curr_stance[l]);

    tr.push_back(SuppTriangle(margins_, swingleg, non_swing_legs));
    curr_stance[swingleg] = footholds.at(s); // update current stance with last step
  }

  return tr;
}


Eigen::Vector2d SuppTriangleContainer::GetCenterOfFinalStance() const
{
  CheckIfInitialized();

  // get last support triangle + last step to form last stance
  ArrayF3 last_tr = GetSupportTriangles().back().footholds_; // FIXME violates law of Dementer
  Foothold last_step = footholds_.back();

  Eigen::Vector2d end_cog = Eigen::Vector2d::Zero();
  for (int i=0; i<3; ++i) {
    end_cog += last_tr[i].p.segment<2>(xpp::utils::X);
  }
  end_cog += last_step.p.segment<2>(xpp::utils::X);

  return end_cog/_LEGS_COUNT;
}


SuppTriangleContainer::MatVec
SuppTriangleContainer::AddLineConstraints(const MatVec& x_zmp, const MatVec& y_zmp,
                                          const xpp::zmp::ContinuousSplineContainer& zmp_splines) const
{
  CheckIfInitialized();

  SuppTriangles supp_triangles = GetSupportTriangles();

  int coeff = zmp_splines.GetTotalFreeCoeff();
  int num_nodes_no_4ls = zmp_splines.GetTotalNodes(true);

  int num_ineq_constr = 3*num_nodes_no_4ls;
  MatVec ineq(num_ineq_constr, coeff);

  int n = 0; // node counter
  int c = 0; // inequality constraint counter
  for (const xpp::zmp::ZmpSpline& s : zmp_splines.splines_) {

    // skip the nodes that belong to the four-leg support phase
    if (s.four_leg_supp_) {
      n += s.GetNodeCount(zmp_splines.dt_);
      continue;
    }

    SuppTriangle::TrLines3 lines = supp_triangles.at(s.step_).CalcLines();
    for (double i=0; i < s.GetNodeCount(zmp_splines.dt_); ++i) {

      // add three line constraints for each node
      for (int l=0; l<3; l++) {
        AddLineConstraint(lines.at(l), x_zmp.M.row(n), y_zmp.M.row(n),
                                       x_zmp.v[n],     y_zmp.v[n],
                                       c, ineq);
      }
      n++;
    }
  }

  return ineq;
}



void SuppTriangleContainer::AddLineConstraint(const SuppTriangle::TrLine& l,
                                    const Eigen::RowVectorXd& x_zmp_M,
                                    const Eigen::RowVectorXd& y_zmp_M,
                                    double x_zmp_v,
                                    double y_zmp_v,
                                    int& c, MatVec& ineq) const
{
  // the zero moment point must always lay on one side of triangle side:
  // p*x_zmp + q*y_zmp + r > stability_margin
  ineq.M.row(c) = l.coeff.p*x_zmp_M + l.coeff.q*y_zmp_M;
  ineq.v[c]     = l.coeff.p*x_zmp_v + l.coeff.q*x_zmp_v;
  ineq.v[c]    += l.coeff.r - l.s_margin;

  c++;
}


void
SuppTriangleContainer::CheckIfInitialized() const
{
  if (!initialized_) {
    throw std::runtime_error("SuppTriangleContainer not initialized. Call Init() first");
  }
}




} /* namespace hyq */
} /* namespace xpp */
