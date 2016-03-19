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
  ArrayF3 non_swing_legs;
  LegDataMap<Foothold> curr_stance = start_stance_;

  for (std::size_t s = 0; s < footholds.size(); s++) {
    LegID swingleg = footholds[s].leg;

    // extract the 3 non-swinglegs from stance
    int i = 0;
    for (LegID l : LegIDArray)
      if(curr_stance[l].leg != swingleg)
        non_swing_legs[i++] = curr_stance[l];

    tr.push_back(SuppTriangle(margins_, swingleg, non_swing_legs));
    curr_stance[swingleg] = footholds[s]; // update current stance with last step
  }

  return tr;
}

Eigen::Vector2d SuppTriangleContainer::GetCenterOfFinalStance() const
{
  CheckIfInitialized();

  // get last support triangle + last step to form last stance
  ArrayF3 last_tr = GetSupportTriangles().back().footholds_;
  Foothold last_step = footholds_.back();

  Eigen::Vector2d end_cog = Eigen::Vector2d::Zero();
  for (int i=0; i<3; ++i) {
    end_cog += last_tr[i].p.segment<2>(xpp::utils::X);
  }
  end_cog += last_step.p.segment<2>(xpp::utils::X);

  return end_cog/_LEGS_COUNT;
}


std::vector<SuppTriangle::TrLine>
SuppTriangleContainer::LineForConstraint(const xpp::zmp::ContinuousSplineContainer& splines,
                                         double dt) const
{


  SuppTriangles supp_triangles = GetSupportTriangles();
  // calculate number of inequality contraints
  std::vector<SuppTriangle::TrLine> line_for_constraint;

  for (const xpp::zmp::ZmpSpline& s : splines.splines_)
  {
    if (s.four_leg_supp_) continue; // no constraints in 4ls phase

    SuppTriangle::TrLines3 lines = supp_triangles.at(s.step_).CalcLines();

    int n_nodes =  s.GetNodeCount(dt);
    for (int i=0; i<n_nodes; ++i) {
      line_for_constraint.push_back(lines.at(0));
      line_for_constraint.push_back(lines.at(1));
      line_for_constraint.push_back(lines.at(2));
    }
  }
  return line_for_constraint;
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
