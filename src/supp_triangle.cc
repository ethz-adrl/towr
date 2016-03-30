/**
@file    supp_triangle.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Defines a triangle created by footholds that affects stability
 */

#include <xpp/hyq/supp_triangle.h>
#include <xpp/utils/point2d_manipulations.h>


namespace xpp {
namespace hyq {

using namespace ::xpp::utils; //X,Y,Z,Poin2dManip


SuppTriangle::SuppTriangle(const MarginValues& margins, LegID swing_leg, const ArrayF3& footholds)
    :  margins_(margins),
       swing_leg_(swing_leg),
       footholds_(footholds)
{
  // sort points so inequality constraints are on correct side of line later
  Point2dManip::StdVectorEig2d f_xy;
  for (int i=0; i<footholds_.size(); ++i) {
    f_xy.push_back(footholds_.at(i).p.segment<2>(0)); // extract x-y position of footholds
  }

  Point2dManip::CounterClockwiseSort(f_xy);

  for (int i=0; i<f_xy.size(); ++i) {
    footholds_.at(i).p.segment<2>(0) = f_xy.at(i);
  }
}


SuppTriangle::TrLines3 SuppTriangle::CalcLines() const
{
  TrLines3 lines(footholds_.size());
  for (uint i = 0; i<lines.size(); ++i) {
    Foothold from = footholds_[i];
    int last_idx = footholds_.size()-1;
    Foothold to = (i == last_idx) ? footholds_[0] : footholds_[i+1];
    lines[i].coeff = Point2dManip::LineCoeff(from.p.segment(0,2), to.p.segment(0,2));
    lines[i].s_margin = UseMargin(from.leg, to.leg);
  }

  return lines;
}


double SuppTriangle::UseMargin(const LegID& f0, const LegID& f1) const
{
  LegID foot[] = {f0, f1};

  bool is_left[2], is_right[2], is_front[2], is_hind[2];
  for (uint f = 0; f < 2; ++f) {
    is_left[f]  = (foot[f] == LF || foot[f] == LH);
    is_right[f] = (foot[f] == RF || foot[f] == RH);
    is_front[f] = (foot[f] == LF || foot[f] == RF);
    is_hind[f]  = (foot[f] == LH || foot[f] == RH);
  }

  if (is_front[0] && is_front[1])
    return margins_[FRONT];
  else if (is_hind[0] && is_hind[1])
    return margins_[HIND];
  else if ((is_left[0] && is_left[1]) || (is_right[0] && is_right[1]))
    return margins_[SIDE];
  else
    return margins_[DIAG];
}


const SuppTriangle::ArrayF3& SuppTriangle::GetFootholds() const
{
  return footholds_;
}


} // namespace hyq
} // namespace xpp
