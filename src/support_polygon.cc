/**
@file    supp_polygon.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Defines a triangle created by footholds that affects stability
 */

#include <xpp/hyq/support_polygon.h>
#include <xpp/utils/point2d_manipulations.h>


namespace xpp {
namespace hyq {

using namespace ::xpp::utils; //X,Y,Z,Poin2dManip


SupportPolygon::SupportPolygon(const MarginValues& margins, const VecFoothold& footholds)
    :  margins_(margins),
       footholds_conv_(BuildSortedConvexHull(footholds))
{}


/** sort points so inequality constraints are on correct side of line later **/
SupportPolygon::VecFoothold
SupportPolygon::BuildSortedConvexHull(const VecFoothold& footholds) const
{
  Point2dManip::StdVectorEig2d f_xy;

  for (const Foothold& f : footholds)
    f_xy.push_back(f.p.segment<2>(0)); // extract x-y position of footholds

  std::vector<size_t> idx = Point2dManip::BuildConvexHullCounterClockwise(f_xy);

  VecFoothold footholds_sorted(idx.size());
  for (uint i=0; i<idx.size(); ++i) {
    footholds_sorted.at(i) = footholds.at(idx[i]);
  }

  return footholds_sorted;
}


SupportPolygon::VecSuppLine SupportPolygon::CalcLines() const
{
  VecSuppLine lines(footholds_conv_.size());
  for (uint i = 0; i<lines.size(); ++i) {
    Foothold from = footholds_conv_[i];
    int last_idx = footholds_conv_.size()-1;
    Foothold to = (i == last_idx) ? footholds_conv_[0] : footholds_conv_[i+1];
    lines[i].coeff = Point2dManip::LineCoeff(from.p.segment(0,2), to.p.segment(0,2));
    lines[i].s_margin = UseMargin(from.leg, to.leg);
  }

  return lines;
}


double SupportPolygon::UseMargin(const LegID& f0, const LegID& f1) const
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


} // namespace hyq
} // namespace xpp
