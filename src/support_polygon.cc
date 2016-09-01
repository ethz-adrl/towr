/**
@file    supp_polygon.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Defines a triangle created by footholds that affects stability
 */

#include <xpp/hyq/support_polygon.h>
#include <xpp/utils/point2d_manipulations.h>
#include <xpp/utils/line_equation.h>
#include <algorithm>


namespace xpp {
namespace hyq {


using namespace ::xpp::utils; //X,Y,Z,Poin2dManip

SupportPolygon::SupportPolygon(const VecFoothold& footholds, const MarginValues& margins)
    :sorted_footholds_(footholds),
     margins_(margins)
{
  SortCounterclockWise(sorted_footholds_);
}

void
SupportPolygon::SortCounterclockWise (VecFoothold& footholds) const
{
  std::map<LegID, int> leg_order = { {LH,0} , {RH,1}, {RF,2}, {LF,3} };

  std::sort(footholds.begin(), footholds.end(),
            [&leg_order](Foothold f1, Foothold f2)
            { return leg_order[f1.leg] < leg_order[f2.leg];}
  );
}

SupportPolygon::VecSuppLine
SupportPolygon::GetLines() const
{
  VecSuppLine lines(sorted_footholds_.size());
  for (uint i = 0; i<lines.size(); ++i) {
    Foothold from = sorted_footholds_[i];
    int last_idx = sorted_footholds_.size()-1;
    Foothold to = (i == last_idx) ? sorted_footholds_[0] : sorted_footholds_[i+1];

    lines[i].from = from;
    lines[i].to = to;
    lines[i].s_margin = UseMargin(from.leg, to.leg);
  }

  return lines;
}

double
SupportPolygon::SuppLine::GetDistanceToPoint(const Vector2d& p) const
{
  LineEquation line(from.p.segment<2>(X), to.p.segment<2>(Y));
  return line.GetDistanceFromLine(p) - s_margin;
}

SupportPolygon::VecFoothold
SupportPolygon::GetFootholds () const
{
  return sorted_footholds_;
}

MarginValues
SupportPolygon::GetMargins () const
{
  return margins_;
}

double
SupportPolygon::UseMargin(const LegID& f0, const LegID& f1) const
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


SupportPolygon SupportPolygon::CombineSupportPolygons(const SupportPolygon& p1,
                                                      const SupportPolygon& p2)
{
  VecFoothold contacts;
  contacts.insert(contacts.end(), p1.sorted_footholds_.begin(), p1.sorted_footholds_.end());
  contacts.insert(contacts.end(), p2.sorted_footholds_.begin(), p2.sorted_footholds_.end());

  // compare leg ids and make sure the same footholds in not inserted twice
  std::sort(contacts.begin(), contacts.end(), [](Foothold f1, Foothold f2) {return f1.leg < f2.leg;});
  contacts.erase(std::unique(contacts.begin(), contacts.end()), contacts.end()); // removes adjacent duplicate and resizes vector

  return SupportPolygon(contacts, p1.margins_);
}

bool
SupportPolygon::IsPointInside (const Vector2d& p) const
{
  for (const auto& l : GetLines()) {
    LineEquation line(l.from.p.segment<2>(X), l.to.p.segment<2>(X));

    bool zmp_outside = line.GetDistanceFromLine(p) < l.s_margin;
    if(zmp_outside)
      return false;
  }

  return true;
}

MarginValues SupportPolygon::GetDefaultMargins()
{
  MarginValues default_margins;
  default_margins[FRONT] = 0.10;
  default_margins[HIND]  = 0.10;
  default_margins[SIDE]  = 0.10;
  default_margins[DIAG]  = 0.06; // controls sidesway motion 0.8 even better

  return default_margins;
}

MarginValues SupportPolygon::GetZeroMargins()
{
  MarginValues zero_margins;
  zero_margins[FRONT] = 0.0;
  zero_margins[HIND]  = 0.0;
  zero_margins[SIDE]  = 0.0;
  zero_margins[DIAG]  = 0.0;

  return zero_margins;
}

SupportPolygon::VecFoothold
SupportPolygon::BuildSortedConvexHull(const VecFoothold& footholds) const
{
  assert(footholds.size() > 2);
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

} // namespace hyq
} // namespace xpp
