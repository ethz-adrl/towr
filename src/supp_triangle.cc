/**
@file    supp_triangle.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Defines a triangle created by footholds that affects stability
 */

#include <xpp/hyq/supp_triangle.h>
#include <xpp/utils/logger_helpers-inl.h>
#include <xpp/utils/point2d_manipulations.h>

#include <algorithm> //std::sort

namespace xpp {
namespace hyq {

using namespace ::xpp::utils; //X,Y,Z,Poin2dManip

log4cxx::LoggerPtr SuppTriangle::log_(log4cxx::Logger::getLogger("xpp.hyq.supptriangle"));
log4cxx::LoggerPtr SuppTriangle::log_matlab_(log4cxx::Logger::getLogger("matlab"));

  // TODO Auto-generated constructor stub
SuppTriangle::SuppTriangle()
{
  // sort points so inequality constraints are on correct side of line later
  SortFootholdsCounterclockwise();
}


SuppTriangle::SuppTriangle(const ArrayF3& footholds, const MarginValues& margins, LegID swing_leg)
    :  margins_(margins), footholds_(footholds), swing_leg_(swing_leg)
{
  // sort points so inequality constraints are on correct side of line later
  SortFootholdsCounterclockwise();
}


SuppTriangle::~SuppTriangle() {
  // TODO Auto-generated destructor stub
}


SuppTriangles SuppTriangle::FromFootholds(LegDataMap<Foothold> stance,
                                          const Footholds& steps,
                                          const MarginValues& margins,
                                          LegDataMap<Foothold>& last_stance)
{
  SuppTriangles tr;
  ArrayF3 non_swing;

  for (std::size_t s = 0; s < steps.size(); s++) {
    LegID swingleg = steps[s].leg;

    // extract the 3 non-swinglegs from stance
    int i = 0;
    for (LegID l : LegIDArray)
      if(stance[l].leg != swingleg)
        non_swing[i++] = stance[l];

    tr.push_back(SuppTriangle(non_swing, margins, swingleg));
    stance[swingleg] = steps[s]; // update current stance with last step
  }

  last_stance = stance;

  ::xpp::utils::logger_helpers::print_triangles(tr, log_);
  ::xpp::utils::logger_helpers::PrintTriaglesMatlabInfo(tr, log_matlab_);
  return tr;
}


bool SuppTriangle::Insert4LSPhase(LegID prev, LegID next)
{
  // check for switching between disjoint support triangles.
  // the direction the robot is moving between triangles does not matter.
  if ((prev==LF && next==RH) || (prev==RF && next==LH)) return true;
  std::swap(prev, next);
  if ((prev==LF && next==RH) || (prev==RF && next==LH)) return true;

  return false;
}


SuppTriangle::TrLines3 SuppTriangle::CalcLines() const
{
  TrLines3 lines;
  for (uint i = 0; i<lines.size(); ++i) {
    Foothold from = footholds_[i];
    Foothold to = (i == 2) ? footholds_[0] : footholds_[i+1];
    lines[i].coeff = Point2dManip::LineCoeff(from.p.segment(0,2), to.p.segment(0,2));
    lines[i].s_margin = UseMargin(from.leg, to.leg);
  }

  LOG4CXX_TRACE(log_, "Calclines():\n\tline 0: " << std::setprecision(3) << lines[0].coeff << ", m=" << lines[0].s_margin << "\n\tline 1: " <<  lines[1].coeff << ", m=" << lines[1].s_margin << "\n\tline 2: " <<  lines[2].coeff << ", m=" << lines[2].s_margin);
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


// TODO: move these functions in a separat foothold class
void SuppTriangle::SortFootholdsCounterclockwise()
{
  // make sure the first foothold is the one with lowest (x,y) value
  std::sort(footholds_.begin(), footholds_.end(), SortFootholdByXThenY);

  // sort footholds counterclockwise
  for (size_t i = 1; i < footholds_.size() - 1; i++) {
    for (size_t j = i + 1; j < footholds_.size(); j++) {
      if (Foothold2isRightOfLine(footholds_[0], footholds_[i], footholds_[j])  > 0.0) {
        Foothold tmp = footholds_[i];
        footholds_[i] = footholds_[j];
        footholds_[j] = tmp;
      }
    }
  }
}

double SuppTriangle::Foothold2isRightOfLine(const Foothold& f0, const Foothold f1,
                              const Foothold& f2) const
{
  return (f2.p(X) - f0.p(X)) * (f1.p(Y) - f0.p(Y))
       - (f1.p(X) - f0.p(X)) * (f2.p(Y) - f0.p(Y));
}

bool SuppTriangle::SortFootholdByXThenY(const Foothold& lhs, const Foothold& rhs)
{
  if (lhs.p(X) != rhs.p(X))
    return lhs.p(X) < rhs.p(X);
  else
    return lhs.p(Y) < rhs.p(Y);
}

} // namespace hyq
} // namespace xpp
