/**
@file    support_polygon.h
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Defines a triangle created by footholds that affects stability.
 */

#ifndef _XPP_HYQ_SUPPORTPOLYGON_H_
#define _XPP_HYQ_SUPPORTPOLYGON_H_

#include <xpp/hyq/foothold.h>
#include <xpp/hyq/leg_data_map.h> // LegID, LegIDArray
#include <xpp/utils/geometric_structs.h>

#include <vector>

namespace xpp {
namespace hyq {

static const int kSideTypeCount = 4;
enum SideTypes {FRONT = 0, HIND, SIDE, DIAG};
typedef std::array<double, kSideTypeCount> MarginValues;

/**
@brief Defines a triangle created by footholds that affects stability.
 */
class SupportPolygon {
public:
  struct SuppLine {
    utils::LineCoeff2d coeff;
    double s_margin;
  };

  typedef std::vector<SuppLine> VecSuppLine;
  typedef std::vector<Foothold> VecFoothold;

public:
  SupportPolygon() {};
  SupportPolygon(const MarginValues& margins, const VecFoothold& footholds);
  virtual ~SupportPolygon() {};

  VecSuppLine CalcLines() const;
  const VecFoothold& GetFootholds() const;

  MarginValues margins_;
private:

  VecFoothold footholds_;
  void SortFootholdsCounterClockwise(const VecFoothold& footholds);
  double UseMargin(const LegID& f0, const LegID& f1) const;
};


inline std::ostream& operator<<(std::ostream& out, const SupportPolygon& tr)
{
  out <<"margins[front,hind,side,diag]=" << tr.margins_[0] << ", " << tr.margins_[1] << ", " << tr.margins_[2] << ", " << tr.margins_[3] << "\n";
  for (const Foothold& f : tr.GetFootholds())
    out  << f << "\n";
  return out;
}

} // namespace hyq
} // namespace xpp

#endif // _XPP_HYQ_SUPPORTPOLYGON_H_
