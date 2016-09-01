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

/** Creates Support Lines from Footholds for HyQ.
  */
class SupportPolygon {
public:
  struct SuppLine {
    Foothold from, to;
    double s_margin;
  };

  using VecSuppLine = std::vector<SuppLine>;
  using VecFoothold = std::vector<Foothold>;
  using Vector2d = Eigen::Vector2d;

  SupportPolygon() {};
  SupportPolygon(const VecFoothold& footholds, const MarginValues& margins = GetZeroMargins());
  virtual ~SupportPolygon() {};

  /** sorts the footholds starting with LH -> RH -> RF -> LF
    *
    * This allows to create the support lines with correct direction f1->f2.
    * However, it assumes that HyQ will never stand in a way that the legs
    * are crossed (e.g support polygon flips).
    */
  void SortCounterclockWise(VecFoothold&) const;
  VecSuppLine GetLines() const;

  bool IsPointInside(const Vector2d& p) const;

  VecFoothold GetFootholds() const;
  MarginValues GetMargins() const;

  static MarginValues GetDefaultMargins();
  static MarginValues GetZeroMargins();
  static SupportPolygon CombineSupportPolygons(const SupportPolygon& p1,
                                               const SupportPolygon& p2);
private:
  VecFoothold sorted_footholds_;
  MarginValues margins_;

  /** sort points so inequality constraints are on correct side of line later **/
  VecFoothold BuildSortedConvexHull(const VecFoothold& footholds) const;
  double UseMargin(const LegID& f0, const LegID& f1) const;
  friend std::ostream& operator<<(std::ostream& out, const SupportPolygon& tr);
};

inline std::ostream& operator<<(std::ostream& out, const SupportPolygon& tr)
{
  out <<"margins[front,hind,side,diag]=" << tr.margins_[0] << ", " << tr.margins_[1] << ", " << tr.margins_[2] << ", " << tr.margins_[3] << "\n";
  for (const Foothold& f : tr.sorted_footholds_)
    out  << f << "\n";
  return out;
}

inline std::ostream& operator<<(std::ostream& out, const SupportPolygon::SuppLine& line)
{
  out << "line:"
      << "\tfrom: " << line.from
      << "\tto: "   << line.to
      << ", margin: " << line.s_margin;

  return out;
}

} // namespace hyq
} // namespace xpp

#endif // _XPP_HYQ_SUPPORTPOLYGON_H_
