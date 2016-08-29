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
#include <xpp/utils/line_equation.h>

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
    utils::LineEquation line;
//    utils::LineCoeff2d coeff;
    double s_margin;
    bool fixed_by_start_stance;
  };

  typedef xpp::utils::LineEquation LineEquation;
  typedef std::vector<SuppLine> VecSuppLine;
  typedef std::vector<Foothold> VecFoothold;
  typedef Eigen::Vector2d Vector2d;

  enum MaxSidesType   { kMaxSides = 4 };

public:
  SupportPolygon() {};
  SupportPolygon(const VecFoothold& footholds, const MarginValues& margins = GetZeroMargins());
  virtual ~SupportPolygon() {};

  VecSuppLine CalcLines() const;
  static MarginValues GetDefaultMargins();
  static MarginValues GetZeroMargins();

  bool IsPointInside(const Vector2d& p) const;


  VecFoothold footholds_; // all the contact points for this support polygon
  MarginValues margins_;
  VecFoothold footholds_conv_; // only the convex footholds if some are not relevant for support polygon


  static SupportPolygon CombineSupportPolygons(const SupportPolygon& p1,
                                               const SupportPolygon& p2);
private:

  VecFoothold BuildSortedConvexHull(const VecFoothold& footholds) const;
  double UseMargin(const LegID& f0, const LegID& f1) const;
  friend std::ostream& operator<<(std::ostream& out, const SupportPolygon& tr);
};

//inline std::ostream& operator<<(std::ostream& out, const SupportPolygon::SuppLine& line)
//{
//  out << "line:"
//      << "\tcoeff: p="  << line.coeff.p << ", q=" << line.coeff.q << ", r=" << line.coeff.r
//      << ", margin: " << line.s_margin;
//
//  return out;
//}

inline std::ostream& operator<<(std::ostream& out, const SupportPolygon& tr)
{
  out <<"margins[front,hind,side,diag]=" << tr.margins_[0] << ", " << tr.margins_[1] << ", " << tr.margins_[2] << ", " << tr.margins_[3] << "\n";
  for (const Foothold& f : tr.footholds_conv_)
    out  << f << "\n";
  return out;
}

} // namespace hyq
} // namespace xpp

#endif // _XPP_HYQ_SUPPORTPOLYGON_H_
