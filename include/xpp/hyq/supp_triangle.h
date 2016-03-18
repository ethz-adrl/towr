/**
@file    supp_triangle.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Defines a triangle created by footholds that affects stability.
 */

#ifndef _XPP_HYQ_SUPPTRIANGLE_H_
#define _XPP_HYQ_SUPPTRIANGLE_H_

#include <xpp/hyq/foothold.h>
#include <xpp/hyq/leg_data_map.h> // LegID, LegIDArray
#include <xpp/utils/geometric_structs.h>

#include <log4cxx/logger.h>
#include <array>
#include <vector>

namespace xpp {
namespace hyq {

static const int kSideTypeCount = 4;
enum SideTypes {FRONT = 0, HIND, SIDE, DIAG};

typedef std::array<double, kSideTypeCount> MarginValues;

/**
@brief Defines a triangle created by footholds that affects stability.
 */
class SuppTriangle {
public:
  struct TrLine {
    utils::LineCoeff2d coeff;
    double s_margin;
  };
  typedef std::array<TrLine,3> TrLines3;
  typedef std::array<Foothold,3> ArrayF3;
  typedef std::vector<Foothold> Footholds;

public:
  SuppTriangle();
  SuppTriangle(const MarginValues& margins, LegID swing_leg, const ArrayF3& footholds);
  virtual ~SuppTriangle();

  TrLines3 CalcLines() const;
  const ArrayF3& GetFootholds() const;

  MarginValues margins_;
  LegID swing_leg_;
  ArrayF3 footholds_;
private:

  double UseMargin(const LegID& f0, const LegID& f1) const;
  void SortFootholdsCounterclockwise();
  static bool SortFootholdByXThenY(const Foothold& lhs, const Foothold& rhs);
  double Foothold2isRightOfLine(const Foothold& f0, const Foothold f1,
                                const Foothold& f2) const;

  static log4cxx::LoggerPtr log_;
  static log4cxx::LoggerPtr log_matlab_;

};

typedef std::vector<SuppTriangle> SuppTriangles;



inline std::ostream& operator<<(std::ostream& out, const SuppTriangle& tr)
{
  out <<"margins[front,hind,side,diag]=" << tr.margins_[0] << ", " << tr.margins_[1] << ", " << tr.margins_[2] << ", " << tr.margins_[3] << "\n";
  for (const Foothold& f : tr.GetFootholds())
    out  << f << "\n";
  return out;
}

} // namespace hyq
} // namespace xpp

#endif // _XPP_HYQ_SUPPTRIANGLE_H_
