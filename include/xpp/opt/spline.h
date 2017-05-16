/**
@file    spline.h
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2015
@brief   For manipulation of multiple sequential polynomials.
 */

#ifndef _XPP_OPT_SPLINE_H_
#define _XPP_OPT_SPLINE_H_

#include <memory>
#include <vector>

#include <xpp/state.h>

namespace xpp {
namespace opt {

/** Spans over a time t0->T, so spline.cc can work with it.
  */
class Segment {
public:
  Segment() {};
  virtual ~Segment() {};

  virtual double GetDuration() const = 0;
  virtual StateLinXd GetPoint(const double t_local) const = 0;
};


/** @brief For manipulation of multiple sequential segments ("spline").
  */
class Spline {
public:
  using SegmentPtr     = std::shared_ptr<Segment>;
  using VecSegments    = std::vector<SegmentPtr>;

  /**
   * @brief Assigns the vector of segments to be used for calculations.
   * @param vec This vector must inherit from class Segment.
   */
  template<class DerivedClass>
  void SetSegmentsPtr(const std::vector<std::shared_ptr<DerivedClass> >& vec)
  {
    segments_ = VecSegments(vec.begin(), vec.end());
  };

  const StateLinXd GetPoint(double t_globals) const;
  int GetSegmentID(double t_global) const;
  double GetLocalTime(double t_global) const;
  double GetTotalTime() const;

private:
  VecSegments segments_;
};

} // namespace opt
} // namespace xpp

#endif // _XPP_OPT_SPLINE_H_
