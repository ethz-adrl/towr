/**
@file   zmp_spline.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2014
@brief  Spline created by the zmp optimizaton and added to SplineContainer.
 */

#ifndef _XPP_ZMP_SPLINE_H_
#define _XPP_ZMP_SPLINE_H_

#include <xpp/utils/geometric_structs.h>

#include <log4cxx/logger.h>

namespace xpp {
namespace zmp {

static const int kCoeffCount = 6;
enum SplineCoeff { A=0, B, C, D, E, F };

static const int kDerivCount = 3;
enum PosVelAcc { kPos=0, kVel, kAcc };

static const int kDim2d = 2; // X,Y

struct CoeffValues {
  double x[kCoeffCount];
  double y[kCoeffCount];
  CoeffValues()
  {
    for (int c = A; c <= F; ++c)
      x[c] = y[c] = 0.0;
  };

  CoeffValues(int xa, int xb, int xc, int xd, int xe, int xf,
              int ya, int yb, int yc, int yd, int ye, int yf)
  {
    x[A] = xa; x[B] = xb; x[C] = xc; x[D] = xd; x[E] = xe; x[F] = xf;
    y[A] = ya; y[B] = yb; y[C] = yc; y[D] = yd; y[E] = ye; y[F] = yf;
  }
};

/**
@class Spline
@brief fully represents a spline in 2d and allows retrieving
       values at specific time instances
*/
class Spline {

public:
  typedef utils::Vec2d Vec2d;
public:
  Spline();
  Spline(const CoeffValues &coeff_values);
  virtual ~Spline();

  Vec2d GetState(const PosVelAcc &whichDeriv, const double &_t) const;
  void set_spline_coeff(const CoeffValues &coeff_values);

  static log4cxx::LoggerPtr log_;
private:
  double spline_coeff_[kDim2d][kCoeffCount];
};

/**
@class ZmpSpline
@brief Extends a general spline by specifying a duration during which it is
       active in creating the spline for the CoG movement.
*/
class ZmpSpline : public Spline {

public:
  ZmpSpline();
  ZmpSpline(const CoeffValues &coeff_values, double duration);
  virtual ~ZmpSpline();

  double Duration() const; ///< global time-span in which this spline is active

private:
  double duration_;
};


} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_SPLINE_H_
