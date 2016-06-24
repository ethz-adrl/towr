/**
@file   zmp_spline.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2014
@brief  Spline created by the zmp optimizaton and added to SplineContainer.
 */

#ifndef _XPP_ZMP_SPLINE_H_
#define _XPP_ZMP_SPLINE_H_

#include <xpp/utils/geometric_structs.h>

// for friend class declaration
namespace xpp {
namespace ros {
class RosHelpers;
}
}


namespace xpp {
namespace zmp {


static const int kCoeffCount = 6;
enum SplineCoeff { A=0, B, C, D }; // the coefficients that are optimized over
enum SplineCoeffE { E=D+1};
enum SplineCoeffF { F=D+2};



struct CoeffValues {
  double x[kCoeffCount];
  double y[kCoeffCount];
  CoeffValues()
  {
    for (int c = A; c <= F; ++c)
      x[c] = y[c] = 0.0;
  };

  CoeffValues(double xa, double xb, double xc, double xd, double xe, double xf,
              double ya, double yb, double yc, double yd, double ye, double yf)
  {
    x[A] = xa; x[B] = xb; x[C] = xc; x[D] = xd; x[E] = xe; x[F] = xf;
    y[A] = ya; y[B] = yb; y[C] = yc; y[D] = yd; y[E] = ye; y[F] = yf;
  }

  /** generates random spline coefficients between -25 and 25 */
  void SetRandom()
  {
    for (int c = A; c <= F; ++c) {
      x[c] = (double)rand() / RAND_MAX * 50 - 25;
      y[c] = (double)rand() / RAND_MAX * 50 - 25;
    }
  }

//  bool operator==(const CoeffValues& rhs) const
//  {
//    static const double eps = std::numeric_limits<double>::epsilon();
//
//    for (int c = A; c <= F; ++c) {
//      bool x_equal = std::abs(x[c] - rhs.x[c]) <= eps * std::abs(x[c]);
//      bool y_equal = std::abs(y[c] - rhs.y[c]) <= eps * std::abs(y[c]);
//
//      if (!x_equal || !y_equal)
//        return false;
//    }
//    return true;
//  }
};


// todo rename to fifth order polynome!

/**
@class Spline
@brief fully represents a spline in 2d and allows retrieving
       values at specific time instances
*/
class Spline {

public:
  typedef xpp::utils::Vec2d Vec2d;
  typedef xpp::utils::PosVelAcc PosVelAcc;

  static const int kDim2d = xpp::utils::kDim2d;

public:
  Spline();
  Spline(const CoeffValues &coeff_values);
  virtual ~Spline() {};

  Vec2d GetState(PosVelAcc whichDeriv, double t) const;
  void SetSplineCoefficients(const CoeffValues &coeff_values = CoeffValues());
  void SetXCoefficient(int coeff, double val) { spline_coeff_[xpp::utils::X][coeff] = val; } ;
  void SetYCoefficient(int coeff, double val) { spline_coeff_[xpp::utils::Y][coeff] = val; } ;

protected:
  double spline_coeff_[kDim2d][kCoeffCount];
  friend class SplineContainerTest_EandFCoefficientTest_Test;
};



/**
@class ZmpSpline
@brief Extends a general spline by specifying a duration during which it is
       active in creating the spline for the CoG movement.
*/
enum ZmpSplineType {StanceSpline=0, StepSpline};
class ZmpSpline : public Spline
{
public:
  ZmpSpline();
  ZmpSpline(uint id, double duration, ZmpSplineType);
  virtual ~ZmpSpline() {};

  uint GetId()            const { return id_; };
  double GetDuration()    const { return duration_; }
  void SetStep(int step) {step_ = step; };

  /** Only if spline is a "StepSpline" is a step currently being executed. */
  uint GetCurrStep() const;

  bool IsFourLegSupport() const { return type_ == StanceSpline; }

private:
  uint id_; // to identify the order relative to other zmp splines
  double duration_; // time during which this spline is active
  ZmpSplineType type_;
  int step_; // current step

  friend struct xpp::ros::RosHelpers;
  friend std::ostream& operator<<(std::ostream& out, const ZmpSpline& tr);
};


} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_SPLINE_H_
