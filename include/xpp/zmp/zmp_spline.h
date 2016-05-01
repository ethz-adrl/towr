/**
@file   zmp_spline.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2014
@brief  Spline created by the zmp optimizaton and added to SplineContainer.
 */

#ifndef _XPP_ZMP_SPLINE_H_
#define _XPP_ZMP_SPLINE_H_

#include <xpp/utils/geometric_structs.h>


namespace xpp {
namespace ros {
class RosHelpers;
}
}


namespace xpp {
namespace zmp {


static const int kCoeffCount = 6;
enum SplineCoeff { A=0, B, C, D, E, F };

static const int kDerivCount = 3;
enum PosVelAcc { kPos=0, kVel, kAcc };


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
  typedef xpp::utils::Vec2d Vec2d;
  static const int kDim2d = xpp::utils::kDim2d;

public:
  Spline();
  Spline(const CoeffValues &coeff_values);
  virtual ~Spline();

  Vec2d GetState(const PosVelAcc &whichDeriv, const double &_t) const;
  void set_spline_coeff(const CoeffValues &coeff_values = CoeffValues());

  double spline_coeff_[kDim2d][kCoeffCount]; // FIXME, make privat
private:
};



/**
@class ZmpSpline
@brief Extends a general spline by specifying a duration during which it is
       active in creating the spline for the CoG movement.
*/
enum ZmpSplineType {Initial4lsSpline, StepSpline, Intermediate4lsSpline, Final4lsSpline};
class ZmpSpline : public Spline {

public:
  ZmpSpline();
  ZmpSpline(uint id, double duration, ZmpSplineType, uint step);
  virtual ~ZmpSpline() {};

  uint GetId() const { return id_; };
  double GetDuration() const { return duration_; }
  ZmpSplineType GetType() const { return type_; }
  int GetNodeCount(double dt) const { return std::floor(duration_/dt); }

  uint GetCurrStep() const
  {
    // Only if spline is "StepSpline" a step is currently beeing executed.
    // If this fails, call "GetCurrStep", because currently in four-leg-support
    assert(!IsFourLegSupport());
    return step_;
  }

  uint GetNextStep() const
  {
    // Only if spline is currently a four-leg support spline.
    // The next step is the step planned to execute after the 4ls-phase is complete.
    assert(IsFourLegSupport());
    return step_;
  }

  bool IsFourLegSupport() const { return type_ != StepSpline; }

private:
  uint id_; // to identify the order relative to other zmp splines
  double duration_; // time during which this spline is active
  ZmpSplineType type_;
  uint step_; // current step if step spline, otherwise planned next step

  friend struct xpp::ros::RosHelpers;
  friend std::ostream& operator<<(std::ostream& out, const ZmpSpline& tr);

};


inline std::ostream& operator<<(std::ostream& out, const ZmpSpline& s)
{
  out << "Spline: id= "   << s.id_                << ":\t"
      << "duration="      << s.duration_          << "\t"
      << "four_leg_supp=" << s.IsFourLegSupport() << "\t"
      << "step="          << s.step_              << "\n";
  return out;
}




} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_SPLINE_H_
