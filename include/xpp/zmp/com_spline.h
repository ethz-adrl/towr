/**
 @file    com_spline.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the class ComSpline
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_POLYNOMIAL_FIFTH_ORDER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_POLYNOMIAL_FIFTH_ORDER_H_

#include "com_motion.h"
#include "com_polynomial.h"
#include <memory>

namespace xpp {
namespace zmp {


/** The duration of each polynome in the sequence that describes some trajectory */
struct SplineTimes
{
  SplineTimes() {};

  /** @param t_swing time for one step
    * @param t_init time before executing the first step
    */
  SplineTimes(double t_swing, double t_init)
      :t_swing_(t_swing), t_stance_initial_(t_init) {}

  double t_swing_;
  double t_stance_initial_;

  void SetDefault() {
    t_swing_          = 0.7; //s
    t_stance_initial_ = 0.4; //s
  }
};



/** Represents the Center of Mass (CoM) motion as a Spline (sequence of polynomials).
  *
  * This class is responsible for abstracting polynomial coefficients of multiple
  * polynomials into a CoM position/velocity and acceleration.
  */
class ComSpline : public ComMotion {
public:
  typedef std::vector<ComPolynomial> VecPolynomials;
  typedef xpp::utils::PosVelAcc PosVelAcc;
  typedef xpp::utils::VecScalar VecScalar;
  typedef xpp::utils::Point2d Point2d;
  typedef std::shared_ptr<ComSpline> Ptr;
  typedef xpp::utils::Coords3D Coords;

  ComSpline ();
  virtual ~ComSpline ();


  // implements these functions from parent class
  Point2d GetCom(double t_global) const override { return GetCOGxy(t_global, polynomials_); }
  double GetTotalTime() const override { return GetTotalTime(polynomials_); }
  PhaseInfo GetCurrentPhase(double t_global) const override;
  PhaseInfoVec GetPhases() const override;
  int GetTotalFreeCoeff() const override;
  VectorXd GetCoeffients () const override;

  int Index(int spline, Coords dim, SplineCoeff coeff) const;

  static Point2d GetCOGxy(double t_global, const VecPolynomials& splines);
  static int GetPolynomialID(double t_global, const VecPolynomials& splines);
  static double GetTotalTime(const VecPolynomials& splines);

  int GetPolynomialID(double t_global)  const { return GetPolynomialID(t_global, polynomials_); }
  double GetLocalTime(double t_global)  const { return GetLocalTime(t_global, polynomials_); };
  VecPolynomials GetPolynomials()       const { return polynomials_; }
  ComPolynomial GetPolynomial(size_t i) const { return polynomials_.at(i); }
  ComPolynomial GetLastPolynomial()     const { return polynomials_.back(); };


  /** Produces a vector and scalar, that, multiplied with the spline coefficients
    * a,b,c,d of all splines returns the position of the CoG at time t_local.
    *
    * @param t_local @attention local time of spline. So t_local=0 returns CoG at beginning of this spline.
    * @param id id of current spline
    * @param dim dimension specifying if x or y coordinate of CoG should be calculated
    * @return
    */
  VecScalar ExpressComThroughCoeff(PosVelAcc, double t_local, int id, Coords dim) const;


protected:
  VecPolynomials polynomials_;
  void Init(int step_count, const SplineTimes& times, bool insert_initial_stance);
  void CheckIfSplinesInitialized() const;

private:
  virtual VecScalar ExpressCogPosThroughABCD (double t_local, int id, Coords dim) const = 0;
  virtual VecScalar ExpressCogVelThroughABCD (double t_local, int id, Coords dim) const = 0;
  virtual VecScalar ExpressCogAccThroughABCD (double t_local, int id, Coords dim) const = 0;
  virtual VecScalar ExpressCogJerkThroughABCD(double t_local, int id, Coords dim) const = 0;

  virtual int NumFreeCoeffPerSpline() const = 0;
  virtual std::vector<SplineCoeff> GetFreeCoeffPerSpline() const = 0;

  void AddPolynomialStepSequence(int step_count, double t_swing);
  void AddStancePolynomial(double t_stance);

  static double GetLocalTime(double t_global, const VecPolynomials& splines);
  bool splines_initialized_ = false;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_POLYNOMIAL_FIFTH_ORDER_H_ */
