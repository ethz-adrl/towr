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
  typedef xpp::utils::Coords3D Coords;
  typedef std::vector<PosVelAcc> Derivatives;
  typedef std::shared_ptr<ComSpline> Ptr;
  typedef std::unique_ptr<ComSpline> UniquePtr;

  ComSpline ();
  virtual ~ComSpline ();

  // implements these functions from parent class, now specific for splines
  Point2d GetCom(double t_global) const override { return GetCOGxy(t_global, polynomials_); }
  double GetTotalTime() const override { return GetTotalTime(polynomials_); }
  PhaseInfo GetCurrentPhase(double t_global) const override;
  PhaseInfoVec GetPhases() const override;
  int GetTotalFreeCoeff() const override;
  VectorXd GetCoeffients () const override;

  int Index(int polynomial, Coords dim, SplineCoeff coeff) const;


  /** The motions (pos,vel,acc) that are fixed by spline structure and cannot
    * be modified through the coefficient values. These will be constrained
    * in the nonlinear program.
    */
  virtual Derivatives GetInitialFreeMotions()  const = 0;
  virtual Derivatives GetJunctionFreeMotions() const = 0;
  virtual Derivatives GetFinalFreeMotions()    const = 0;


  static Point2d GetCOGxy(double t_global, const VecPolynomials& splines);
  static int GetPolynomialID(double t_global, const VecPolynomials& splines);
  static double GetTotalTime(const VecPolynomials& splines);

  int GetPolynomialID(double t_global)  const { return GetPolynomialID(t_global, polynomials_); }
  double GetLocalTime(double t_global)  const { return GetLocalTime(t_global, polynomials_); };
  VecPolynomials GetPolynomials()       const { return polynomials_; }
  ComPolynomial GetPolynomial(size_t i) const { return polynomials_.at(i); }
  ComPolynomial GetLastPolynomial()     const { return polynomials_.back(); };


  virtual UniquePtr clone() const = 0;


  // refactor write documentation or get rid off
  /**
    *
    * @param posVelAcc
    * @param t_local
    * @param id
    * @param dim
    * @return
    */
  Jacobian GetJacobianWrtCoeffAtPolynomial(PosVelAcc posVelAcc, double t_local, int id, Coords3D dim) const;
  static Point2d GetCOGxyAtPolynomial(int id, double t_local, const VecPolynomials& splines);
  Point2d GetCOGxyAtPolynomial(int id, double t_local) {return GetCOGxyAtPolynomial(id, t_local, polynomials_); };


  Jacobian GetJacobian(double t_global, PosVelAcc posVelAcc, Coords3D dim) const override;

protected:
  VecPolynomials polynomials_;
  void Init(int step_count, const SplineTimes& times, bool insert_initial_stance);
  void CheckIfSplinesInitialized() const;


private:

  virtual void ExpressCogPosThroughABCD (double t_local, int id, Coords dim, Jacobian&) const = 0;
  virtual void ExpressCogVelThroughABCD (double t_local, int id, Coords dim, Jacobian&) const = 0;
  virtual void ExpressCogAccThroughABCD (double t_local, int id, Coords dim, Jacobian&) const = 0;
  virtual void ExpressCogJerkThroughABCD(double t_local, int id, Coords dim, Jacobian&) const = 0;

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
