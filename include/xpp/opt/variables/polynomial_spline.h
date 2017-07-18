/**
 @file    com_spline.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the class ComSpline
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_POLYNOMIAL_SPLINE_H_
#define XPP_OPT_INCLUDE_XPP_OPT_POLYNOMIAL_SPLINE_H_

#include <memory>
#include <string>
#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/state.h>

#include <xpp/opt/constraints/composite.h>
#include <xpp/opt/polynomial.h>

namespace xpp {
namespace opt {

/** @brief Represents a sequence of polynomials with optimized coefficients.
  *
  * This class is responsible for abstracting polynomial coefficients of multiple
  * polynomials into a spline with position/velocity and acceleration.
  */
// zmp_ the whole class can be seen as a composite of polynomial components.
// zmp_ this shouldn't be a composite, just a facade class (use word facade)
class PolynomialSpline {
public:
  using PolynomialPtr  = std::shared_ptr<Polynomial>;
  using VecPolynomials = std::vector<PolynomialPtr>;
  using VecTimes       = std::vector<double>;
  using PtrS           = std::shared_ptr<PolynomialSpline>; // pointer to oneself
  using OptVarsPtr     = Primitive::OptVarsPtr;

  PolynomialSpline (const std::string& component_name);
  virtual ~PolynomialSpline ();

  // zmp_ dummy method just to get to compile
  std::string GetName() const {};
  // zmp_ also remove this one
  int GetRows() const { return polynomials_.size()*polynomials_.front()->GetValues().rows();};


  // zmp_ move to src
  static PtrS BuildSpline(const OptVarsPtr& opt_vars,
                          const std::string& spline_base_id,
                          const VecTimes& poly_durations)
  {
    auto spline_ = std::make_shared<PolynomialSpline>("dummy");
    spline_->SetDurations(poly_durations);
    for (int i=0; i<poly_durations.size(); ++i) {
      auto p = std::dynamic_pointer_cast<Polynomial>(opt_vars->GetComponent(spline_base_id+std::to_string(i)));
      spline_->AddPolynomial(p);
    }

    return spline_;
  }

  // zmp_ remove
//  void FillCorrectPolynomial(std::string var_set, Jacobian& jac, const MotionDerivative& dxdt)
//  {
//    // check which polynomial we are at
//    for (auto p : polynomials_) {
//      if (var_set == p->GetName()) {
//        Jacobian jac_dxdt = spline_->GetJacobian(t_, dxdt);
//        jac.middleRows(row,n_dim_) =  spline_->GetJacobian(t_, dxdt);
//      }
//    }
//  }



  void Init (int n_polys, int poly_order, const VectorXd& initial_value);
  void SetPhaseDurations(const std::vector<double>& phase_durations,
                         int polys_per_duration);

//  VectorXd GetValues () const override;
//  void SetValues (const VectorXd& optimized_coeff) override;

//  int Index(int polynomial, int dim, Polynomial::PolynomialCoeff coeff) const;

//  /** @brief Calculates the Jacobian at a specific state.
//    *
//    * @param dxdt whether position, velocity, acceleration or jerk Jacobian is desired
//    * @param t_poly the time at which the Jacobian is desired, expressed since current polynomial is active.
//    * @param id the ID of the current polynomial
//    * @param dim in which dimension (x,y) the Jacobian is desired.
//    */
//  JacobianRow GetJacobianWrtCoeffAtPolynomial(MotionDerivative dxdt,
//                                              double t_poly, int id,
//                                              int dim) const;
//  JacobianRow GetJacobianWrtCoeffAtPolynomial(MotionDerivative dxdt,
//                                              int id, double t_poly,
//                                              int dim) const = delete;

//  JacobianRow GetJacobian(double t_global, MotionDerivative dxdt, int dim) const;

  // zmp_ there is no need to combine these jacobians!
  Jacobian    GetJacobian(double t_global, MotionDerivative dxdt) const;
  Jacobian    GetJacobian(int id, double t_local, MotionDerivative dxdt) const;

  VecPolynomials GetPolynomials() const { return polynomials_; }
  PolynomialPtr GetPolynomial(int id) const { return polynomials_.at(id); }

  PolynomialPtr GetActivePolynomial(double t_global) const
  {
    int id = GetSegmentID(t_global, durations_);
    return polynomials_.at(id);
  }


//  int GetNDim() const {return n_dim_; };


  const StateLinXd GetPoint(double t_globals) const;
  const StateLinXd GetPoint(int id, double t_local) const;

  static int GetSegmentID(double t_global, const VecTimes&);
  static double GetLocalTime(double t_global, const VecTimes&);

  double GetDurationOfPoly(int id) const { return durations_.at(id); };


  // zmp_ move to source
  void AddPolynomial(const PolynomialPtr& poly)
  {
    polynomials_.push_back(poly);
  }

  void SetDurations(const VecTimes& durations)
  {
    durations_ = durations;
  }



protected:
  VecTimes durations_; ///< duration of each polynomial in spline

  ///< pointer to retain access to polynomial functions
  VecPolynomials polynomials_;

  int n_dim_;

  int n_polys_per_phase_; // polynomials used to represent each timing phase

//  int GetFreeCoeffPerPoly() const;
};



//class EndeffectorSpline : public PolynomialSpline {
//public:
//  EndeffectorSpline(const std::string& id, bool first_phase_in_contact);
//  virtual ~EndeffectorSpline ();
//
//  virtual VecBound GetBounds () const override;
//
//  // shouldn't be allowed to use this function for now
////  Jacobian GetJacobian(double t_global, MotionDerivative dxdt) = delete;
//
//private:
//  bool first_phase_in_contact_;
//};


//// zmp_ DRY almost the same as above, combine
//class ForceSpline : public PolynomialSpline {
//public:
//  ForceSpline(const std::string& id, bool first_phase_in_contact, double max_force);
//  virtual ~ForceSpline ();
//
//  virtual VecBound GetBounds () const override;
//
//private:
//  bool first_phase_in_contact_;
//  double max_force_;
//};







} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_POLYNOMIAL_SPLINE_H_ */
