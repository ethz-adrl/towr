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
#include <xpp/opt/spline.h>

namespace xpp {
namespace opt {

/** @brief Represents a sequence of polynomials with optimized coefficients.
  *
  * This class is responsible for abstracting polynomial coefficients of multiple
  * polynomials into a spline with position/velocity and acceleration.
  */
class PolynomialSpline : public Component, public Spline {
public:
  using State          = StateLinXd;
  using PolyCoeff      = Polynomial::PolynomialCoeff;
  using PolyomialPtr   = std::shared_ptr<Polynomial>;
  using VecPolynomials = std::vector<PolyomialPtr>;

  PolynomialSpline (const std::string& component_name);
  virtual ~PolynomialSpline ();

  template<typename PolyT>
  void Init (double t_global, double dt, const VectorXd& initial_value)
  {
    // initialize at com position with zero velocity & acceleration
    n_dim_ = initial_value.rows();
    State initial_state(n_dim_);
    initial_state.p_ = initial_value;

    double t_left = t_global;
    while (t_left > 0.0) {
      double duration = t_left>dt?  dt : t_left;
      auto p = std::make_shared<PolyT>();
      p->SetBoundary(duration, initial_state, initial_state);
      polynomials_.push_back(p);
      t_left -= dt;
    }

    SetSegmentsPtr(polynomials_);

    int n_polys = polynomials_.size();
    SetRows(n_polys*GetFreeCoeffPerPoly()*n_dim_);
  }


  // zmp_ DRY with other init function
  template<typename PolyT>
  void Init (std::vector<double> T_polys, const VectorXd& initial_value)
  {
    // initialize at com position with zero velocity & acceleration
    n_dim_ = initial_value.rows();
    State initial_state(n_dim_);
    initial_state.p_ = initial_value;

    for (double duration : T_polys) {
      auto p = std::make_shared<PolyT>();
      p->SetBoundary(duration, initial_state, initial_state);
      polynomials_.push_back(p);
    }

    // DRY with above init
    SetSegmentsPtr(polynomials_);
    int n_polys = polynomials_.size();
    SetRows(n_polys*GetFreeCoeffPerPoly()*n_dim_);
  }




  void Init(double t_global, double duration_per_polynomial,
            const VectorXd& initial_val);

  void Init(std::vector<double> T_polys, const VectorXd& initial_val);

  VectorXd GetValues () const override;
  void SetValues (const VectorXd& optimized_coeff) override;

  int Index(int polynomial, int dim, PolyCoeff coeff) const;

  /** Calculates the Jacobian at a specific time of the motion, but specified by
    * a local time and a polynome id. This allows to create spline junction constraints
    *
    * @param dxdt whether position, velocity, acceleration or jerk Jacobian is desired
    * @param t_poly the time at which the Jacobian is desired, expressed since current polynomial is active.
    * @param id the ID of the current polynomial
    * @param dim in which dimension (x,y) the Jacobian is desired.
    */
  JacobianRow GetJacobianWrtCoeffAtPolynomial(MotionDerivative dxdt,
                                              double t_poly, int id,
                                              int dim) const;
  JacobianRow GetJacobianWrtCoeffAtPolynomial(MotionDerivative dxdt,
                                              int id, double t_poly,
                                              int dim) const = delete;

  JacobianRow GetJacobian(double t_global, MotionDerivative dxdt, int dim) const;
  Jacobian    GetJacobian(double t_global, MotionDerivative dxdt) const;
  Jacobian    GetJacobian(int id, double t_local, MotionDerivative dxdt) const;

  VecPolynomials GetPolynomials() const { return polynomials_; }
  int GetNDim() const {return n_dim_; };

protected:
  VecPolynomials polynomials_; ///< pointer to retain access to polynomial functions
  int n_dim_;

  int GetFreeCoeffPerPoly() const;
};



class EndeffectorSpline : public PolynomialSpline {
public:
  EndeffectorSpline(const std::string& id, bool first_phase_in_contact);
  virtual ~EndeffectorSpline ();

  virtual VecBound GetBounds () const override;

  // shouldn't be allowed to use this function for now
//  Jacobian GetJacobian(double t_global, MotionDerivative dxdt) = delete;

private:
  bool first_phase_in_contact_;
};

// zmp_ DRY almost the same as above, combine
class ForceSpline : public PolynomialSpline {
public:
  ForceSpline(const std::string& id, bool first_phase_in_contact, double max_force);
  virtual ~ForceSpline ();

  virtual VecBound GetBounds () const override;

  // zmp_ DRY with other init function
  template<typename PolyT>
  void Init (std::vector<double> T_polys,
             int n_polys_per_phase,
             const VectorXd& initial_value)
  {
    n_polys_per_phase_ = n_polys_per_phase;

    // initialize at com position with zero velocity & acceleration
    n_dim_ = initial_value.rows();
    State initial_state(n_dim_);
    initial_state.p_ = initial_value;

    for (double duration : T_polys) {
      for (int i=0; i < n_polys_per_phase_; ++i) {
        auto p = std::make_shared<PolyT>();
        p->SetBoundary(duration/n_polys_per_phase_, initial_state, initial_state);
        polynomials_.push_back(p);
      }
    }

    // DRY with above init
    SetSegmentsPtr(polynomials_);
    int n_polys = polynomials_.size();
    SetRows(n_polys*GetFreeCoeffPerPoly()*n_dim_);
  }

private:
  bool first_phase_in_contact_;
  double max_force_;

  int n_polys_per_phase_; // polynomials used to represent each stance/swing phase
};







} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_POLYNOMIAL_SPLINE_H_ */
