/**
 @file    com_spline.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the class ComSpline
 */

#ifndef TOWR_VARIABLES_SPLINE_H_
#define TOWR_VARIABLES_SPLINE_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/state.h>

#include <ifopt/composite.h> // for Jacobian definition

#include "polynomial.h"
#include "node_values.h"

#include "spline_observer.h"
//#include "contact_schedule.h" // don't need this because of observer pattern


namespace towr {



/** @brief Wraps a sequence of polynomials with optimized coefficients.
  *
  * This class is responsible for abstracting polynomial coefficients of multiple
  * polynomials into a spline with position/velocity and acceleration.
  *
  * could also have derive from Observer class, with UpdateDurations() and UpdateNodes()
  */
class Spline : public SplineObserver {
public:
  using Ptr        = std::shared_ptr<Spline>;
  using LocalInfo  = std::pair<int,double>; ///< id and local time
  using StateLinXd = xpp::StateLinXd;
  using MotionDerivative = xpp::MotionDerivative;

  using VecTimes = std::vector<double>;
  using Node     = CubicHermitePoly::Node;
  using VecNodes = std::vector<Node>;
  using Side     = CubicHermitePoly::Side;
  using VecPoly  = std::vector<std::shared_ptr<CubicHermitePoly>>;

  using Jacobian = ifopt::Component::Jacobian;

  // the constructor with constant durations
  Spline(const NodeValues::Ptr& nodes, const VecTimes& phase_durations);

//  Spline(const NodeValues::Ptr& nodes,
//         const ContactSchedule::Ptr& contact_schedule);


  virtual ~Spline () = default;



  static int GetSegmentID(double t_global, const VecTimes& durations);
  static LocalInfo GetLocalTime(double t_global, const VecTimes& durations);

  std::string GetName() { return node_values_->GetName(); };


  // some observer kinda stuff
  void UpdatePhaseDurations(const VecTimes& phase_durations);
//  void UpdateNodes(const VecNodes& nodes);
  virtual void UpdatePolynomials() override ;




  virtual const StateLinXd GetPoint(double t_global) const;

  // call GetJacobianWrtNodeValues()
  virtual Jacobian GetJacobian(double t_global, MotionDerivative dxdt) const;
  // add GetJacobianPosWrtTime()
  Eigen::VectorXd GetDerivativeOfPosWrtPhaseDuration (double t_global) const;


  // possibly move to different class
//  bool IsConstantPhase(double t) const;

private:

  // these two elements are combined here into cubic polynomials
  VecTimes poly_durations_;
  NodeValues::Ptr node_values_;
//  ContactSchedule::Ptr contact_schedule_;

  VecPoly cubic_polys_;



  mutable bool fill_jacobian_structure_ = true;
  mutable Jacobian jac_structure_; // all zeros
  bool durations_change_ = false;



  // fill_with_zeros is to get sparsity
  void FillJacobian (int poly_id, double t_local, MotionDerivative dxdt,
                     Jacobian& jac, bool fill_with_zeros) const;



  // possibly move this to a derived class
  VecTimes ConvertPhaseToSplineDurations(const VecTimes& phase_durations) const;
//  VecTimes phase_durations_;

};

} /* namespace towr */

#endif /* TOWR_VARIABLES_SPLINE_H_ */
