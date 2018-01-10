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
#include "contact_schedule.h"

#include "contact_schedule_observer.h"
#include "node_observer.h"
//#include "contact_schedule.h" // don't need this because of observer pattern


namespace towr {



/** @brief Wraps a sequence of polynomials with optimized coefficients.
  *
  * This class is responsible for abstracting polynomial coefficients of multiple
  * polynomials into a spline with position/velocity and acceleration.
  *
  * could also have derive from Observer class, with UpdateDurations() and UpdateNodes()
  */
// smell rename SplineObserver to NodeObserver, cuz that's what its watching
class Spline : public NodeObserver, public ContactScheduleObserver {
public:
  using Ptr        = std::shared_ptr<Spline>;
  using LocalInfo  = std::pair<int,double>; ///< id and local time
  using StateLinXd = xpp::StateLinXd;
  using MotionDerivative = xpp::MotionDerivative;

  using VecTimes = std::vector<double>;
  using Node     = CubicHermitePoly::Node;
  using VecNodes = std::vector<Node>;
  using Side     = CubicHermitePoly::Side;
  using VecPoly  = std::vector<CubicHermitePoly>;

  using Jacobian = ifopt::Component::Jacobian;

  // the constructor with constant durations
  Spline(const NodeValues::Ptr& nodes, const VecTimes& phase_durations);

  Spline(const NodeValues::Ptr& nodes,
         const ContactSchedule::Ptr& contact_schedule);

//  void SetContactSchedule(const ContactSchedule::Ptr& contact_schedule)
//  {
//    contact_schedule_ = contact_schedule;
//  }

//  Spline(const NodeValues::Ptr& nodes,
//         const ContactSchedule::Ptr& contact_schedule);


  virtual ~Spline () = default;




  // some observer kinda stuff
  // only make one update() function?
  void UpdatePhaseDurations();
  virtual void UpdatePolynomials() override ;




  const StateLinXd GetPoint(double t_global) const;


  Jacobian GetJacobianWrtNodes(double t_global, MotionDerivative dxdt) const;
  Jacobian GetJacobianOfPosWrtDurations(double t_global) const;

  // possibly move to different class
//  bool IsConstantPhase(double t) const;

private:
  int GetSegmentID(double t_global, const VecTimes& durations) const;
  LocalInfo GetLocalTime(double t_global, const VecTimes& durations) const;


  Eigen::VectorXd GetDerivativeOfPosWrtPhaseDuration (double t_global) const;

  // these two elements are combined here into cubic polynomials
  VecTimes poly_durations_;

  NodeValues::Ptr node_values_;
  ContactSchedule::Ptr contact_schedule_;

  VecPoly cubic_polys_;



  mutable bool fill_jacobian_structure_ = true;
  mutable Jacobian jac_structure_; // all zeros
  bool durations_change_ = false;



  // fill_with_zeros is to get sparsity
  void FillJacobian (int poly_id, double t_local, MotionDerivative dxdt,
                     Jacobian& jac, bool fill_with_zeros) const;



  // possibly move this to a derived class
  VecTimes ConvertPhaseToPolyDurations(const VecTimes& phase_durations) const;
//  VecTimes phase_durations_;

};

} /* namespace towr */

#endif /* TOWR_VARIABLES_SPLINE_H_ */
