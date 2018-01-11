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

#include "contact_schedule_observer.h"
#include "nodes_observer.h"


namespace towr {



/** @brief Wraps a sequence of polynomials with optimized coefficients.
  *
  * This class is responsible for abstracting polynomial coefficients of multiple
  * polynomials into a spline with position/velocity and acceleration.
  *
  * could also have derive from Observer class, with UpdateDurations() and UpdateNodes()
  */
class Spline : public NodesObserver, public ContactScheduleObserver {
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
  // don't take ownership of object
  Spline(NodeValues* const, const VecTimes& phase_durations);

  // the contructor with changing durations
  Spline(NodeValues* const, ContactSchedule* const);


  virtual ~Spline () = default;



  // some observer kinda stuff
  // only make one update() function?
  // triggered when nodes or durations change values
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


  VecPoly cubic_polys_;

  void Init(const VecTimes& durations);


  // the structure of the nonzero elements of the jacobian with respect to
  // the node values
  mutable Jacobian jac_wrt_nodes_structure_; // all zeros


  // fill_with_zeros is to get sparsity
  void FillJacobian (int poly_id, double t_local, MotionDerivative dxdt,
                     Jacobian& jac, bool fill_with_zeros) const;



  // possibly move this to a derived class
  VecTimes ConvertPhaseToPolyDurations(const VecTimes& phase_durations) const;
//  VecTimes phase_durations_;

};

} /* namespace towr */

#endif /* TOWR_VARIABLES_SPLINE_H_ */
