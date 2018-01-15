/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_VARIABLES_SPLINE_H_
#define TOWR_VARIABLES_SPLINE_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Sparse>


#include "polynomial.h"
#include "nodes_observer.h"
#include "contact_schedule_observer.h"


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
  using VectorXd   = Eigen::VectorXd;
  using Jacobian   = Eigen::SparseMatrix<double, Eigen::RowMajor>;

  using VecTimes = std::vector<double>;
  using VecNodes = std::vector<Node>;
  using VecPoly  = std::vector<CubicHermitePoly>;

  // the constructor with constant durations
  // don't take ownership of object
  Spline(NodesObserver::SubjectPtr const, const VecTimes& phase_durations);

  // the contructor with changing durations
  Spline(NodesObserver::SubjectPtr const, ContactSchedule* const);


  virtual ~Spline () = default;



  // some observer kinda stuff
  // only make one update() function?
  // triggered when nodes or durations change values
  void UpdatePhaseDurations();
  virtual void UpdatePolynomials() override ;




  const State GetPoint(double t_global) const;


  Jacobian GetJacobianWrtNodes(double t_global, Dx dxdt) const;
  Jacobian GetJacobianOfPosWrtDurations(double t_global) const;

  // possibly move to different class
  bool IsConstantPhase(double t) const;

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
  void FillJacobian (int poly_id, double t_local, Dx dxdt,
                     Jacobian& jac, bool fill_with_zeros) const;


  void SetPolyFromPhaseDurations(const VecTimes& phase_durations);


};

} /* namespace towr */

#endif /* TOWR_VARIABLES_SPLINE_H_ */
