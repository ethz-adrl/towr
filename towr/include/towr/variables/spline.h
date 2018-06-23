/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_VARIABLES_SPLINE_H_
#define TOWR_VARIABLES_SPLINE_H_

#include <vector>

#include "polynomial.h"

namespace towr {

/**
 * @brief A spline built from a sequence of cubic polynomials.
 *
 * This class is responsible for stitching together multiple individual
 * polynomials into one spline.
 */
class Spline  {
public:
  using VecTimes = std::vector<double>;
  using VecPoly  = std::vector<CubicHermitePolynomial>;

  Spline(const VecTimes& poly_durations, int n_dim);
  virtual ~Spline () = default;

  /**
   * @returns The state of the spline at time t.
   * @param t  The time at which the state of the spline is desired.
   */
  const State GetPoint(double t) const;

  /**
   * @param poly_id  Polynomial id, 0 is first polynomial.
   * @param t_local  Time along the current polynomial.
   * @returns The position, velocity and acceleration of spline.
   */
  const State GetPoint(int poly_id, double t_local) const;

  /**
   * @returns The segment (e.g. phase, polynomial) at time t_global.
   * @param t_global  The global time in the spline.
   * @param durations The durations [s] of each segment.
   */
  static int GetSegmentID(double t_global, const VecTimes& durations);

  /**
   * @returns The total time of the spline.
   */
  double GetTotalTime() const;

  /**
   * @returns the number of polynomials making up this spline.
   */
  int GetPolynomialCount() const;

  /**
   * @returns the durations of each polynomial.
   */
  VecTimes GetPolyDurations() const;

protected:
  VecPoly cubic_polys_; ///< the sequence of polynomials making up the spline.

  /**
   * @brief How much time of the current segment has passed at t_global.
   * @param t_global The global time [s] along the spline.
   * @param d The durations of each segment.
   * @return The segment id and the time passed in this segment.
   */
  std::pair<int,double> GetLocalTime(double t_global, const VecTimes& d) const;

  /**
   * @brief Updates the cubic-Hermite polynomial coefficients using the
   *        currently set nodes values and durations.
   */
  void UpdatePolynomialCoeff();
};

} /* namespace towr */

#endif /* TOWR_VARIABLES_SPLINE_H_ */
