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

#ifndef OPT_SOLVE_INCLUDE_OPT_BOUNDS_H_
#define OPT_SOLVE_INCLUDE_OPT_BOUNDS_H_

namespace opt {

/**
 * @brief Upper and lower bound for optimization variables and constraints.
 */
struct Bounds {

  /**
   * @brief Creates a bound between @a lower and @a upper.
   */
  Bounds(double lower = 0.0, double upper = 0.0)
  {
    lower_ = lower;
    upper_ = upper;
  }

  double lower_;
  double upper_;

  void operator+=(double scalar)
  {
    lower_ += scalar;
    upper_ += scalar;
  }

  void operator-=(double scalar)
  {
    lower_ -= scalar;
    upper_ -= scalar;
  }
};

// settings this as signals infinity for IPOPT/SNOPT solvers
static const double inf = 1.0e20;

static const Bounds NoBound          = Bounds(-inf, +inf);
static const Bounds BoundZero        = Bounds( 0.0,  0.0);
static const Bounds BoundGreaterZero = Bounds( 0.0, +inf);
static const Bounds BoundSmallerZero = Bounds(-inf,  0.0);

} // namespace opt

#endif /* OPT_SOLVE_INCLUDE_OPT_BOUNDS_H_ */
