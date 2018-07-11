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

#ifndef SPLINE_ACC_CONSTRAINT_H_
#define SPLINE_ACC_CONSTRAINT_H_

#include <ifopt/constraint_set.h>

#include <towr/variables/node_spline.h>

namespace towr {

/**
 *  @brief Ensures continuous accelerations between polynomials.
 *
 *  This is used to restrict jumps in linear and angular base accelerations,
 *  since this would require jumps in foot positions or endeffector forces,
 *  which aren't allowed in our formulation.
 *
 * @ingroup Constraints
 */
class SplineAccConstraint : public ifopt::ConstraintSet {
public:
  SplineAccConstraint(const NodeSpline::Ptr& spline, std::string name);
  virtual ~SplineAccConstraint() = default;

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;

private:
  NodeSpline::Ptr spline_;        ///< a spline comprised of polynomials
  std::string node_variables_id_; /// polynomial parameterized node values

  int n_junctions_;       ///< number of junctions between polynomials in spline.
  int n_dim_;             ///< dimensions that this polynomial represents (e.g. x,y).
  std::vector<double> T_; ///< Duration of each polynomial in spline.
};

} /* namespace towr */

#endif /* SPLINE_ACC_CONSTRAINT_H_ */
