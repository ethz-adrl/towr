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

#ifndef TOWR_TOWR_H_
#define TOWR_TOWR_H_

#include <iostream>

#include <ifopt/problem.h>
#include <ifopt/solver.h>

#include <towr/variables/spline_holder.h>

#include <towr/models/robot_model.h>
#include <towr/terrain/height_map.h>
#include "nlp_factory.h"
#include "parameters.h"


/**
 * @brief Entire code of this project is defined in this namespace.
 */
namespace towr {

/**
 * @brief %TOWR - Trajectory Optimizer for Walking Robots.
 *
 * Facade to the motion optimization framework. The user sets the initial
 * state and the desired motion parameters, then and NLP is constructed and
 * solved with the chosen solver and finally the solution splines can be
 * retrieved.
 */
class TOWR {
public:
  using FeetPos = std::vector<Eigen::Vector3d>;

  TOWR (bool print_boilerplate = true);
  virtual ~TOWR () = default;

  /**
   * @brief The current state of the robot where the optimization starts from.
   *
   * @param base  The linear and angular position and velocity of the 6D- base.
   * @param feet  The current position of the end-effectors.
   */
  void SetInitialState(const BaseState& base, const FeetPos& feet);

  /**
   * @brief  The parameters that determine the type of motion produced.
   *
   * @param final_base  The desired final position and velocity of the base.
   * @param params      The parameters defining the optimization problem.
   * @param model       The kinematic and dynamic model of the system.
   * @param terrain     The height map of the terrain to walk over.
   */
  void SetParameters(const BaseState& final_base,
                     const Parameters& params,
                     const RobotModel& model,
                     HeightMap::Ptr terrain);

  /**
   * @brief Constructs the problem and solves it with IPOPT.
   *
   * Could use any ifopt solver interface, see (http://wiki.ros.org/ifopt).
   * Currently IPOPT and SNOPT are implemented.
   */
  void SolveNLP(const ifopt::Solver::Ptr& solver);

  /**
   * @returns the optimized motion for base, feet and force as splines.
   *
   * The can then be queried at specific times to get the positions, velocities,
   * or forces.
   */
  SplineHolder GetSolution() const;

  /**
   * @brief Sets the solution to a previous iteration of solver.
   * @param solver_iteration  The iteration to be inspected.
   *
   * This can be helpful when trying to understand how the NLP solver reached
   * a particular solution. The initialization of the NLP can also be inspected
   * by setting the iteration to zero.
   */
  void SetSolution(int solver_iteration);

  /**
   * @returns The number of iterations the solver took to find the solution.
   */
  int GetIterationCount() const;

private:
  /**
   * @brief The solver independent optimization problem formulation.
   *
   * This object holds ownership of the optimization variables, so must
   * exist to query the solution values.
   */
  ifopt::Problem nlp_;

  NlpFactory factory_;

  /**
   * @returns the solver independent optimization problem.
   */
  ifopt::Problem BuildNLP();
};

} /* namespace towr */

#endif /* TOWR_TOWR_H_ */
