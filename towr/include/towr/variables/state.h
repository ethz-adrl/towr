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

#ifndef TOWR_VARIABLES_STATE_H_
#define TOWR_VARIABLES_STATE_H_


#include <Eigen/Dense>


namespace towr {

// smell possibly move to cartesian declarations
enum MotionDerivative { kPos=0, kVel, kAcc };

/**
 * @brief Represents position, velocity and acceleration in x-dimensions.
 */
class StateLinXd {
public:
  using VectorXd = Eigen::VectorXd;

  VectorXd p_, v_, a_; ///< position, velocity and acceleration

  /**
   * @brief  Constructs an object of dimensions dim.
   *
   * Be careful of default value, as zero dimensional object is bound to
   * cause seg-fault at some point.
   */
  explicit StateLinXd(int dim);

  /**
   * @brief Constructs object with specific position, velocity and acceleration.
   * @param  p  Position of the state.
   * @param  v  Velocity of the state.
   * @param  a  Acceleration of the state.
   *
   * The dimensions are set to the number of rows of p.
   */
  explicit StateLinXd(const VectorXd& p, const VectorXd& v, const VectorXd& a);
  virtual ~StateLinXd() = default;

  /**
   * @brief  Read either position, velocity of acceleration by index.
   * @param  deriv  Index for that specific derivative (pos=0, vel=1, acc=2).
   * @return  Read only n-dimensional position, velocity or acceleration.
   */
  const VectorXd at(MotionDerivative deriv) const;

  /**
   * @brief  Read and write either position, velocity of acceleration by index.
   * @param  deriv  Index for that specific derivative (pos=0, vel=1, acc=2).
   * @return  Read/write n-dimensional position, velocity or acceleration.
   */
  VectorXd& at(MotionDerivative deriv);
};


} // namespace towr

#endif // TOWR_VARIABLES
