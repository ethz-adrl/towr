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

#include <towr/models/dynamic_model.h>

namespace towr {

DynamicModel::DynamicModel(double mass, int ee_count)
{
  m_ = mass;
  g_ = 9.80665;

  com_pos_.setZero();
  com_acc_.setZero();

  w_R_b_.setIdentity();
  omega_.setZero();
  omega_dot_ .setZero();

  ee_force_ = EELoad(ee_count);
  ee_pos_ = EEPos(ee_count);
}

void
DynamicModel::SetCurrent (const ComPos& com_W, const Vector3d com_acc_W,
                          const Matrix3d& w_R_b, const AngVel& omega_W, const Vector3d& omega_dot_W,
                          const EELoad& force_W, const EEPos& pos_W)
{
  com_pos_   = com_W;
  com_acc_   = com_acc_W;

  w_R_b_     = w_R_b;
  omega_     = omega_W;
  omega_dot_ = omega_dot_W;

  ee_force_  = force_W;
  ee_pos_    = pos_W;
}

} /* namespace towr */
