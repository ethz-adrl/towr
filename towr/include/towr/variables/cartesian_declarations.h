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

/**
 * @file cartesian_declartions.h
 *
 * Defines common conventions and index values to be used in Cartesian
 * environments.
 */
#ifndef TOWR_VARIABLES_CARTESIAN_DECLARATIONS_H_
#define TOWR_VARIABLES_CARTESIAN_DECLARATIONS_H_

#include <cassert>

namespace towr {

// 2-dimensional
static constexpr int kDim2d = 2;
enum Coords2D { X_=0, Y_};

// 3-dimensional
static constexpr int kDim3d = 3;
enum Coords3D { X=0, Y, Z };
static Coords2D To2D(Coords3D dim)
{
  assert(dim != Z);
  return static_cast<Coords2D>(dim);
};

// 6-dimensional
// 'A' stands for angular, 'L' for linear.
static constexpr int kDim6d = 6; // X,Y,Z, roll, pitch, yaw
enum Coords6D { AX=0, AY, AZ, LX, LY, LZ };
static const Coords6D AllDim6D[] = {AX, AY, AZ, LX, LY, LZ};

} // namespace towr

#endif /* TOWR_VARIABLES_CARTESIAN_DECLARATIONS_H_ */
