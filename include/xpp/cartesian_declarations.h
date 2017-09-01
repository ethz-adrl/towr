/**
@file    cartesian_declarations.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Sep 28, 2016
@brief   Defines common Identifiers to be used in Cartesian Space
 */

#ifndef XPP_CARTESIAN_DECLARATIONS_H_
#define XPP_CARTESIAN_DECLARATIONS_H_

#include <cassert>

namespace xpp {

/// To be used with 6D vectors. 'A' stands for angular, 'L' for linear.
enum Coords2D { X_=0, Y_};
enum Coords3D { X=0, Y, Z };

// could create constructor, that takes as input an enum of Coords2D and transforms to 3D.
// but only one way
static Coords2D To2D(Coords3D dim)
{
  assert(dim != Z);
  return static_cast<Coords2D>(dim);
};


enum Coords6D { AX=0, AY, AZ, LX, LY, LZ };
static const Coords6D AllDim6D[] = {AX, AY, AZ, LX, LY, LZ};

static Coords3D To3D(Coords6D dim6d) {
  assert(LX==dim6d || dim6d==LY || dim6d==LZ);
  return static_cast<Coords3D>(dim6d-3);
}

static Coords6D To6D(Coords3D dim3d) {
  return static_cast<Coords6D>(dim3d+3);
}

//namespace d2 {
//  enum Coords { X=xpp::X, Y=xpp::Y };
//  static const Coords AllDimensions[] = { X, Y };
//}
//
//static d2::Coords To2D(Coords6D dim6d) {
//  assert(dim6d == LX || dim6d == LY);
//  return static_cast<d2::Coords>(dim6d-3);
//}

static constexpr int kDim2d = 2; // X,Y
static constexpr int kDim3d = 3; // X,Y,Z
static constexpr int kDim6d = 6; // X,Y,Z, alpha, beta, gamma
enum MotionDerivative { kPos=0, kVel, kAcc, kJerk };


} // namespace xpp


#endif /* XPP_CARTESIAN_DECLARATIONS_H_ */
