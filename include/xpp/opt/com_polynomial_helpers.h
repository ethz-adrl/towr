/**
@file   polynomial_helpers.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2015
@brief  Declares a representation of a Center of Mass Polynomial
 */

#ifndef _XPP_ZMP_COM_POLYNOMIAL_H_
#define _XPP_ZMP_COM_POLYNOMIAL_H_

#include <vector>

#include <xpp/state.h>

#include "polynomial.h"
#include "polynomial_xd.h"

namespace xpp {
namespace opt {

// for now a 2d polynomial
using ComPolynomial = PolynomialXd<QuinticPolynomial, StateLin2d>;

/** Some convenience functions to interact with a multiple polynomials.
  */
class ComPolynomialHelpers {
public:
  using VecPolynomials = std::vector<ComPolynomial>;

  static StateLin2d GetCOM(double t_global, const VecPolynomials& splines);
  static int GetPolynomialID(double t_global, const VecPolynomials& splines);
  static double GetTotalTime(const VecPolynomials& splines);
  static double GetLocalTime(double t_global, const VecPolynomials& splines);
  static StateLin2d GetCOGxyAtPolynomial(int id, double t_local, const VecPolynomials& splines);
};


} // namespace opt
} // namespace xpp

#endif // _XPP_ZMP_COM_POLYNOMIAL_H_
