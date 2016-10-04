/**
@file   com_polynomial.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2015
@brief  Declares a representation of a Center of Mass Polynomial
 */

#ifndef _XPP_ZMP_COM_POLYNOMIAL_H_
#define _XPP_ZMP_COM_POLYNOMIAL_H_

#include <xpp/utils/polynomial_xd.h>

namespace xpp {
namespace utils {

// for now a 2d polynomial
using ComPolynomial = PolynomialXd<QuinticPolynomial, kDim2d, BaseLin2d>;

/** A fifth order spline that now holds some context information about the
  *  Center of Mass (CoM).
  */
class ComPolynomialHelpers {
public:
  using VecPolynomials = std::vector<ComPolynomial>;

  static BaseLin2d GetCOM(double t_global, const VecPolynomials& splines);
  static int GetPolynomialID(double t_global, const VecPolynomials& splines);
  static double GetTotalTime(const VecPolynomials& splines);
  static double GetLocalTime(double t_global, const VecPolynomials& splines);
  static BaseLin2d GetCOGxyAtPolynomial(int id, double t_local, const VecPolynomials& splines);
};


} // namespace utils
} // namespace xpp

#endif // _XPP_ZMP_COM_POLYNOMIAL_H_
