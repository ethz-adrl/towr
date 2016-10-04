/**
@file   com_polynomial.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2015
@brief  Declares CoeffValues, PolynomialFifthOrder and ComPolynomial
 */

#ifndef _XPP_ZMP_COM_POLYNOMIAL_H_
#define _XPP_ZMP_COM_POLYNOMIAL_H_

#include <xpp/utils/polynomial_xd.h>

namespace xpp {
namespace zmp {

/** A fifth order spline that now holds some context information about the
  *  Center of Mass (CoM).
  */
// cmo move to polynomial class, doesn't introduce extra dependencies
class ComPolynomialHelpers {
public:
  using ComPolynomial  = xpp::utils::ComPolynomial;
  using VecPolynomials = std::vector<ComPolynomial>;
  using BaseLin2d      = xpp::utils::BaseLin2d;

  static BaseLin2d GetCOM(double t_global, const VecPolynomials& splines);
  static int GetPolynomialID(double t_global, const VecPolynomials& splines);
  static double GetTotalTime(const VecPolynomials& splines);
  static double GetLocalTime(double t_global, const VecPolynomials& splines);
  static BaseLin2d GetCOGxyAtPolynomial(int id, double t_local, const VecPolynomials& splines);
};


} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_COM_POLYNOMIAL_H_
