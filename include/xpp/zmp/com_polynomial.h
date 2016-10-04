/**
@file   com_polynomial.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2015
@brief  Declares CoeffValues, PolynomialFifthOrder and ComPolynomial
 */

#ifndef _XPP_ZMP_COM_POLYNOMIAL_H_
#define _XPP_ZMP_COM_POLYNOMIAL_H_

#include <xpp/utils/base_state.h>
#include <xpp/utils/polynomial_xd.h>

namespace cmo {namespace ros{ class RosHelpers; }};

namespace xpp {
namespace zmp {

/** A fifth order spline that now holds some context information about the
  *  Center of Mass (CoM).
  */
class ComPolynomial : public xpp::utils::PolynomialXd<xpp::utils::QuinticPolynomial,
                                                      xpp::utils::kDim2d,
                                                      xpp::utils::BaseLin2d>
{
public:
  using VecPolynomials = std::vector<ComPolynomial>;
  using BaseLin2d = Point;

  ComPolynomial();
  ComPolynomial(uint id, double duration);
  virtual ~ComPolynomial() {};

  uint GetId()            const { return id_; };

  static BaseLin2d GetCOM(double t_global, const VecPolynomials& splines);
  static int GetPolynomialID(double t_global, const VecPolynomials& splines);
  static double GetTotalTime(const VecPolynomials& splines);
  static double GetLocalTime(double t_global, const VecPolynomials& splines);
  static BaseLin2d GetCOGxyAtPolynomial(int id, double t_local, const VecPolynomials& splines);

private:
  uint id_; // to identify the order relative to other polynomials
  // cmo remove duration here, alrady in spliner2d

  friend std::ostream& operator<<(std::ostream& out, const ComPolynomial& tr);
  friend struct cmo::ros::RosHelpers;
};


} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_COM_POLYNOMIAL_H_
