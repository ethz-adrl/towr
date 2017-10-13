/**
@file    matrix_vector.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Sep 28, 2016
@brief   Allows for easier representation of linear equations.
         of the form M*x + v.
 */

#ifndef XPP_OPT_INCLUDE_XPP_UTILS_MATRIX_VECTOR_H_
#define XPP_OPT_INCLUDE_XPP_UTILS_MATRIX_VECTOR_H_

#include <stddef.h>
#include <Eigen/Dense>


namespace xpp {
namespace opt {

struct VecScalar {
  Eigen::RowVectorXd v;
  double s;
  VecScalar() {};
  VecScalar(const Eigen::RowVectorXd& _v, double _s)
      :v(_v),
       s(_s)
  {}
  VecScalar operator-(const VecScalar& rhs) const
  {
    return VecScalar(v-rhs.v, s-rhs.s);
  }
};

struct MatVec {
  Eigen::MatrixXd M;
  Eigen::VectorXd v;
  MatVec() {}
  MatVec(int rows, int cols)
      :M(Eigen::MatrixXd::Zero(rows, cols)),
       v(Eigen::VectorXd::Zero(rows))
  {};
  void WriteRow(const VecScalar& val, size_t row) ;
};

inline void MatVec::WriteRow(const VecScalar& val, size_t row)
{
  M.row(row) = val.v;
  v[row]     = val.s;
}

} // namespace opt
} // namespace xpp

#endif /* XPP_OPT_INCLUDE_XPP_UTILS_MATRIX_VECTOR_H_ */
