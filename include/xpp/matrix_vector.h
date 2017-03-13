/**
@file    matrix_vector.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Sep 28, 2016
@brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_UTILS_MATRIX_VECTOR_H_
#define XPP_OPT_INCLUDE_XPP_UTILS_MATRIX_VECTOR_H_

#include <Eigen/Dense>

namespace xpp {
namespace opt {

struct VecScalar {
  Eigen::RowVectorXd v;
  double s;
  VecScalar() {}
  VecScalar(int rows)
      :v(Eigen::RowVectorXd::Zero(rows)),
       s(0.0)
  {}
  VecScalar(const Eigen::RowVectorXd& _v, double _s)
      :v(_v),
       s(_s)
  {}
  VecScalar operator-(const VecScalar& rhs) const
  {
    return VecScalar(v-rhs.v, s-rhs.s);
  }
};

VecScalar operator*(double d, const VecScalar& rhs);

struct MatVec {
  Eigen::MatrixXd M;
  Eigen::VectorXd v;
  MatVec() {}
  MatVec(int rows, int cols)
      :M(Eigen::MatrixXd::Zero(rows, cols)),
       v(Eigen::VectorXd::Zero(rows))
  {}
  VecScalar GetRow(int r) const;
  void operator<<(const MatVec& rhs);
  void WriteRow(const VecScalar& val, size_t row) ;
};


inline VecScalar operator*(double d, const VecScalar& rhs)
{
  return VecScalar(d*rhs.v, d*rhs.s);
}

inline VecScalar MatVec::GetRow(int row) const
{
  return VecScalar(M.row(row), v[row]);
}

inline void MatVec::operator<<(const MatVec& rhs)
{
  assert((M.cols()==0 && M.rows()==0) || (M.cols() == rhs.M.cols()));

  M.conservativeResize(M.rows() + rhs.M.rows(), rhs.M.cols());
  M.bottomRows(rhs.M.rows()) = rhs.M;

  v.conservativeResize(v.rows() + rhs.v.rows());
  v.tail(rhs.v.rows()) = rhs.v;
}

inline void MatVec::WriteRow(const VecScalar& val, size_t row)
{
  assert((val.v.cols()==M.cols()) && (row<M.rows()));

  M.row(row) = val.v;
  v[row]     = val.s;
}

} // namespace opt
} // namespace xpp

#endif /* XPP_OPT_INCLUDE_XPP_UTILS_MATRIX_VECTOR_H_ */
