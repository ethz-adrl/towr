/*
 * geometric_structs.cc
 *
 *  Created on: Apr 28, 2016
 *      Author: winklera
 */

#include<xpp/utils/geometric_structs.h>


namespace xpp {
namespace utils {


Point3d Point2d::Make3D() const
{
  Point3d p3d;
  p3d.p.segment<kDim2d>(X) = p;
  p3d.v.segment<kDim2d>(X) = v;
  p3d.a.segment<kDim2d>(X) = a;
  return p3d;
}


Point2d Point3d::Get2D() const
{
  Point2d p2d;
  p2d.p = p.segment<kDim2d>(X);
  p2d.v = v.segment<kDim2d>(X);
  p2d.a = a.segment<kDim2d>(X);
  return p2d;
}


VecScalar operator*(double d, const VecScalar& rhs)
{
  return VecScalar(d*rhs.v, d*rhs.s);
}


VecScalar MatVec::GetRow(int row) const
{
  return VecScalar(M.row(row), v[row]);
}


void MatVec::operator<<(const MatVec& rhs)
{
  assert((M.cols()==0 && M.rows()==0) || (M.cols() == rhs.M.cols()));

  M.conservativeResize(M.rows() + rhs.M.rows(), rhs.M.cols());
  M.bottomRows(rhs.M.rows()) = rhs.M;

  v.conservativeResize(v.rows() + rhs.v.rows());
  v.tail(rhs.v.rows()) = rhs.v;
}


void MatVec::WriteRow(const VecScalar& val, size_t row)
{
  assert((val.v.cols()==M.cols()) && (row<M.rows()));

  M.row(row) = val.v;
  v[row]     = val.s;
}



}
}
