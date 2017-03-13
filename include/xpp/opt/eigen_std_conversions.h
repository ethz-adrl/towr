/**
@file    eigen_std_conversions.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Sep 28, 2016
@brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_EIGEN_STD_CONVERSIONS_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_EIGEN_STD_CONVERSIONS_H_

#include <Eigen/Dense>
#include <Eigen/StdVector> // for std::eigen vector

namespace xpp {
namespace opt {

typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > StdVecEigen2d;
Eigen::VectorXd ConvertStdToEig(const StdVecEigen2d& vec_std_2d);
StdVecEigen2d ConvertEigToStd (const Eigen::VectorXd& vec_eigen);

inline Eigen::VectorXd
ConvertStdToEig(const StdVecEigen2d& vec_std_2d)
{
  const int kDim2d = 2; // X,Y
  int n_steps = vec_std_2d.size();

  Eigen::VectorXd vec(kDim2d*n_steps);
  int c=0;
  for (int i=0; i<n_steps; ++i)
  {
    vec[c++] = vec_std_2d.at(i).x();
    vec[c++] = vec_std_2d.at(i).y();
  }
  return vec;
}

inline StdVecEigen2d
ConvertEigToStd (const Eigen::VectorXd& vec_eig)
{
  const int kDim2d = 2; // X,Y
  StdVecEigen2d fooothold_vec(vec_eig.rows()/2.);
  for (int i=0; i<fooothold_vec.size(); ++i) {
    fooothold_vec.at(i) = vec_eig.segment<kDim2d>(kDim2d*i);
  }

  return fooothold_vec;
}

} // namespace opt
} // namespace xpp

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_UTILS_EIGEN_STD_CONVERSIONS_H_ */
