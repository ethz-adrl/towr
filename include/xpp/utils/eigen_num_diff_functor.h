/*
 * eigen_num_diff_functor.h
 *
 *  Created on: Apr 11, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_EIGEN_NUM_DIFF_FUNCTOR_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_EIGEN_NUM_DIFF_FUNCTOR_H_

#include <unsupported/Eigen/NumericalDiff>

namespace xpp {
namespace utils {


template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
class EigenNumDiffFunctor {
public:
  typedef _Scalar Scalar;
  enum {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

  explicit EigenNumDiffFunctor(int inputs, int values) : m_inputs(inputs), m_values(values) {}
  virtual ~EigenNumDiffFunctor () {};

  virtual int operator() (const InputType& x_coeff, ValueType& obj_value) const = 0;

  int m_inputs, m_values;
  int inputs() const { return m_inputs; }
  int values() const { return m_values; }
};


} /* namespace utils */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_EIGEN_NUM_DIFF_FUNCTOR_H_ */
