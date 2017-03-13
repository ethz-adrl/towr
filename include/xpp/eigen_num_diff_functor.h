/*
 * eigen_num_diff_functor.h
 *
 *  Created on: Apr 11, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_EIGEN_NUM_DIFF_FUNCTOR_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_EIGEN_NUM_DIFF_FUNCTOR_H_

#include <unsupported/Eigen/NumericalDiff>
#include <iostream>

namespace xpp {
namespace opt {

template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
class EigenNumDiffFunctor {
protected:
  typedef _Scalar Scalar;
  enum {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

  explicit EigenNumDiffFunctor() : m_inputs(0), m_values(0) {}
  explicit EigenNumDiffFunctor(int inputs, int values) : m_inputs(inputs), m_values(values) {}
  virtual ~EigenNumDiffFunctor () {};

  virtual int operator() (const InputType& x_coeff, ValueType& obj_value) const = 0;

  int inputs() const { return m_inputs; }
  int values() const { return m_values; }

  void set_inputs(int inputs) { m_inputs = inputs; }
  void set_values(int values) { m_values = values; }

private:
  int m_inputs, m_values;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_EIGEN_NUM_DIFF_FUNCTOR_H_ */
