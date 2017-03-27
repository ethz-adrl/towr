/**
 @file    endeffector_load.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 16, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTOR_LOAD_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTOR_LOAD_H_

#include <xpp/opt/endeffectors_motion.h>

namespace xpp {
namespace opt {

/** Parametrizes the load/force each endeffector is holding during the motion.
  *
  * The are the lambda values in the paper.
  */
class EndeffectorLoad {
public:
  using VectorXd = Eigen::VectorXd;
  using LoadParams = Endeffectors<double>;//std::vector<double>;
  static constexpr const char* ID = "convexity_lambdas";

  EndeffectorLoad ();
  virtual ~EndeffectorLoad ();
  void Init(const EndeffectorsMotion ee_motion, double dt, double T);

  void SetOptimizationVariables(const VectorXd& x);
  VectorXd GetOptimizationVariables() const;

  int GetOptVarCount() const;
  LoadParams GetLoadValues(double t) const;
  LoadParams GetLoadValuesIdx(int k) const;
  int GetNumberOfSegments() const;

  /** @param k the number of discretized node with lambda parameters.
    * @param ee which endeffector we are interested in.
    * @returns the index in the optimization vector where this value is stored
    */
  int IndexDiscrete(int k, EndeffectorID ee) const;
  int Index(double t, EndeffectorID ee) const;
  double GetTStart(int node) const;

private:
  int n_ee_; ///< number of endeffectors
  VectorXd lambdas_;
  double dt_;

  int GetSegment(double t) const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTOR_LOAD_H_ */
