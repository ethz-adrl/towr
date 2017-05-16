/**
 @file    endeffector_load.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 16, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTOR_LOAD_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTOR_LOAD_H_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include <xpp/endeffectors.h>

#include <xpp/bound.h>
#include <xpp/opt/constraints/composite.h>

#include "contact_schedule.h"
#include "ee_force.h"

namespace xpp {
namespace opt {


class Force : public Composite {
public:
  using ComponentPtr = std::shared_ptr<EEForce>;
  using ComponentVec = std::vector<ComponentPtr>;
  using LoadParams = Endeffectors<double>;

  Force(double dt, const ContactSchedule&);
  virtual ~Force();

  LoadParams GetLoadValues(double t) const;
  int Index(double t, EndeffectorID ee) const;

private:
  ComponentVec ee_forces_; // derived class pointer to access ee specific functions
  std::vector<EndeffectorID> ee_ordered_;
};


// zmp_ remove all this
///** Parameterizes the load/force each endeffector is holding during the motion.
//  *
//  * The are the lambda values in the paper.
//  */
//class EndeffectorsForce : public Component {
//public:
//  using VectorXd   = Eigen::VectorXd;
//  using LoadParams = Endeffectors<double>;
//
//  EndeffectorsForce (int num_ee, double dt, const ContactSchedule&);
//  virtual ~EndeffectorsForce ();
//
//  virtual VectorXd GetValues() const override;
//  virtual void SetValues(const VectorXd& x) override;
//  virtual VecBound GetBounds() const override;
//
//  int Index(double t, EndeffectorID ee) const;
//
//  LoadParams GetLoadValues(double t) const;
////  int GetNumberOfSegments() const;
//
//
//private:
//  int n_ee_; ///< number of endeffectors
//  VectorXd lambdas_;
//  double dt_; ///< disretization interval [s]
//  double T_;  ///< total time [s]
//  int num_segments_;
//
//  /** @param k the number of discretized node with lambda parameters.
//    * @param ee which endeffector we are interested in.
//    * @returns the index in the optimization vector where this value is stored
//    */
//  int IndexDiscrete(int k, EndeffectorID ee) const;
//  LoadParams GetLoadValuesIdx(int k) const;
//  int GetSegment(double t) const;
//
//  /** Global time at beginning and end of segment */
//  double GetTimeCenterSegment(int segment_id) const;
//
//  void SetBounds(const ContactSchedule&, double max_load);
//  VecBound bounds_;
//
//};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTOR_LOAD_H_ */
