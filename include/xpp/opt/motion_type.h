/**
@file    motion_type.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jan 11, 2017
@brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_

//#include <xpp/opt/cost_container.h>

namespace xpp {
namespace opt {

enum MotionTypeID { WalkID, TrottID };

class MotionType {
public:
//  using CostContainerPtr = std::shared_ptr<CostContainer>;

  virtual ~MotionType();

//  virtual void SetCostTerms(CostContainerPtr&) const =0;

  MotionTypeID id_;
  int swinglegs_per_phase_;
  double t_phase_;
  double max_step_length_;
};

class Walk : public MotionType {
public:
  Walk();

//  virtual void SetCostTerms(CostContainerPtr&) const override;
};

class Trott : public MotionType {
public:
  Trott();

//  virtual void SetCostTerms(CostContainerPtr&) const override;
};

}
}

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_ */
