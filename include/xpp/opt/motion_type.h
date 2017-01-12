/**
@file    motion_type.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jan 11, 2017
@brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_

//#include <xpp/opt/cost_container.h>
#include <xpp/utils/endeffectors.h>
#include <memory>

namespace xpp {
namespace opt {

enum MotionTypeID { WalkID, TrottID, CamelID, BoundID };

class MotionType {
public:
//  using CostContainerPtr = std::shared_ptr<CostContainer>;
  using MotionTypePtr  = std::shared_ptr<MotionType>;
  using EEID           = xpp::utils::EndeffectorID;
  using Swinglegs      = std::vector<EEID>;
  using SwingLegCycle  = std::vector<Swinglegs>;

  virtual ~MotionType();

//  virtual void SetCostTerms(CostContainerPtr&) const =0;

  virtual SwingLegCycle GetOneCycle() const = 0;

  MotionTypeID id_;
  double t_phase_;
  double max_step_length_;

  double weight_com_motion_cost_;
  double weight_range_of_motion_cost_;
  double weight_polygon_center_cost_;

  static MotionTypePtr MakeMotion(MotionTypeID);
};

} // namespace opt
} // namespace hyq

namespace xpp {
namespace hyq {

class Walk : public opt::MotionType {
public:
  Walk();

  virtual SwingLegCycle GetOneCycle() const;

//  virtual void SetCostTerms(CostContainerPtr&) const override;
};

class Trott : public opt::MotionType {
public:
  Trott();

  virtual SwingLegCycle GetOneCycle() const;

//  virtual void SetCostTerms(CostContainerPtr&) const override;
};

class Camel : public opt::MotionType {
public:
  Camel();

  virtual SwingLegCycle GetOneCycle() const;

//  virtual void SetCostTerms(CostContainerPtr&) const override;
};

class Bound : public opt::MotionType {
public:
  Bound();

  virtual SwingLegCycle GetOneCycle() const;

//  virtual void SetCostTerms(CostContainerPtr&) const override;
};

} // namespace opt
} // namespace hyq

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_ */
