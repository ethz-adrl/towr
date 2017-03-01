/**
 @file    i_visualizer.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Declares the base class IVisualizer
 */

#ifndef XPP_XPP_OPT_SRC_IVISUALIZER_H_
#define XPP_XPP_OPT_SRC_IVISUALIZER_H_

#include "motion_structure.h"
#include <xpp/utils/eigen_std_conversions.h>
#include <memory>

namespace xpp {
namespace opt {

class ComMotion;
class OptimizationVariables;
class MotionParameters;

/** @brief Interface to derive from to visualize the optimization results.
  *
  * "Program to an interface, not an implementation"
  */
class IVisualizer {
public:
  using MotionPtrS               = std::shared_ptr<ComMotion>;
  using OptimizationVariablesPtr = std::shared_ptr<OptimizationVariables>;
  using VecFoothold              = utils::StdVecEigen2d;
  using MotionParamsPtr          = std::shared_ptr<MotionParameters>;

  IVisualizer();
  virtual ~IVisualizer ();

  /** This method can be used by derived class to visualize in ROS.
    */
  virtual void Visualize() const = 0;

  void SetMotionStructure(const MotionStructure&);
  void SetComMotion(const MotionPtrS&);
  void SetOptimizationVariables(const OptimizationVariablesPtr&);

  void SetMotionParameters(const MotionParamsPtr& params);

protected:
  const MotionPtrS GetComMotion() const;
  MotionStructure GetMotionStructure() const;
  VecFoothold GetContacts() const;

  MotionParamsPtr motion_params_;

private:
  OptimizationVariablesPtr opt_variables_;
  MotionPtrS com_motion_;
  MotionStructure motion_structure_;
};

class DoNothingVisualizer : public IVisualizer {
  void Visualize() const override { /* this interface does nothing in this case; */ };
};

// compilation unit scope object that can be used as default initialization
// "static" : each .cc files that includes this header has it's own copy of this variable
typedef std::shared_ptr<IVisualizer> VisualizerPtr;
static VisualizerPtr do_nothing_visualizer = std::make_shared< DoNothingVisualizer>();

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_SRC_IVISUALIZER_H_ */
