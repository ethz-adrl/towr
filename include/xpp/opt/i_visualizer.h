/*
 * zmp_publisher.h
 *
 *  Created on: Apr 5, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_SRC_IVISUALIZER_H_
#define USER_TASK_DEPENDS_XPP_OPT_SRC_IVISUALIZER_H_

#include <memory>

namespace xpp {
namespace opt {

class NlpObserver;

/** @brief Interface to derive from to visualize the optimization results.
  *
  * "Program to an interface, not an implementation"
  */
class IVisualizer {
public:
  using NlpObserverPtr = std::shared_ptr<xpp::opt::NlpObserver>;

  IVisualizer() { observer_ = nullptr; };
  virtual ~IVisualizer () {};

  virtual void Visualize() const = 0;
  virtual void SetObserver(const NlpObserverPtr& observer) { observer_ = observer;};

protected:
  NlpObserverPtr observer_;
};

class DoNothingVisualizer : public IVisualizer {
  void Visualize() const override { /* this interface does nothing in this case; */ };
};

// compilation unit scope object that can be used as default initialization
// "static" : each .cc files that includes this header has it's own copy of this variable
typedef std::shared_ptr<IVisualizer> VisualizerPtr;
static VisualizerPtr do_nothing_visualizer = std::make_shared< DoNothingVisualizer>();

} /* namespace ros */
} /* namespace zmp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_IVISUALIZER_H_ */
