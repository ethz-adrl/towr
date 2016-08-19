/*
 * zmp_publisher.h
 *
 *  Created on: Apr 5, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_SRC_IVISUALIZER_H_
#define USER_TASK_DEPENDS_XPP_OPT_SRC_IVISUALIZER_H_

namespace xpp {
namespace zmp {

/** @brief Interface to derive from to visualize the optimization results.
  *
  * "Program to an interface, not an implementation"
  */
class IVisualizer {
public:
  IVisualizer() {};
  virtual ~IVisualizer () {};

  virtual void Visualize() const = 0;
};

class DoNothingVisualizer : public IVisualizer {
  void Visualize() const override { /* this interface does nothing in this case; */ };
};

// compilation unit scope object that can be used as default initialization
// "static" : each .cc files that includes this header has it's own copy of this variable
static DoNothingVisualizer do_nothing_visualizer;

} /* namespace ros */
} /* namespace zmp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_IVISUALIZER_H_ */
