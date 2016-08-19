/**
 @file    sample_controller.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 19, 2016
 @brief   Declares a sample controller as example.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_EXE_SAMPLE_CONTROLLER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_EXE_SAMPLE_CONTROLLER_H_

#include <xpp/exe/controller.h>

namespace xpp {
namespace exe {

/** A sample controller to demonstrate how to create a controller using the interface.
  *
  */
class SampleController : public Controller {
public:


  SampleController ();
  virtual ~SampleController ();

  void GetReadyHook() override;
  bool RunHook() override;
};

} /* namespace exe */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_EXE_SAMPLE_CONTROLLER_H_ */
