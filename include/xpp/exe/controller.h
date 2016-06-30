/*!
 * \file   controller.h
 * \author Alexander Winkler
 * \date   Oct 4, 2016
 * \brief  A controller that incorporates general functionalities used by
 *         all tasks such as time keeping, logging, failure handling.
 *
 *         Derive from this to create more specific tasks.
 */

#ifndef XPP_EXE_CONTROLLER_H_
#define XPP_EXE_CONTROLLER_H_

#include <memory>

namespace xpp {
namespace exe {

class RobotInterface;

class Controller {
public:
  typedef std::unique_ptr<RobotInterface> RobotInterfacePtr;

  explicit Controller();
  virtual ~Controller();

  bool GetReady();
  bool Run();
  bool Change();

  void AddRobot(RobotInterfacePtr robot);

protected:
  double Time() const { return time_; };
  void ResetTime();
  RobotInterfacePtr robot_;

  // using the template method pattern, where the base algorithm gets filled
  // in by specific implementations of the derived class
  virtual void GetReadyHook() = 0;
  virtual bool RunHook() = 0;

private:
  double time_;
};





} // namespace exe
} // namespace xpp

#endif /* XPP_EXE_CONTROLLER_H_ */
