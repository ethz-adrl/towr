/*!
 * \file   controller.h
 * \author Alexander Winkler
 * \date   Oct 4, 2014
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

/*!
 * \class SlTask
 * \brief Abstract class that provides basic functionality
 *
 * Supplies current time, logging and config file reading
 */
class Controller {
public:
  typedef std::unique_ptr<RobotInterface> RobotInterfacePtr;

  bool GetReady();
  bool Run();
  bool Change();

  void AddRobot(RobotInterfacePtr robot);

  virtual ~Controller();
protected:
  explicit Controller();

  double Time() const { return time_; };

  // using the template method pattern, where the base algorithm gets filled
  // in by specific implementations of the derived class
  virtual void InitDerivedClassMembers() = 0;
  virtual bool DoSomething() = 0;

  void ResetTime();
  RobotInterfacePtr robot_;

private:

  double time_;
};





} // namespace exe
} // namespace xpp

#endif /* XPP_EXE_CONTROLLER_H_ */
