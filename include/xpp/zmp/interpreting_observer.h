/**
 @file    interpreting_observer.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 8, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_INTERPRETING_OBSERVER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_INTERPRETING_OBSERVER_H_

#include <xpp/zmp/i_observer.h>
#include <xpp/zmp/motion_structure.h>

#include <memory>
#include <vector>

namespace xpp {
namespace hyq {
class Foothold;
class SupportPolygonContainer;
}
}

namespace xpp {
namespace zmp {

class ComMotion;
class OptimizationVariables;

/** @brief General optimization values observer
  *
  * This class is responsible for observing the current values of the
  * optimization variables and returning them if needed.
  */
class InterpretingObserver : public IObserver {
public:
  using VecFoothold     = std::vector<xpp::hyq::Foothold>;
  using Contacts        = xpp::hyq::SupportPolygonContainer;
  using ContactsPtrU    = std::unique_ptr<Contacts>;
  using MotionPtrS      = std::shared_ptr<ComMotion>;

  InterpretingObserver (OptimizationVariables& subject);
  virtual ~InterpretingObserver ();

  void Init(const MotionStructure& structure,
            const ComMotion& com_motion,
            const Contacts& contacts);

  void Update() override;

  VecFoothold GetFootholds() const;
  VecFoothold GetStartStance() const;
  MotionPtrS GetComMotion() const;
  MotionStructure GetStructure() const;

private:
  MotionPtrS com_motion_;
  ContactsPtrU contacts_;
  MotionStructure motion_structure_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_INTERPRETING_OBSERVER_H_ */
