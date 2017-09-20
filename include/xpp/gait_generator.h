/**
 @file    gait_generator.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 20, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_GAIT_GENERATOR_H_
#define XPP_OPT_INCLUDE_XPP_GAIT_GENERATOR_H_

#include <vector>

#include "endeffectors.h"

namespace xpp {
namespace opt {

enum GaitTypes {Stand=0, Flight, Walk1, Walk2, Run2, Run1,  Run3, Hop1, Hop2, kNumGaits};
static std::map<int, std::string> gait_names =
{
  {Stand,   "Stand"},
  {Flight,  "Flight"},
  {Walk1,   "Walk"},
  {Walk2,   "WalkOverlap"},
  {Run1,    "Trot"},
  {Run2,    "TrotFly"},
  {Run3,    "Pace"},
  {Hop1,    "Bound"},
  {Hop2,    "Pronk"},
};

class GaitGenerator {
public:
  using VecTimes = std::vector<double>;
  using FootDurations  = std::vector<VecTimes>;
  using ContactState   = EndeffectorsBool;
  using GaitInfo       = std::pair<VecTimes,std::vector<ContactState>>;

  GaitGenerator ();
  virtual ~GaitGenerator ();

  VecTimes GetContactSchedule(EndeffectorID ee) const;
  VecTimes GetNormalizedContactSchedule(EndeffectorID ee) const;

  FootDurations GetContactSchedule() const;


  void SetGaits(const std::vector<GaitTypes>& gaits);

protected:
  std::vector<double> times_;
  std::vector<ContactState> contacts_;

private:
  virtual GaitInfo GetGait(GaitTypes gait) const = 0;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_GAIT_GENERATOR_H_ */
