/**
 @file    endeffectors4.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 2, 2017
 @brief   Some definitions to use for a quadruped robot.
 */

#ifndef XPP_XPP_COMMON_INCLUDE_XPP_HYQ_ENDEFFECTORS4_H_
#define XPP_XPP_COMMON_INCLUDE_XPP_HYQ_ENDEFFECTORS4_H_

#include <xpp/endeffectors.h>
#include <map>

namespace xpp {

/** This is the order the hyq joints follow when converted to Eigen.
 */
enum LegID { RF, LF, LH, RH };

static const std::map<EndeffectorID, LegID> kMapOptToQuad {
  { EndeffectorID::E0,  LH},
  { EndeffectorID::E1,  LF},
  { EndeffectorID::E2,  RH},
  { EndeffectorID::E3,  RF},
};

static const std::map<LegID, EndeffectorID> kMapQuadToOpt {
  { kMapOptToQuad.at(EndeffectorID::E0) , EndeffectorID::E0},
  { kMapOptToQuad.at(EndeffectorID::E1) , EndeffectorID::E1},
  { kMapOptToQuad.at(EndeffectorID::E2) , EndeffectorID::E2},
  { kMapOptToQuad.at(EndeffectorID::E3) , EndeffectorID::E3},
};

} /* namespace xpp */

#endif /* XPP_XPP_COMMON_INCLUDE_XPP_HYQ_ENDEFFECTORS4_H_ */
