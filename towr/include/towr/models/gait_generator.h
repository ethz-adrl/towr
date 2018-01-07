/**
 @file    gait_generator.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 20, 2017
 @brief   Brief description
 */

#ifndef TOWR_MODELS_GAIT_GENERATOR_H_
#define TOWR_MODELS_GAIT_GENERATOR_H_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <xpp_states/endeffectors.h>

namespace towr {

class GaitGenerator {
public:
  using Ptr            = std::shared_ptr<GaitGenerator>;
  using VecTimes       = std::vector<double>;
  using FootDurations  = std::vector<VecTimes>;
  using ContactState   = xpp::EndeffectorsContact;
  using GaitInfo       = std::pair<VecTimes,std::vector<ContactState>>;
  using EndeffectorID  = xpp::EndeffectorID;

  GaitGenerator () = default;
  virtual ~GaitGenerator () = default;

  VecTimes GetContactSchedule(double T, EndeffectorID ee) const;
  VecTimes GetNormalizedContactSchedule(EndeffectorID ee) const;
  bool IsInContactAtStart(EndeffectorID ee) const;
//  std::vector<bool> GetContactSequence() const;

  //  contact_timings_.at(kMapIDToEE.at(L)) = {c+offset,f,c,f,c,f,c,f,c,f,c};
  //  contact_timings_.at(kMapIDToEE.at(R)) = {       c,f,c,f,c,f,c,f,c,f, c+offset};
  FootDurations GetContactSchedule() const;

  std::vector<std::string> GetEndeffectorNames() const;

  enum GaitCombos { Combo0=0, Combo1, Combo2, Combo3, Combo4, Combo5, Combo6, Combo7, Combo8, kNumCombos };


  virtual void SetCombo(GaitCombos combo) = 0;





protected:
  std::vector<double> times_;
  std::vector<ContactState> contacts_;
  std::map<std::string, EndeffectorID> map_id_to_ee_; // must be filled by derived class

  enum GaitTypes {Stand=0, Flight,
                  Walk1, Walk2, Walk2E,
                  Run2, Run2E, Run1, Run1E, Run3,
                  Hop1, Hop2, Hop3, Hop3E, Hop5, Hop5E,
                  kNumGaits};
  void SetGaits(const std::vector<GaitTypes>& gaits);

  /**
   * Removes the last phase that would transition to a new stride.
   * This is usually neccessary for a gait change.
   */
  GaitInfo RemoveTransition(const GaitInfo& g) const;

private:
  virtual GaitInfo GetGait(GaitTypes gait) const = 0;

};

} /* namespace towr */

#endif /* TOWR_MODELS_GAIT_GENERATOR_H_ */
