/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_MODELS_GAIT_GENERATOR_H_
#define TOWR_MODELS_GAIT_GENERATOR_H_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace towr {

/**
 * @brief Generates endeffector phase durations for predefined gait styles.
 *
 * These gaits (e.g. quadruped trotting, biped walking) are used to
 * initialize the towr optimization problem.
 */
class GaitGenerator {
public:
  using Ptr           = std::shared_ptr<GaitGenerator>;
  using VecTimes      = std::vector<double>;
  using FootDurations = std::vector<VecTimes>;
  using ContactState  = std::vector<bool>;
  using GaitInfo      = std::pair<VecTimes,std::vector<ContactState>>;
  using EE            = uint;
  enum GaitCombos    { C0=0, C1, C2, C3, C4, C5, C6, C7, C8, kNumCombos };
  enum GaitTypes     {Stand=0, Flight,
                      Walk1, Walk2, Walk2E,
                      Run2, Run2E, Run1, Run1E, Run3,
                      Hop1, Hop2, Hop3, Hop3E, Hop5, Hop5E,
                      kNumGaits};

  GaitGenerator () = default;
  virtual ~GaitGenerator () = default;

  /**
   * @returns the swing and stance durations for the set gait.
   * @param ee  endeffector for which the phase durations are desired.
   * @param T   total time for all phases, durations are scaled by that.
   */
  VecTimes GetPhaseDurations(double T, EE ee) const;

  /**
   * @returns true if the foot is initially in contact with the environment.
   * @param ee  The endeffector/foot/hand.
   */
  bool IsInContactAtStart(EE ee) const;

  /**
   * @brief Sets a specific sequence of gaits.
   *
   * The derived class decides what each combo maps to. This function then fills
   * the times_ and contacts_ variables accordingly.
   */
  virtual void SetCombo(GaitCombos combo) = 0;

  /**
   * @returns The endefftor name (e.g. "left_foot") corresonding to each index.
   */
  std::vector<std::string> GetEndeffectorNames() const;


protected:
  /// Phase times for the complete robot during which no contact state changes.
  std::vector<double> times_;

  /**
   * The contact state for the complete robot. The size of this vector must
   * be equal to the above times_.
   */
  std::vector<ContactState> contacts_;

  /// Sets the times_ and contacts_ variables according to the gaits.
  void SetGaits(const std::vector<GaitTypes>& gaits);

  /**
   * Removes the last phase that would transition to a new stride.
   * This is usually necessary for a gait change.
   */
  GaitInfo RemoveTransition(const GaitInfo& g) const;

  /// mapping from endeffector name to index, filled by derived class
  std::map<std::string, EE> map_id_to_ee_;

private:
  FootDurations GetPhaseDurations() const;
  virtual GaitInfo GetGait(GaitTypes gait) const = 0;
  VecTimes GetNormalizedPhaseDurations(EE ee) const;
};

} /* namespace towr */

#endif /* TOWR_MODELS_GAIT_GENERATOR_H_ */
