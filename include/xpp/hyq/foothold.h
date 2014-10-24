/**
@file    foothold.h
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   A foot position (x,y,z) and an associated leg.
 */

#ifndef FOOTHOLD_H_
#define FOOTHOLD_H_

#include "leg_data_map.h" // LegID, LegIDArray
#include <xpp/utils/geometric_structs.h>

#include <Eigen/Dense>

#include <array>
#include <map>
#include <iostream>
#include <fstream>

namespace xpp {
namespace hyq {


/**
@brief A foot position (x,y,z) and an associated leg.
*/
class Foothold
{

public:
  Eigen::Vector3d p;
  LegID leg;

  Foothold(Eigen::Vector3d _pos = Eigen::Vector3d::Zero(), LegID _leg = LF)
      : p(_pos), leg(_leg)
  {
  };

  Foothold(double x, double y, double z, LegID _leg = LF)
      : p(x, y, z), leg(_leg)
  {
  };

  /**
   * @brief Read in footsteps from a file and saves them in a vector.
   *
   * The file should be formatted as:
   *  LegID1    x-pos1    y-pos1   z-pos1
   *  LegID2    x-pos2    y-pos2   z-pos2
   *  .
   *  .
   *
   * @param file_name The absolute path of the file, eg. /home/jondow/...
   * @param n_steps Number of steps to read in from footstep file.
   * @return A vector of footholds.
   */
  static std::vector<Foothold> ReadFromFile(
      const std::string& file_name,
      size_t n_steps = std::numeric_limits<size_t>::max());

  bool operator==(const Foothold& rhs) const {
    return (p==rhs.p) && (leg==rhs.leg);
  }
};


inline std::vector<Foothold> Foothold::ReadFromFile(
    const std::string& file_name,
    size_t n_steps)
{
  std::vector<Foothold> steps;

  std::ifstream file;
  file.open(file_name);

  if (file.is_open()) {
    while (!file.eof() && steps.size() < n_steps) {

      int sl; double x, y, z;
      file >> sl >> x >> y >> z;

      if (z < -0.2 || 0.8 < z) {
        throw std::logic_error("z-coord " + std::to_string(z) + " of footstep file out of bounds.");
      }

      // -1 because old footstep files used enum starting at 1 for legs.
      Foothold f(x, y, z, static_cast<LegID>(sl-1));
      steps.push_back(f);
    }
    return steps;
  } else
    throw std::runtime_error("Couldn't read footstep file " + file_name);
}


inline std::ostream& operator<<(std::ostream& out, const Foothold& f)
{
  std::map<LegID, std::string> l {
    { LF, "LF" }, { RF, "RF" }, { LH, "LH" }, { RH, "RH" }
  };

  out << l[f.leg] << " : " << f.p.transpose();
  return out;
}

} // namespace hyq
} // namespace xpp

#endif /* FOOTHOLD_H_ */
