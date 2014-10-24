/**
@file    orientations.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Utilities for degree/radians to quaternion conversions
 */

#ifndef XPPORIENTATION_H
#define XPPORIENTATION_H

#include <Eigen/Dense>

namespace xpp {
namespace utils {

/**
 * @class Orientation
 * @brief Class for converting the orientation between roll, pitch and yaw angles, rotation matrix and quaternions
 */
class Orientation
{
	public:
		/**
		 * @brief Gets the roll, pitch and yaw angles
		 * @param double& roll Roll angle
		 * @param double& pitch Pitch angle
		 * @param double& yaw Yaw angle
		 */
		static void QuaternionToRPY(Eigen::Quaterniond q, Eigen::Vector3d& rpy);
    static void QuaternionToRPYDeg(Eigen::Quaterniond q, Eigen::Vector3d& rpy);

		/**
		 * @brief Returns the quaternion from roll, pitch and yaw angles
		 * @param double roll Roll angle
		 * @param pitch Pitch angle
		 * @param yaw Yaw angle
		 */
		static Eigen::Quaterniond RPYRadToQuaternion(double roll_rad, double pitch_rad, double yaw_rad);
		static Eigen::Quaterniond RPYDegToQuaternion(double roll_deg, double pitch_deg, double yaw_deg);
		static Eigen::Quaterniond RPYRadToQuaternion(const Eigen::Vector3d& rpy_rad);

};

} // namespace utils
} // namespace xpp

#endif
