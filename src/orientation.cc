/**
@file    orientations.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Utilities for degree/radians to quaternion conversions
 */

#include <xpp/utils/orientation.h>
#include <xpp/utils/geometric_structs.h>

namespace xpp {
namespace utils {

void Orientation::QuaternionToRPY(Eigen::Quaterniond q, Eigen::Vector3d& rpy)
{
  q.normalize();
	Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();

	rpy(X) = atan2f((float) rotation_matrix(2,1), (float) rotation_matrix(2,2));
	rpy(Y) = asinf((float) -rotation_matrix(2,0));
	rpy(Z) = atan2f((float) rotation_matrix(1,0), (float) rotation_matrix(0,0));
}


void Orientation::QuaternionToRPYDeg(Eigen::Quaterniond q, Eigen::Vector3d& rpy)
{
  q.normalize();
  Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();

  rpy(X) = atan2f((float) rotation_matrix(2,1), (float) rotation_matrix(2,2));
  rpy(Y) = asinf((float) -rotation_matrix(2,0));
  rpy(Z) = atan2f((float) rotation_matrix(1,0), (float) rotation_matrix(0,0));

  const static double kRadToDeg = M_PI / 180.0;
  rpy(X) /=  kRadToDeg;
  rpy(Y) /=  kRadToDeg;
  rpy(Z) /=  kRadToDeg;
}


Eigen::Quaterniond Orientation::RPYRadToQuaternion(double roll, double pitch, double yaw)
{
	double w = cos(roll / 2.0) * cos(pitch / 2.0) * cos(yaw / 2.0) + sin(roll / 2.0) * sin(pitch / 2.0) * sin(yaw / 2.0);
	double x = sin(roll / 2.0) * cos(pitch / 2.0) * cos(yaw / 2.0) - cos(roll / 2.0) * sin(pitch / 2.0) * sin(yaw / 2.0);
	double y = cos(roll / 2.0) * sin(pitch / 2.0) * cos(yaw / 2.0) + sin(roll / 2.0) * cos(pitch / 2.0) * sin(yaw / 2.0);
	double z = cos(roll / 2.0) * cos(pitch / 2.0) * sin(yaw / 2.0) - sin(roll / 2.0) * sin(pitch / 2.0) * cos(yaw / 2.0);
	Eigen::Quaterniond quaternion(w, x, y, z);

	quaternion.normalize(); // so Eigen::toRotationMatrix() yields correct results

	return quaternion;
}


Eigen::Quaterniond Orientation::RPYRadToQuaternion(const Eigen::Vector3d& rpy)
{
  return RPYRadToQuaternion(rpy(X), rpy(Y), rpy(Z));
}


Eigen::Quaterniond Orientation::RPYDegToQuaternion(double roll, double pitch, double yaw)
{
  const static double kRadToDeg = M_PI / 180.0;
  return Orientation::RPYRadToQuaternion(roll*kRadToDeg, pitch*kRadToDeg, yaw*kRadToDeg);
}

} // namespace utils
} // namespace xpp
