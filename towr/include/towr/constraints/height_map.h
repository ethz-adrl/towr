/**
 @file    height_map.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_HEIGHT_MAP_H_
#define XPP_OPT_INCLUDE_XPP_HEIGHT_MAP_H_

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/terrain_types.h>

namespace xpp {


// TODO: use ct_core/test/auto_diff functions to generate these derivatives
// which should be possible, as it is a very separated class
class HeightMap {
public:
  using Ptr         = std::shared_ptr<HeightMap>;
  using Vector3d    = Eigen::Vector3d;
  using Derivatives = std::vector<Coords2D>;

  static Ptr MakeTerrain(TerrainID type);
  virtual ~HeightMap () {};

  enum BasisVector { Normal, Tangent1, Tangent2 };

  virtual double GetHeight(double x, double y) const = 0;
  double GetDerivativeOfHeightWrt(Coords2D dim, double x, double y) const;
  Vector3d GetNormalizedBasis(BasisVector, double x, double y) const;
  Vector3d GetDerivativeOfNormalizedBasisWrt(BasisVector, Coords2D dim, double x, double y) const;

  double GetFrictionCoeff() const { return friction_coeff_; };
  virtual void SetGroundHeight(double height) {};

private:
  double friction_coeff_ = 0.5;

  // not normalized basis vectors of basis vector derivatives
  Vector3d GetBasisNotNormalized(BasisVector, double x, double y, const Derivatives& = {}) const;
  Vector3d GetNormalNotNormalized(double x, double y, const Derivatives& = {}) const;
  Vector3d GetTangent1NotNormalized(double x, double y, const Derivatives& = {}) const;
  Vector3d GetTangent2NotNormalized(double x, double y, const Derivatives& = {}) const;


  // first derivaties that must be implemented by the user
  virtual double GetHeightDerivWrtX(double x, double y) const { return 0.0; };
  virtual double GetHeightDerivWrtY(double x, double y) const { return 0.0; };

  // second derivatives wrt first letter, then second
  double GetSecondDerivativeOfHeightWrt(Coords2D dim1, Coords2D dim2, double x, double y) const;
  virtual double GetHeightDerivWrtXX(double x, double y) const { return 0.0; };
  virtual double GetHeightDerivWrtXY(double x, double y) const { return 0.0; };
  virtual double GetHeightDerivWrtYX(double x, double y) const { return 0.0; };
  virtual double GetHeightDerivWrtYY(double x, double y) const { return 0.0; };


  Vector3d GetDerivativeOfNormalizedVectorWrtNonNormalizedIndex(const Vector3d& non_normalized, int index) const;
};


class FlatGround : public HeightMap {
public:
  virtual double GetHeight(double x, double y)  const override { return height_; };

private:
  virtual void SetGroundHeight(double h) override { height_ = h; };
  double height_ = 0.0; // [m]
};

class Block : public HeightMap {
public:
  virtual double GetHeight(double x, double y)  const override;
  virtual double GetHeightDerivWrtX(double x, double y) const override;

private:
  double block_start = 1.5;
  double length_     = 3.5;
  double height_     = 0.8; // [m]

  double eps_ = 0.03; // approximate as slope
  const double slope_ = height_/eps_;
};



class Stairs : public HeightMap {
public:
  virtual double GetHeight(double x, double y) const override;

private:
  double first_step_start_  = 1.5;
  double first_step_width_  = 0.4;
  double height_first_step  = 0.2;
  double height_second_step = 0.4;
  double width_top = 1.0;
};




class Gap : public HeightMap {
public:
  virtual double GetHeight(double x, double y) const override;
  virtual double GetHeightDerivWrtX(double x, double y) const override;
  virtual double GetHeightDerivWrtXX(double x, double y) const override;

private:
  const double gap_start_ = 1.5;
  const double w = 1.0; // gap width or 0.5 for ANYmal
  const double h = 1.0; // 1.6 was

  const double slope_ = h/w;
  const double dx = w/2.0; // gap witdh 2
  const double xc = gap_start_ + dx; // gap center
  const double gap_end_x = gap_start_ + w;



  // generated with matlab
  // see /matlab/gap_model.m
  // coefficients of 2nd order polynomial
  // h = a*x^2 + b*x + c
  const double a = (4*h)/(w*w);
  const double b = -(8*h*xc)/(w*w);
  const double c = -(h*(w - 2*xc)*(w + 2*xc))/(w*w);
};


class Slope : public HeightMap {
public:
  virtual double GetHeight(double x, double y) const override;
  virtual double GetHeightDerivWrtX(double x, double y) const override;

private:
  const double slope_start_ = 1.0;
  const double up_length_   = 1.0;
  const double down_length_ = 1.0;
  const double height_center = 0.7;

  const double x_down_start_ = slope_start_+up_length_;
  const double x_flat_start_ = x_down_start_ + down_length_;
  const double slope_ = height_center/up_length_;
};



class Chimney : public HeightMap {
public:
  virtual double GetHeight(double x, double y) const override;
  virtual double GetHeightDerivWrtY(double x, double y) const override;

private:
  const double x_start_ = 1.5;
  const double length_  = 1.5;
  const double y_start_ = 0.5; // distance to start of slope from center at z=0
  const double slope_   = 3;   // 2 or 3

  const double x_end_ = x_start_+length_;
};


class ChimneyLR : public HeightMap {
public:
  virtual double GetHeight(double x, double y) const override;
  virtual double GetHeightDerivWrtY(double x, double y) const override;

private:
  const double x_start_ = 0.5;
  const double length_  = 1.0;
  const double y_start_ = 0.5; // distance to start of slope from center at z=0
  const double slope_   = 2;

  const double x_end1_ = x_start_+length_;
  const double x_end2_ = x_start_+2*length_;
};


} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_HEIGHT_MAP_H_ */
