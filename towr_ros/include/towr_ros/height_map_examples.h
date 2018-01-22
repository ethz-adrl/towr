/*
 * height_map_examples.h
 *
 *  Created on: Jan 22, 2018
 *      Author: winklera
 */

#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_EXAMPLES_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_EXAMPLES_H_

#include <towr/height_map.h>

namespace towr {

/**
 * @brief Terrains IDs corresponding to a draw function in xpp_vis and a
 * detailed (gradient supplying) function in the optimizer.
 */
enum TerrainID { FlatID=0,
                 BlockID,
                 StairsID,
                 GapID,
                 SlopeID,
                 ChimneyID,
                 ChimneyLRID,
                 K_TERRAIN_COUNT };

class HeightMapFactory {
public:
  static HeightMap::Ptr MakeTerrain(TerrainID type, double ground_height);
};



class FlatGround : public HeightMap {
public:
  FlatGround(double height);
  virtual double GetHeight(double x, double y)  const override { return height_; };

private:
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


} /* namespace towr */

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_EXAMPLES_H_ */
