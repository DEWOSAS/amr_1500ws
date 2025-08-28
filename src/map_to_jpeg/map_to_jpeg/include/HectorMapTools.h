//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.
// ... (license content remains unchanged)
//=================================================================================================

#ifndef __HectorMapTools_h_
#define __HectorMapTools_h_

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>

#include <Eigen/Core>
#include <memory>
#include <vector>
#include <cmath>

class HectorMapTools{
public:

  template<typename ConcreteScalar>
  class CoordinateTransformer{
  public:

    CoordinateTransformer() {}

    CoordinateTransformer(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> &map) {
      this->setTransforms(*map);
    }

    void setTransforms(const nav_msgs::msg::OccupancyGrid& map) {
      this->setTransforms(map.info);
    }

    void setTransforms(const nav_msgs::msg::MapMetaData& meta) {
      origo_ = Eigen::Matrix<ConcreteScalar, 2, 1>(meta.origin.position.x, meta.origin.position.y);
      scale_ = static_cast<ConcreteScalar>(meta.resolution);
      inv_scale_ = static_cast<ConcreteScalar>(1.0f / meta.resolution);
    }

    void setTransformsBetweenCoordSystems(const Eigen::Matrix<ConcreteScalar, 2, 1>& origoCS1, const Eigen::Matrix<ConcreteScalar, 2, 1>& endCS1,
                                          const Eigen::Matrix<ConcreteScalar, 2, 1>& origoCS2, const Eigen::Matrix<ConcreteScalar, 2, 1>& endCS2) {
      Eigen::Matrix<ConcreteScalar, 2, 1> map_t_geotiff_x_factors = getLinearTransform(Eigen::Vector2f(origoCS1[0], endCS1[0]), Eigen::Vector2f(origoCS2[0], endCS2[0]));
      Eigen::Matrix<ConcreteScalar, 2, 1> map_t_geotiff_y_factors = getLinearTransform(Eigen::Vector2f(origoCS1[1], endCS1[1]), Eigen::Vector2f(origoCS2[1], endCS2[1]));

      origo_.x() = map_t_geotiff_x_factors[1];
      origo_.y() = map_t_geotiff_y_factors[1];

      scale_ = map_t_geotiff_x_factors[0];
      inv_scale_ = 1.0 / scale_;
    }

    Eigen::Matrix<ConcreteScalar, 2, 1> getC1Coords(const Eigen::Matrix<ConcreteScalar, 2, 1>& mapCoords) const {
      return origo_ + (mapCoords * scale_);
    }

    Eigen::Matrix<ConcreteScalar, 2, 1> getC2Coords(const Eigen::Matrix<ConcreteScalar, 2, 1>& worldCoords) const {
      return ((worldCoords - origo_) * inv_scale_);
    }

    ConcreteScalar getC1Scale(float c2_scale) const {
      return scale_ * c2_scale;
    }

    ConcreteScalar getC2Scale(float c1_scale) const {
      return inv_scale_ * c1_scale;
    }

  protected:

    Eigen::Matrix<ConcreteScalar, 2, 1> getLinearTransform(const Eigen::Matrix<ConcreteScalar, 2, 1>& coordSystem1, const Eigen::Matrix<ConcreteScalar, 2, 1>& coordSystem2) {
      ConcreteScalar scaling = (coordSystem2[0] - coordSystem2[1]) / (coordSystem1[0] - coordSystem1[1]);
      ConcreteScalar translation = coordSystem2[0] - coordSystem1[0] * scaling;
      return Eigen::Vector2f(scaling, translation);
    }

    Eigen::Matrix<ConcreteScalar, 2, 1> origo_;
    ConcreteScalar scale_;
    ConcreteScalar inv_scale_;
  };

  class DistanceMeasurementProvider {
  public:
    DistanceMeasurementProvider() {}

    void setMap(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> &map) {
      map_ptr_ = map;
      world_map_transformer_.setTransforms(*map_ptr_);
    }

    float getDist(const Eigen::Vector2f& begin_world, const Eigen::Vector2f& end_world, Eigen::Vector2f* hitCoords = nullptr) {
      Eigen::Vector2i end_point_map;

      Eigen::Vector2i begin_map(world_map_transformer_.getC2Coords(begin_world).cast<int>());
      Eigen::Vector2i end_map(world_map_transformer_.getC2Coords(end_world).cast<int>());
      float dist = checkOccupancyBresenhami(begin_map, end_map, &end_point_map);

      if (hitCoords != nullptr){
        *hitCoords = world_map_transformer_.getC1Coords(end_point_map.cast<float>());
      }

      return world_map_transformer_.getC1Scale(dist);
    }

    float checkOccupancyBresenhami(const Eigen::Vector2i& beginMap, const Eigen::Vector2i& endMap, Eigen::Vector2i* hitCoords = nullptr, unsigned int max_length = UINT_MAX) {
      int x0 = beginMap[0];
      int y0 = beginMap[1];

      int sizeX = map_ptr_->info.width;
      int sizeY = map_ptr_->info.height;

      if ((x0 < 0) || (x0 >= sizeX) || (y0 < 0) || (y0 >= sizeY)) {
        return -1.0f;
      }

      int x1 = endMap[0];
      int y1 = endMap[1];

      if ((x1 < 0) || (x1 >= sizeX) || (y1 < 0) || (y1 >= sizeY)) {
        return -1.0f;
      }

      int dx = x1 - x0;
      int dy = y1 - y0;

      unsigned int abs_dx = std::abs(dx);
      unsigned int abs_dy = std::abs(dy);

      int offset_dx = dx > 0 ? 1 : -1;
      int offset_dy = (dy > 0 ? 1 : -1) * sizeX;

      unsigned int startOffset = y0 * sizeX + x0;

      int end_offset;

      if(abs_dx >= abs_dy){
        int error_y = abs_dx / 2;
        end_offset = bresenham2D(abs_dx, abs_dy, error_y, offset_dx, offset_dy, startOffset, 5000);
      }else{
        int error_x = abs_dy / 2;
        end_offset = bresenham2D(abs_dy, abs_dx, error_x, offset_dy, offset_dx, startOffset, 5000);
      }

      if (end_offset != -1){
        Eigen::Vector2i end_coords(end_offset % sizeY, end_offset / sizeY);
        int distMap = ((beginMap - end_coords).cast<float>()).norm();

        if (hitCoords != nullptr){
          *hitCoords = end_coords;
        }

        return distMap;
      }

      return -1.0f;
    }

    int bresenham2D(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b,
                    unsigned int offset, unsigned int max_length) {
      unsigned int end = std::min(max_length, abs_da);
      const std::vector<int8_t>& data = map_ptr_->data;

      for (unsigned int i = 0; i < end; ++i){
        if (data[offset] == 100){
          return static_cast<int>(offset);
        }
        offset += offset_a;
        error_b += abs_db;
        if(static_cast<unsigned int>(error_b) >= abs_da){
          offset += offset_b;
          error_b -= abs_da;
        }
      }
      return -1;
    }

  protected:
    CoordinateTransformer<float> world_map_transformer_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> map_ptr_;
  };

  static bool getMapExtends(const nav_msgs::msg::OccupancyGrid& map, Eigen::Vector2i& topLeft, Eigen::Vector2i& bottomRight) {
    int lowerStart = -1;
    int upperStart = 10000000;

    int xMaxTemp = lowerStart;
    int yMaxTemp = lowerStart;
    int xMinTemp = upperStart;
    int yMinTemp = upperStart;

    int width = map.info.width;
    int height = map.info.height;

    for (int y = 0; y < height; ++y){
      for (int x = 0; x < width; ++x){
        if (map.data[x + y * width] != -1){
          xMaxTemp = std::max(xMaxTemp, x);
          xMinTemp = std::min(xMinTemp, x);
          yMaxTemp = std::max(yMaxTemp, y);
          yMinTemp = std::min(yMinTemp, y);
        }
      }
    }

    if ((xMaxTemp != lowerStart) && (yMaxTemp != lowerStart) && (xMinTemp != upperStart) && (yMinTemp != upperStart)) {
      topLeft = Eigen::Vector2i(xMinTemp, yMinTemp);
      bottomRight = Eigen::Vector2i(xMaxTemp + 1, yMaxTemp + 1);
      return true;
    } else {
      return false;
    }
  }
};

#endif
