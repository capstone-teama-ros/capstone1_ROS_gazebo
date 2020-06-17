#ifndef DATA_INTEGRATE_FEATURES_COLUMN_H
#define DATA_INTEGRATE_FEATURES_COLUMN_H

#include "./feature.h"

/**
 * Represents a column obstacle on the map.
 */
class Column : public Feature
{
public:
  /**
   * @param distance Distance from the robot to the feature.
   * @param angle    Angle between the robot's direction (y axis) and the position vector of the feature.
   *                 This is automatically normalized to a value between -π and π.
   */
  Column(double distance, double angle) : Feature(distance, angle)
  {
  }

  /**
   * Creates a column using x, y coordinates on the robot frame.
   *
   * @param rel_x x (meters)
   * @param rel_y y (meters)
   */
  static Column fromRelXY(double rel_x, double rel_y)
  {
    Column column(0, 0);
    Feature::setRelXY(&column, rel_x, rel_y);
    return column;
  }

  /**
   * @returns Collision radius of the column (meters)
   */
  double getCollisionRadius() const override
  {
    return 0.07;
  }
};

#endif  // DATA_INTEGRATE_FEATURES_COLUMN_H
