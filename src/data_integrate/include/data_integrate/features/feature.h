#ifndef DATA_INTEGRATE_FEATURES_FEATURE_H
#define DATA_INTEGRATE_FEATURES_FEATURE_H

/**
 * Represents a feature on the map, positioned on the robot frame (relative to the robot).
 */
class Feature
{
public:
  /**
   * @param distance Distance from the robot to the feature.
   * @param angle    Angle between the robot's direction (y axis) and the position vector of the feature.
   *                 This is automatically normalized to a value between -π and π.
   */
  Feature(double distance, double angle);

  /**
   * @returns x coordinate on the robot frame (meters)
   */
  double getRelX() const
  {
    return rel_x_;
  }

  /**
   * @returns y coordinate on the robot frame (meters)
   */
  double getRelY() const
  {
    return rel_y_;
  }

  /**
   * @returns Distance from the robot to this feature (meters)
   */
  double getDistance() const
  {
    return distance_;
  }

  /**
   * Angle between the robot's direction (y axis) and the position vector of the feature.
   * This is automatically normalized to a value between -π and π.
   * Positive value indicates counterclockwise(left), negative value indicates clockwise(right).
   */
  double getAngle() const
  {
    return angle_;
  }

  /**
   * Computes the distance between this feature and another one.
   */
  double getDistanceTo(const Feature& other) const;

  /**
   * Retrieves the collision radius of this feature.
   * Note: This value must be 0 or greater!
   */
  virtual double getCollisionRadius() const = 0;

protected:
  /**
   * For internal use by derived classes.
   * Updates the coordinates of a feature using x, y coordinates on the robot frame.
   *
   * @param feature Feature to update
   * @param rel_x x (meters)
   * @param rel_y y (meters)
   */
  static void setRelXY(Feature* feature, double rel_x, double rel_y);

private:
  /**
   * For internal use by other constructors.
   * This constructor saves all values as-is.
   */
  Feature(double distance, double angle, double rel_x, double rel_y)
    : distance_(distance), angle_(angle), rel_x_(rel_x), rel_y_(rel_y){};

  double distance_;
  double angle_;
  double rel_x_;
  double rel_y_;
};

#endif  // DATA_INTEGRATE_FEATURES_FEATURE_H
