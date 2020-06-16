#include "data_integrate/features/visible_feature_manager.h"

#include <ros/ros.h>
#include <cmath>
#include <limits>
#include <vector>

using RelPointList = std::vector<RelPoint>;
using RelPointClusterList = std::vector<RelPointList>;

/**
 * 도우미 구조체: 직사각형을 나타냅니다.
 */
struct Rect
{
  double x;
  double y;
  double width;
  double height;
};

/**
 * 도우미 함수: @p cluster 에 속한 점들 중에서 @p point 와 가장 가까운 점과 @p point 사이의 거리를 구합니다.
 * @p cluster 가 비어있으면 무한대를 리턴합니다.
 */
double computeDistanceToNearestPointInCluster(const RelPoint& point, const RelPointList& cluster)
{
  auto nearest_distance_sq = std::numeric_limits<double>::infinity();

  for (auto& cluster_point : cluster)
  {
    auto distance_sq = std::norm(
        std::complex<double>(cluster_point.getRelX() - point.getRelX(), cluster_point.getRelY() - point.getRelY()));
    if (distance_sq < nearest_distance_sq)
    {
      nearest_distance_sq = distance_sq;
    }
  }

  // 못 찾았을 경우
  if (std::isinf(nearest_distance_sq))
  {
    return nearest_distance_sq;
  }

  return std::sqrt(nearest_distance_sq);
}

/**
 * 도우미 함수: @p points 에 속한 점들의 좌표의 평균점을 구합니다.
 */
RelPoint computeAveragePoint(const RelPointList& points)
{
  double x_sum = 0, y_sum = 0;
  for (auto& point : points)
  {
    x_sum += point.getRelX();
    y_sum += point.getRelY();
  }
  return RelPoint::fromRelXY(x_sum / points.size(), y_sum / points.size());
}

/**
 * 도우미 함수: 여러 개의 점들을 클러스터로 묶습니다. 이때 각 클러스터의 점들 중 가장 가까운 점까지의 거리를 사용합니다.
 * 주의: 비효율적입니다. O(n^2)
 *
 * @param points      점들의 목록. 무한히 먼 거리에 있는 점들은 제외합니다.
 * @param threshold   같은 클러스터에 속한 점으로 취급하기 위한 점 간의 최대 거리
 * @returns 검출한 기둥들의 좌표
 */
RelPointClusterList clusterByNearestPointDistance(const RelPointList& points, double threshold)
{
  RelPointClusterList clusters;

  for (auto& point : points)
  {
    RelPointList* nearest_cluster = nullptr;
    auto nearest_distance = std::numeric_limits<double>::infinity();

    // 가장 가까운 클러스터를 찾는다
    for (auto& cluster : clusters)
    {
      auto distance = computeDistanceToNearestPointInCluster(point, cluster);
      if (distance <= threshold)
      {
        nearest_cluster = &cluster;
        nearest_distance = distance;
      }
    }

    if (nearest_cluster)
    {
      // 가장 가까운 클러스터에 점 추가
      nearest_cluster->emplace_back(point);
    }
    else
    {
      // 새로운 클러스터를 만들어 넣는다.
      clusters.emplace_back(1, point);
    }
  }

  return clusters;
}

/**
 * 도우미 함수: 클러스터에 속한 모든 점을 포함하는 가장 작은 직사각형을 구합니다.
 * @p cluster 가 비어있으면 { 0, 0, 0, 0 }을 돌려줍니다.
 *
 * @param points  클러스터에 속한 점들의 목록
 * @returns 클러스터를 감싸는 직사각형의 위치 및 크기 정보
 */
Rect findBoundingBox(const RelPointList& points)
{
  if (points.size() == 0)
    return {};

  double x_min = std::numeric_limits<double>::infinity(), x_max = -std::numeric_limits<double>::infinity();
  double y_min = std::numeric_limits<double>::infinity(), y_max = -std::numeric_limits<double>::infinity();
  for (auto& point : points)
  {
    auto x = point.getRelX(), y = point.getRelY();
    if (x < x_min)
      x_min = x;
    if (x > x_max)
      x_max = x;
    if (y < y_min)
      y_min = y;
    if (y > y_max)
      y_max = y;
  }

  ROS_ASSERT(x_min <= x_max);
  ROS_ASSERT(y_min <= y_max);
  return { x_min, y_min, x_max - x_min, y_max - y_min };
}

/**
 * 도우미 함수: LIDAR 데이터로부터 기둥을 검출합니다.
 *
 * @param points      LIDAR로 검출한 점들의 목록. 무한히 먼 거리에 있는 점들은 제외합니다.
 * @param threshold   같은 기둥 내 점으로 취급하기 위한 점 간의 최대 거리 (meters)
 * @param min_points  한 기둥 내에 포함된 점의 최소 갯수. 이보다 점이 적으면 노이즈로 취급해 무시합니다.
 * @param max_points  한 기둥 내에 포함된 점의 최대 갯수. 이보다 점이 많으면 기둥이 아닌 벽으로 판단해 무시합니다.
 * @param max_cluster_size  클러스터의 최대 크기 (meters).
 * 클러스터의 bounding box의 너비나 높이가 이보다 크면 기둥이 아닌 벽으로 판단해 무시합니다.
 * @returns 검출한 기둥들의 좌표
 */
RelPointList findColumns(const RelPointList& points, double threshold, size_t min_points, size_t max_points,
                         double max_cluster_size)
{
  ROS_ASSERT(threshold >= 0);
  ROS_ASSERT(min_points <= max_points);
  ROS_ASSERT(max_cluster_size >= 0);

  auto clusters = clusterByNearestPointDistance(points, threshold);

  RelPointList columns;
  for (auto& cluster : clusters)
  {
    auto bounding_box = findBoundingBox(cluster);

    if (min_points <= cluster.size() && cluster.size() <= max_points && bounding_box.width <= max_cluster_size &&
        bounding_box.height <= max_cluster_size)
    {
      columns.emplace_back(computeAveragePoint(cluster));
    }
  }

  return columns;
}

void VisibleFeatureManager::subscribeToLidar(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // 기존의 점을 모두 지웁니다
  lidar_points_.clear();

  std::vector<int>::size_type i = 0;
  for (auto& range : msg->ranges)
  {
    // 허용 범위 내의 값만 사용합니다.
    // 참고: https://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html
    if (msg->range_min <= range && range <= msg->range_max)
    {
      // LIDAR는 x축을 기준으로 반시계 방향으로 각도를 측정합니다.
      // 저희는 y축을 기준으로 측정할 예정이니 90도를 빼야 합니다.
      // 그런데 180도 뒤집힌 것을 보니 센서가 거꾸로 달려 있는 듯? 그래서 90도를 더하는 것으로 했습니다.
      auto angle = msg->angle_min + msg->angle_increment * i + M_PI / 2;
      lidar_points_.emplace_back(range, angle);
    }

    ++i;
  }

  // 기둥을 검출합니다.
  const double COLUMN_THRESHOLD = 0.3;  // meters
  const size_t COLUMN_MIN_POINTS = 2;
  const size_t COLUMN_MAX_POINTS = 20;  // 너무 작게 하면 가까운 기둥을 인식하지 못하므로, 적당히 큰 값으로 합니다.
  const double COLUMN_MAX_CLUSTER_SIZE = 0.2;  // meters

  columns_ =
      findColumns(lidar_points_, COLUMN_THRESHOLD, COLUMN_MIN_POINTS, COLUMN_MAX_POINTS, COLUMN_MAX_CLUSTER_SIZE);

  const size_t EXPECTED_COLUMN_COUNT = 4;
  ROS_WARN_COND(columns_.size() > EXPECTED_COLUMN_COUNT, "%lu columns detected (expected %lu)", columns_.size(),
                EXPECTED_COLUMN_COUNT);
}

void VisibleFeatureManager::subscribeToCamera(const core_msgs::ball_position::ConstPtr& msg)
{
  // 기존의 공을 모두 지웁니다
  balls_.clear();

  // 공의 (x, y, z) 좌표에서 x축은 로봇과 동일하지만 z축은 로봇이 바라보는 방향입니다.
  // 로봇에 대한 평면 좌표계를 사용하려면 y축 대신 z축을 사용해야 합니다.
  for (auto& ball : msg->blue_balls)
  {
    addBall(Ball::fromRelXY(BallColor::Blue, ball.x, ball.z));
  }

  for (auto& ball : msg->red_balls)
  {
    addBall(Ball::fromRelXY(BallColor::Red, ball.x, ball.z));
  }

  for (auto& ball : msg->green_balls)
  {
    addBall(Ball::fromRelXY(BallColor::Green, ball.x, ball.z));
  }

  is_blue_ball_captured_ = msg->still_blue;
}

void VisibleFeatureManager::addBall(const Ball& ball)
{
  balls_.emplace_back(ball);
}

void VisibleFeatureManager::clearAllFeatures()
{
  balls_.clear();
  lidar_points_.clear();
}

void VisibleFeatureManager::subscribeToImu(const sensor_msgs::Imu::ConstPtr& msg)
{
  // 주의: 창시구 규정에 의해 orientation은 사용 금지됨
  imu_angular_velocity_ = msg->angular_velocity;
  imu_linear_acceleration_ = msg->linear_acceleration;
}

void VisibleFeatureManager::subscribeToLineInfo(const core_msgs::line_info::ConstPtr& msg)
{
  line_tracer_section_ = msg->section;
  line_tracer_box_x_ = msg->x;
  line_tracer_box_y_ = msg->y;
  line_tracer_box_width_ = msg->w;
  line_tracer_box_height_ = msg->h;
  line_tracer_image_width_ = msg->image_width;
  line_tracer_image_height_ = msg->image_height;
}
