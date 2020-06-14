#ifndef DATA_INTEGRATE_LINE_CLUSTER_H
#define DATA_INTEGRATE_LINE_CLUSTER_H

#include <algorithm>
#include <cmath>
#include <limits>

/** Represents a line in 2D polar coordinates. */
struct PolarLine
{
public:
  PolarLine(double rho_, double theta_) : rho(rho_), theta(theta_)
  {
  }

  double rho;    ///< Distance from the origin to the closest point on the line
  double theta;  ///< Angle of the line from the origin to the closest point on the line
};

/**
 * Computes the average theta value of all lines in the given cluster.
 */
template <typename ClusterT>
double getAverageTheta(const ClusterT& cluster)
{
  // Computing the average angle is weird!
  // See: https://rosettacode.org/wiki/Averages/Mean_angle
  double cos_sum = 0;
  double sin_sum = 0;
  for (const PolarLine& line : cluster)
  {
    cos_sum += std::cos(line.theta);
    sin_sum += std::sin(line.theta);
  }
  return std::atan(cos_sum / sin_sum);
}

/**
 * Computes the average rho value of all lines in the given cluster.
 */
template <typename ClusterT>
double getAverageRho(const ClusterT& cluster)
{
  double rho_sum = 0;
  int size = 0;
  for (const PolarLine& line : cluster)
  {
    rho_sum += line.rho;
    ++size;
  }
  return rho_sum / size;
}

// Anonymous namespace used to hold intermediate template functions
// These template functions should not be called directly
namespace
{
/**
 * Searches for a theta cluster that can accept the given line.
 * If no cluster can accept the given line, returns nullptr.
 */
template <typename ClusterGroupT, typename ClusterT = typename ClusterGroupT::value_type>
const ClusterT* findMatchingThetaCluster(const ClusterGroupT& clusters, const PolarLine& line, double threshold = 1)
{
  const ClusterT* closest_cluster = nullptr;
  auto closest_difference = std::numeric_limits<double>::infinity();

  for (const ClusterT& cluster : clusters)
  {
    auto average_theta = getAverageTheta(cluster);
    auto difference = std::abs(average_theta - line.theta);
    difference = std::min(difference, M_PI - difference);
    if (difference < threshold && difference < closest_difference)
    {
      closest_cluster = &cluster;
      closest_difference = difference;
    }
  }

  return closest_cluster;
}

/**
 * Searches for a rho cluster that can accept the given line.
 * If no cluster can accept the given line, returns nullptr.
 */
template <typename ClusterGroupT, typename ClusterT = typename ClusterGroupT::value_type>
const ClusterT* findMatchingRhoCluster(const ClusterGroupT& clusters, const PolarLine& line, double threshold = 1)
{
  const ClusterT* closest_cluster = nullptr;
  auto closest_difference = std::numeric_limits<double>::infinity();

  for (const ClusterT& cluster : clusters)
  {
    auto average_rho = getAverageRho(cluster);
    auto difference = std::abs(average_rho - line.rho);
    if (difference < threshold && difference < closest_difference)
    {
      closest_cluster = &cluster;
      closest_difference = difference;
    }
  }

  return closest_cluster;
}

template <typename ClusterGroupT, typename LineContainerT>
ClusterGroupT buildThetaClusters(const LineContainerT& lines, double threshold)
{
  using ClusterT = typename ClusterGroupT::value_type;
  ClusterGroupT theta_clusters;

  for (const auto& line : lines)
  {
    auto* cluster = const_cast<ClusterT*>(findMatchingThetaCluster(theta_clusters, line, threshold));
    if (cluster == nullptr)
    {
      // No matching cluster found, so create a new cluster
      theta_clusters.emplace_back();
      cluster = &theta_clusters.back();
    }
    cluster->push_back(line);
  }

  return theta_clusters;
}

template <typename ClusterGroupT, typename LineContainerT>
ClusterGroupT buildRhoClusters(const LineContainerT& lines, double threshold)
{
  using ClusterT = typename ClusterGroupT::value_type;
  ClusterGroupT rho_clusters;

  for (const auto& line : lines)
  {
    auto* cluster = const_cast<ClusterT*>(findMatchingRhoCluster(rho_clusters, line, threshold));
    if (cluster == nullptr)
    {
      // No matching cluster found, so create a new cluster
      rho_clusters.emplace_back();
      cluster = &rho_clusters.back();
    }
    cluster->push_back(line);
  }

  return rho_clusters;
}

}  // Anonymous namespace

template <typename ClusterGroupT, typename LineContainerT>
ClusterGroupT buildLineClusters(const LineContainerT& lines, double rho_threshold, double theta_threshold)
{
  ClusterGroupT all_clusters;
  ClusterGroupT theta_clusters = buildThetaClusters<ClusterGroupT>(lines, theta_threshold);
  for (auto& theta_cluster : theta_clusters)
  {
    ClusterGroupT rho_clusters = buildRhoClusters<ClusterGroupT>(theta_cluster, rho_threshold);
    all_clusters.insert(all_clusters.end(), rho_clusters.begin(), rho_clusters.end());
  }
  return all_clusters;
}

#endif
