#ifndef POINT_CLOUD_REGISTRATION_UTILITIES_HPP
#define POINT_CLOUD_REGISTRATION_UTILITIES_HPP
#include <assert.h>

#include <pcl/common/distances.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace point_cloud_registration {

using pcl::euclideanDistance;

inline double calculateMSE(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
    assert(cloud1->size() == cloud2->size());
    double mse = 0;
    for (int i = 0; i < cloud1->size(); i++) {
        mse += euclideanDistance(cloud1->at(i), cloud2->at(i));
    }
    mse /= cloud1->size();
    return mse;
}

inline double averageClosestDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
    double avgDistance = 0;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud2);

    for (std::size_t i = 0; i < cloud1->size(); i++) {
        std::vector<int> neighbours;
        std::vector<float> distances;
        neighbours.reserve(1);
        distances.reserve(1);
        kdtree.nearestKSearch(*cloud1, i, 1, neighbours, distances);
        avgDistance += distances[0];
    }
    avgDistance /= cloud1->size();
    return avgDistance;
}

inline double medianClosestDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
    double median_distance = 0;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud2);
    std::vector<float> distances;
    for (std::size_t i = 0; i < cloud1->size(); i++) {
        std::vector<int> neighbours;
        std::vector<float> dist;
        neighbours.reserve(1);
        distances.reserve(1);
        kdtree.nearestKSearch(*cloud1, i, 1, neighbours, dist);
        distances.push_back(dist[0]);
    }
    std::sort(distances.begin(), distances.end());
    if (distances.size() % 2 != 0) {
        median_distance = distances[(distances.size() + 1) / 2];
    } else {
        median_distance = (distances[distances.size() / 2] + distances[(distances.size() / 2) + 1]) / 2.0;
    }
    return median_distance;
}
} // namespace point_cloud_registration

#endif
