#ifndef POINT_CLOUD_REGISTRATION_UTILITIES_HPP
#define POINT_CLOUD_REGISTRATION_UTILITIES_HPP
#include <assert.h>

#include <pcl/common/distances.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace point_cloud_registration {

using pcl::euclideanDistance;

double calculateMSE(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
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

} // namespace point_cloud_registration

#endif
