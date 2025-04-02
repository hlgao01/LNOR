#ifndef FEATURE_H
#define FEATURE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

class Feature {
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
    using PointCloudRGB = pcl::PointCloud<pcl::PointXYZRGB>;
    using Point = pcl::PointXYZI;
    Feature(PointCloud::Ptr cloud, int N);
    void calculateFeatures(); // Executive feature calculation
    PointCloudRGB::Ptr getOutputCloud(); // Map and get colored point clouds
    static std::vector<std::pair<int, Eigen::Vector3f>> pointFeature; // Public feature container
private:
    PointCloud::Ptr inputCloud;
    PointCloudRGB::Ptr outputCloud;
    pcl::search::KdTree<Point>::Ptr kdtree;
    int num;
    std::vector<int> sequence;
    void computeFeatureForPoint(const Point& searchPoint, std::pair<int, Eigen::Vector3f>& bestFeature) const;
};

#endif // FEATURE_H
