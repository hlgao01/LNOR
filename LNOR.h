#ifndef LNOR_H
#define LNOR_H


#include "Feature.h"
#include "TerrainGrid.h"

class LNOR {
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

    LNOR(PointCloud::Ptr inputCloud,
        const TerrainGrid& terrainGrid,
        const std::vector<std::pair<int, Eigen::Vector3f>>& pointFeature);

    void execute();

    PointCloud::Ptr getFilteredPoints() const;
    PointCloud::Ptr getRemovedOutliers() const;

private:
    PointCloud::Ptr inputCloud;
    const TerrainGrid& terrainGrid;
    std::vector<std::pair<int, Eigen::Vector3f>> pointFeature;
    float alpha = 0.5f;
    float theta = 1.0f;

    PointCloud::Ptr filteredPoints;
    PointCloud::Ptr removedOutliers;

    pcl::KdTreeFLANN<pcl::PointXYZI> kdTree; // k-d tree

    void buildKdTree(); // build k-d tree
    bool isOutlier(const pcl::PointXYZI& point, int i,float gridHeight, const std::vector<int>& normalNeighbors) const;
    float findGridHeight(const pcl::PointXYZI& point) const;
    void findNormalNeighbors(const pcl::PointXYZI& point, std::vector<int>& normalNeighbors) const;
};

#endif // LNOR_H
