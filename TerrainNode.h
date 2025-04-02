#ifndef TERRAIN_NODE_H
#define TERRAIN_NODE_H

#include "Feature.h"

class TerrainNode {
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
    using Point = pcl::PointXYZI;

    // Constructor: Receives input point cloud and feature data
    TerrainNode(PointCloud::Ptr inputCloud,
        const std::vector<std::pair<int, Eigen::Vector3f>>& pointFeature = Feature::pointFeature,
        float theta = 0.52f, float distanceThreshold = 1.0f);

    // Main process: Generate terrain nodes and save them
    void generateTerrainNodes();

    // Get the terrain node point cloud
    PointCloud::Ptr getTerrainNodes() const;
    PointCloud::Ptr getSurfacePoints() const;
private:
    PointCloud::Ptr inputCloud;
    std::vector<std::pair<int, Eigen::Vector3f>> currentpointFeature;

    float theta;                // Normal Angle threshold (radians)
    float distanceThreshold;    // Surface distance threshold
    
    PointCloud::Ptr surfacePoints; // The point used to fit a surface is the identified initial surface point
    PointCloud::Ptr terrainNodes; // Storage terrain node

    // Calculate the points that meet the conditions
    void filterSurfacePoints(PointCloud::Ptr& filteredPoints, std::vector<int>& indices) const;

    // surface fitting: Z(x, y) = ax^2 + by^2 + cxy + dx + ey + f
    Eigen::VectorXd fitSurface(const PointCloud::Ptr& points) const;

    // Filter terrain nodes by surface
    void filterTerrainNodes(const Eigen::VectorXd& surfaceParams, const PointCloud::Ptr& surfacePoints);
};

#endif // TERRAIN_GRID_H
