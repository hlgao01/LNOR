#include "TerrainNode.h"
#include <pcl/common/transforms.h>
#include <fstream>
#include <iostream>

// constructor
TerrainNode::TerrainNode(PointCloud::Ptr inputCloud,
    const std::vector<std::pair<int, Eigen::Vector3f>>& pointFeature,
    float theta, float distanceThreshold)
    : inputCloud(inputCloud), currentpointFeature(pointFeature),
    theta(theta), distanceThreshold(distanceThreshold) {
    terrainNodes.reset(new PointCloud);
}

// Main process
void TerrainNode::generateTerrainNodes() {
    surfacePoints.reset(new PointCloud);
    std::vector<int> indices;
    filterSurfacePoints(surfacePoints, indices);

    Eigen::VectorXd surfaceParams = fitSurface(surfacePoints);
    filterTerrainNodes(surfaceParams, surfacePoints);
}
// Get the surface fitting points
TerrainNode::PointCloud::Ptr TerrainNode::getSurfacePoints() const {
    return surfacePoints;
}
// Get terrain node
TerrainNode::PointCloud::Ptr TerrainNode::getTerrainNodes() const {
    return terrainNodes;
}

// Filter the surface points that meet the conditions
void TerrainNode::filterSurfacePoints(PointCloud::Ptr& filteredPoints, std::vector<int>& indices) const {
    for (size_t i = 0; i < currentpointFeature.size(); ++i) {
        if (currentpointFeature[i].first == 2) { // Planar structure
            Eigen::Vector3f normal = currentpointFeature[i].second;
            float cosTheta = std::abs(normal.dot(Eigen::Vector3f(0, 0, 1))); // Cosine of the Angle in the vertical direction
            if (std::acos(cosTheta) <= theta) { // The Angle threshold is met
                filteredPoints->points.push_back(inputCloud->points[i]);
                indices.push_back(i);
            }
        }
    }
    filteredPoints->width = filteredPoints->points.size();
    filteredPoints->height = 1;
    filteredPoints->is_dense = true;
}

// Surface fitting
Eigen::VectorXd TerrainNode::fitSurface(const PointCloud::Ptr& points) const {
    size_t n = points->points.size();
    Eigen::MatrixXd A(n, 6);
    Eigen::VectorXd b(n);

    for (size_t i = 0; i < n; ++i) {
        float x = points->points[i].x;
        float y = points->points[i].y;
        float z = points->points[i].z;

        A(i, 0) = x * x;
        A(i, 1) = y * y;
        A(i, 2) = x * y;
        A(i, 3) = x;
        A(i, 4) = y;
        A(i, 5) = 1.0;
        b(i) = z;
    }

    // Least squares fitting
    Eigen::VectorXd params = A.colPivHouseholderQr().solve(b);
    return params; // return [a, b, c, d, e, f]
}

// Filtered terrain node
void TerrainNode::filterTerrainNodes(const Eigen::VectorXd& surfaceParams, const PointCloud::Ptr& surfacePoints) {
    for (const auto& point : surfacePoints->points) {
        float x = point.x;
        float y = point.y;
        float z = point.z;

        float fittedZ = surfaceParams(0) * x * x + surfaceParams(1) * y * y +
            surfaceParams(2) * x * y + surfaceParams(3) * x +
            surfaceParams(4) * y + surfaceParams(5);

        float distance = std::abs(z - fittedZ);
        if (distance <= distanceThreshold) {
            terrainNodes->points.push_back(point);
        }
    }

    terrainNodes->width = terrainNodes->points.size();
    terrainNodes->height = 1;
    terrainNodes->is_dense = true;
}
