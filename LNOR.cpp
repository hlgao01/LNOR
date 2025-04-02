#include "LNOR.h"

LNOR::LNOR(PointCloud::Ptr inputCloud,
    const TerrainGrid& terrainGrid,
    const std::vector<std::pair<int, Eigen::Vector3f>>& pointFeature,
    float alpha, float theta)
    : inputCloud(inputCloud), terrainGrid(terrainGrid),
    pointFeature(pointFeature), alpha(alpha), theta(theta) {
    filteredPoints.reset(new PointCloud);
    removedOutliers.reset(new PointCloud);
    buildKdTree(); // Build a k-d tree at initialization
}

void LNOR::buildKdTree() {
    if (!inputCloud || inputCloud->empty()) {
        std::cerr << "Input cloud is empty. Cannot build k-d tree." << std::endl;
        return;
    }
    PointCloud::Ptr projectedcloud(new PointCloud);
    for (auto& point : *inputCloud)
    {
        pcl::PointXYZI p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0;
        p.intensity = point.intensity;
        projectedcloud->push_back(p);
    }
    kdTree.setInputCloud(projectedcloud);
}

LNOR::PointCloud::Ptr LNOR::getFilteredPoints() const {
    return filteredPoints;
}

LNOR::PointCloud::Ptr LNOR::getRemovedOutliers() const {
    return removedOutliers;
}

void LNOR::execute() {
    if (!inputCloud || inputCloud->empty()) {
        std::cerr << "Invalid input data for LNOR algorithm." << std::endl;
        return;
    }

    for (size_t i = 0; i < inputCloud->size(); i++) {
        const auto& point = inputCloud->points[i];
        float gridHeight = findGridHeight(point);

        if (point.z < gridHeight - alpha) {
            /*removedOutliers->points.push_back(point);
            continue;*/
            std::vector<int> normalNeighbors;
            findNormalNeighbors(point, normalNeighbors);

            if (isOutlier(point,i, gridHeight, normalNeighbors)) {
                removedOutliers->points.push_back(point);
            }
            else {
                filteredPoints->points.push_back(point);
            }
        }
        else {
            filteredPoints->points.push_back(point);
        }
    }

    filteredPoints->width = filteredPoints->points.size();
    filteredPoints->height = 1;
    filteredPoints->is_dense = true;

    removedOutliers->width = removedOutliers->points.size();
    removedOutliers->height = 1;
    removedOutliers->is_dense = true;
}

float LNOR::findGridHeight(const pcl::PointXYZI& point) const {
    int gridX = static_cast<int>(std::floor(point.x / terrainGrid.gridResolution));
    int gridY = static_cast<int>(std::floor(point.y / terrainGrid.gridResolution));

    auto it = terrainGrid.filledGridHeights.find({ gridX, gridY });
    if (it != terrainGrid.filledGridHeights.end() && !isnan(it->second)) {
        return  it->second;
    }

    // The default value is 0 or some other policy height value
    std::cerr << "Warning: Point outside grid bounds. Returning default height of 0." << std::endl;
    return 0.0f;
}

void LNOR::findNormalNeighbors(const pcl::PointXYZI& point, std::vector<int>& normalNeighbors) const {
    if (!inputCloud || inputCloud->empty()) {
        std::cerr << "Input cloud is empty. Cannot find normal neighbors." << std::endl;
        return;
    }
    pcl::PointXYZI projection_p;
    projection_p.x = point.x;
    projection_p.y = point.y;
    projection_p.z = 0;
    projection_p.intensity = point.intensity;
    std::vector<float> pointRadiusSquaredDistance;
    kdTree.radiusSearch(projection_p, theta, normalNeighbors, pointRadiusSquaredDistance);
}

bool LNOR::isOutlier(const pcl::PointXYZI& point,int i, float gridHeight, const std::vector<int>& normalNeighbors) const {
    for (const auto& index : normalNeighbors) {
        const auto& neighbor = inputCloud->points[index];
        float zCondition = std::fabs(point.z + neighbor.z - 2 * gridHeight);
        if (zCondition <= 2) { //zCondition <= alpha + theta
            int pointFeatureIndex = pointFeature[i].first;
            int neighborFeatureIndex = pointFeature[index].first;

            if (pointFeatureIndex == neighborFeatureIndex ||
                point.intensity < 0.5f * neighbor.intensity) {
                return true; // This point meets the anomaly condition
            }
        }
    }

    return false;
}
