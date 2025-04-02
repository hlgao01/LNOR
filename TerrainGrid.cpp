#include "TerrainGrid.h"


// constructor
TerrainGrid::TerrainGrid(PointCloud::Ptr terrainNode, PointCloud::Ptr inputcloud, float gridResolution)
    : terrainNode(terrainNode), inputcloud(inputcloud), gridResolution(gridResolution) {
    terrainGrids.reset(new PointCloud);
}

// Gets the generated terrain grid
TerrainGrid::PointCloud::Ptr TerrainGrid::getTerrainGrids() const {
    return terrainGrids;
}

// Generate a terrain grid and save it
void TerrainGrid::generateTerrainGrids() {
    if (!terrainNode || terrainNode->empty()) {
        std::cerr << "No terrain nodes available for grid generation." << std::endl;
        return;
    }

    // Step 1: Calculate the height of each grid and get the grid range
    std::map<std::pair<int, int>, std::vector<float>> gridHeights;
    std::pair<int, int> minGrid, maxGrid;
    calculateGridHeights(gridHeights, minGrid, maxGrid);

    // Step 2: Calculates the lower quartile height of the center of the grid
    
    std::map<std::pair<int, int>, float> gridHeightMap;
    for (const auto& grid : gridHeights) {
        const auto& heights = grid.second;
        if (!heights.empty()) {
            std::vector<float> sortedHeights = heights;
            std::sort(sortedHeights.begin(), sortedHeights.end());
            float q1Height = sortedHeights[sortedHeights.size() / 4];
            gridHeightMap[grid.first] = q1Height;
        }
    }

    // Step 3: Fill empty grid
  
    fillEmptyGrids(gridHeightMap, filledGridHeights, minGrid, maxGrid);

    // Step 4: Save the grid center point
    for (const auto& grid : filledGridHeights) {
        const auto& gridIndex = grid.first;
        float height = grid.second;

        Point gridPoint;
        gridPoint.x = gridIndex.first * gridResolution;
        gridPoint.y = gridIndex.second * gridResolution;
        gridPoint.z = height;

        terrainGrids->points.push_back(gridPoint);
    }

    terrainGrids->width = terrainGrids->points.size();
    terrainGrids->height = 1;
    terrainGrids->is_dense = true;
}

// Calculate the height of each grid
void TerrainGrid::calculateGridHeights(std::map<std::pair<int, int>, std::vector<float>>& gridHeights,
    std::pair<int, int>& minGrid,
    std::pair<int, int>& maxGrid) const {
    minGrid = { INT_MAX, INT_MAX };
    maxGrid = { INT_MIN, INT_MIN };
    for (const auto& point : inputcloud->points) {
        int gridX = static_cast<int>(std::floor(point.x / gridResolution));
        int gridY = static_cast<int>(std::floor(point.y / gridResolution));
        minGrid.first = std::min(minGrid.first, gridX);
        minGrid.second = std::min(minGrid.second, gridY);
        maxGrid.first = std::max(maxGrid.first, gridX);
        maxGrid.second = std::max(maxGrid.second, gridY);
    }
    for (const auto& point : terrainNode->points) {
        int gridX = static_cast<int>(std::floor(point.x / gridResolution));
        int gridY = static_cast<int>(std::floor(point.y / gridResolution));

        gridHeights[{gridX, gridY}].push_back(point.z);
    }
}

// Fill empty grid
void TerrainGrid::fillEmptyGrids(const std::map<std::pair<int, int>, float>& gridHeightMap,
    std::map<std::pair<int, int>, float>& filledGridHeights,
    const std::pair<int, int>& minGrid,
    const std::pair<int, int>& maxGrid) {
    std::vector<std::pair<int, int>> gridlessContainers;

    for (int x = minGrid.first; x <= maxGrid.first; ++x) {
        for (int y = minGrid.second; y <= maxGrid.second; ++y) {
            std::pair<int, int> gridIndex = { x, y };

            auto it = gridHeightMap.find(gridIndex);
            if (it != gridHeightMap.end()) {
                filledGridHeights[gridIndex] = it->second;
            }
            else {
                gridlessContainers.push_back(gridIndex);
            }
        }
    }

    int searchDistance = 1;
    while (!gridlessContainers.empty()) {
        std::vector<std::pair<int, int>> remainingGridlessContainers;

        for (const auto& gridIndex : gridlessContainers) {
            std::vector<std::pair<std::pair<int, int>, float>> neighbors;

            for (int dx = -searchDistance; dx <= searchDistance; ++dx) {
                for (int dy = -searchDistance; dy <= searchDistance; ++dy) {
                    if (dx == 0 && dy == 0) continue;

                    std::pair<int, int> neighborIndex = { gridIndex.first + dx, gridIndex.second + dy };
                    auto it = filledGridHeights.find(neighborIndex);
                    if (it != filledGridHeights.end()) {
                        neighbors.emplace_back(neighborIndex, it->second);
                    }
                }
            }

            if (!neighbors.empty()) {
                filledGridHeights[gridIndex] = inverseDistanceWeighting(neighbors, gridIndex);
            }
            else {
                remainingGridlessContainers.push_back(gridIndex);
            }
        }

        gridlessContainers = std::move(remainingGridlessContainers);
        ++searchDistance;
    }
}

// Inverse distance weight interpolation
float TerrainGrid::inverseDistanceWeighting(const std::vector<std::pair<std::pair<int, int>, float>>& neighbors,
    const std::pair<int, int>& targetGrid) const {
    float weightedSum = 0.0f;
    float weightTotal = 0.0f;

    for (const auto& neighbor : neighbors) {
        float dx = neighbor.first.first - targetGrid.first;
        float dy = neighbor.first.second - targetGrid.second;
        float distance = std::sqrt(dx * dx + dy * dy);

        float weight = 1.0f / (distance + 1e-6f);
        weightedSum += weight * neighbor.second;
        weightTotal += weight;
    }

    return weightedSum / weightTotal;
}
