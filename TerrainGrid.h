#ifndef TERRAIN_GRID_H
#define TERRAIN_GRID_H

#include "Feature.h"

class TerrainGrid {
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
    using Point = pcl::PointXYZI;

    // Constructor: Receive terrain node point cloud and grid resolution
    TerrainGrid(PointCloud::Ptr terrainNode, PointCloud::Ptr inputcloud, float gridResolution = 1.0f);

    // Generate a terrain grid and save it
    void generateTerrainGrids();

    // Gets the generated terrain grid
    PointCloud::Ptr getTerrainGrids() const;
    float gridResolution;
    std::map<std::pair<int, int>, float> filledGridHeights;// Terrain grid node container
    PointCloud::Ptr inputcloud;
private:
    PointCloud::Ptr terrainNode;     // Input terrain node point cloud
    PointCloud::Ptr terrainGrids;    // Generated topographic grid point cloud

                // Grid resolution

    // Calculate the height of each grid (lower quartile)
    void calculateGridHeights(std::map<std::pair<int, int>, std::vector<float>>& gridHeights,
        std::pair<int, int>& minGrid, std::pair<int, int>& maxGrid) const;

    // Fill empty grid
    void fillEmptyGrids(const std::map<std::pair<int, int>, float>& gridHeightMap,
        std::map<std::pair<int, int>, float>& filledGridHeights,
        const std::pair<int, int>& minGrid,
        const std::pair<int, int>& maxGrid);

    // Inverse distance weight interpolation method
    float inverseDistanceWeighting(const std::vector<std::pair<std::pair<int, int>, float>>& neighbors,
        const std::pair<int, int>& targetGrid) const;
};

#endif // TERRAIN_GRID_H
