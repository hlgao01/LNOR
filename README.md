# LiDAR Negative Outlier Removal Algorithm

## Introduction

This project implements a LiDAR-based negative outlier removal algorithm, specifically designed for coastal environments where multipath reflections cause negative outliers. The algorithm uses mirror structure and intensity features for outlier detection and removal.

## Features

- Point cloud data processing using PCL
- Negative outlier removal based on mirror feature and intensity comparison
- Terrain node and grid generation for spatial feature analysis
  
## Workflow
- Load Point Cloud
- Load the .pcd file into memory using PCL.

- Calculate Features
- Use a K-nearest neighbors (KNN) approach to compute structural features for each point.

- Generate Terrain Nodes
- Extract representative points (terrain nodes) from the original cloud based on calculated features.

- Build Terrain Grid
- Create a terrain mesh/grid based on the nodes for local terrain representation.

- Outlier Removal
- Apply the LNOR algorithm to:

- Find potential mirror points for each candidate

- Evaluate the mirror structure and intensity similarity

- Remove points identified as negative outliers

## Example Code(main)

#include "Feature.h"
#include "TerrainNode.h"
#include "TerrainGrid.h"
#include "LNOR.h"
#include <pcl/io/pcd_io.h>

pcl::io::loadPCDFile("Inputcloud.pcd", *cloud);

// Step 1: Feature calculation
Feature featureCalculator(cloud, 20); // set K = 20
featureCalculator.calculateFeatures();

// Step 2: Terrain node generation
TerrainNode terrainnode(cloud, featureCalculator.pointFeature, 0.5f); // set incident angle (3.14f=phi)
terrainnode.generateTerrainNodes();
auto node = terrainnode.getTerrainNodes();

// Step 3: Terrain grid generation
TerrainGrid terraingrid(node, cloud, 0.5f); // set Grid size = 0.5m
terraingrid.generateTerrainGrids();
auto grid = terraingrid.getTerrainGrids();

// Step 4: LNOR execution
LNOR NegativeFilter(cloud, terraingrid, featureCalculator.pointFeature); 
NegativeFilter.execute();
auto filteredPoints = NegativeFilter.getFilteredPoints();
auto Outliers = NegativeFilter.getRemovedOutliers();

## Installation

### Prerequisites

- C++ compiler (C++11 or later)
- [PCL (Point Cloud Library)](https://pointclouds.org/) for point cloud processing

### Build Instructions

1. Clone the repository:

   ```bash
   git clone https://github.com/hlgao01/LNOR.git
   cd LNOR
