# LiDAR Negative Outlier Removal Algorithm

## Introduction

This project implements a LiDAR-based negative outlier removal algorithm, specifically designed for coastal environments where multipath reflections cause negative outliers. The algorithm uses mirror structure and intensity features for outlier detection and removal.

## Features

- Point cloud data processing using PCL
- Negative outlier removal based on mirror feature and intensity comparison
- Terrain node and grid generation for spatial feature analysis

## Installation

### Prerequisites

- C++ compiler (C++11 or later)
- [PCL (Point Cloud Library)](https://pointclouds.org/) for point cloud processing

### Build Instructions

1. Clone the repository:

   ```bash
   git clone https://github.com/hlgao01/LNOR.git
   cd LNOR
