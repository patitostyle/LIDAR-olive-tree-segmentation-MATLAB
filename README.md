# LIDAR Point Cloud Processing for Olive Tree Segmentation

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023a+-orange.svg)](https://www.mathworks.com/)

## Project Context

This repository contains a MATLAB script for processing LIDAR point clouds to segment individual olive trees and extract geometric parameters (crown volume, projected area, centroid). The workflow is designed for precision agriculture applications, enabling tree-by-tree analysis from aerial or terrestrial LIDAR data.

## Methodology

The processing pipeline consists of the following steps:

1. **Data Loading**: Reads a LAS/LAZ file containing 3D point cloud data.
2. **Ground Segmentation**: Uses the Simple Morphological Filter (SMRF) algorithm to separate ground points from vegetation/objects.
3. **Crown Segmentation**: Applies Euclidean distance clustering (`pcsegdist`) to isolate individual tree crowns based on a minimum distance threshold.
4. **Parameter Extraction**: For each cluster (tree), computes:
   - **Volume**: 3D convex hull volume (crown envelope).
   - **Projected Area**: 2D convex hull area on the XY plane.
   - **Centroid**: Mean X, Y, Z coordinates of the cluster.
5. **Visualization**: Generates maps showing tree identification, volume distribution, and a bubble chart for spatial volume analysis.

**Known Limitation**: When tree crowns are touching or overlapping, the clustering algorithm may merge them into a single cluster, overestimating volume. This can be partially mitigated by adjusting the `minDistance` parameter.

## Requirements

- MATLAB R2023a or later
- Toolboxes:
  - Lidar Toolbox
  - Computer Vision Toolbox (for `pcsegdist`)
  - Statistics and Machine Learning Toolbox (for `convhull`)
- Input data: LAS or LAZ file (point cloud)

## How to Run

1. Clone this repository:
   ```bash
   git clone https://github.com/patitostyle/LIDAR-olive-tree-segmentation-MATLAB.git
