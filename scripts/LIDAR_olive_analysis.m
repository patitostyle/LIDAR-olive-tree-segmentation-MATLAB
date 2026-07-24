%% LIDAR POINT CLOUD PROCESSING FOR OLIVE TREE SEGMENTATION
% This script reads a LAS/LAZ point cloud, separates ground from vegetation,
% segments individual olive tree crowns, and computes geometric parameters
% (volume, projected area, centroid) for each tree.
%
% Author: Patricio Hernandez
% Date: 2026-07-24

clear; clc; close all;

%% 1. USER INPUT - Define file path
% Change this path to point to your LAS/LAZ file
filepath = "C:\lidar_leñosos\Nube puntos LiDAR\pequeño2.las"; % <-- MODIFY THIS

% Check if file exists
if ~isfile(filepath)
    error('File not found. Please check the path: %s', filepath);
end

%% 2. READ LAS FILE
fprintf('Reading LAS file...\n');
lasReader = lasFileReader(filepath);
[ptCloud, pointAttributes] = readPointCloud(lasReader, "Attributes", "ScanAngle");
fprintf('Point cloud loaded: %d points\n', ptCloud.Count);

%% 3. VISUALIZE ORIGINAL POINT CLOUD
figure('Name', 'Original Point Cloud');
pcshow(ptCloud.Location);
title('Original LIDAR Point Cloud');
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on;

%% 4. GROUND SEGMENTATION (SMRF algorithm)
fprintf('Segmenting ground points using SMRF...\n');
[groundPtsIdx, nonGroundPtCloud, groundPtCloud] = segmentGroundSMRF(ptCloud);
fprintf('Ground points: %d, Non-ground points: %d\n', ...
    groundPtCloud.Count, nonGroundPtCloud.Count);

%% 5. VISUALIZE GROUND vs NON-GROUND
figure('Name', 'Ground vs Non-Ground');
pcshowpair(groundPtCloud, nonGroundPtCloud);
title('Ground (purple) vs Non-Ground (green) Points');
xlabel('X'); ylabel('Y'); zlabel('Z');
legend('Ground', 'Non-Ground');
grid on;

%% 6. CROWN SEGMENTATION (Euclidean clustering)
% NOTE: If crowns are touching, they may be merged into one cluster.
% Adjust 'minDistance' to control sensitivity (smaller = more clusters).
minDistance = 0.3; % meters
fprintf('Segmenting non-ground points into clusters (minDist = %.2f m)...\n', minDistance);
[labels, numClusters] = pcsegdist(nonGroundPtCloud, minDistance);
fprintf('Number of clusters detected: %d\n', numClusters);

%% 7. VISUALIZE CLUSTERS
figure('Name', 'Clustered Point Cloud');
pcshow(nonGroundPtCloud.Location, labels);
colormap(hsv(numClusters));
title(sprintf('Point Cloud Clusters (minDist = %.2f m)', minDistance));
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on;

%% 8. EXTRACT PARAMETERS FOR EACH CLUSTER
% Initialize output arrays
numPointsPerCluster = zeros(numClusters, 1);
volumePerCluster    = zeros(numClusters, 1);
areaPerCluster      = zeros(numClusters, 1);
centroidPerCluster  = zeros(numClusters, 3);
labelPerCluster     = (1:numClusters)';

% Minimum points to consider a valid cluster (noise filter)
minPointsThreshold = 15;

fprintf('Extracting parameters for %d clusters...\n', numClusters);
for t = 1:numClusters
    % Extract points belonging to cluster 't'
    clusterMask = (labels == t);
    clusterPoints = nonGroundPtCloud.Location(clusterMask, :);
    
    % If cluster has too few points, treat as noise (volume = 0)
    if size(clusterPoints, 1) < minPointsThreshold
        numPointsPerCluster(t) = size(clusterPoints, 1);
        volumePerCluster(t)    = 0;
        areaPerCluster(t)      = 0;
        centroidPerCluster(t, :) = mean(clusterPoints, 1);
        continue;
    end
    
    % Compute 3D convex hull (crown volume)
    [~, volume] = convhull(clusterPoints(:,1), clusterPoints(:,2), clusterPoints(:,3));
    volumePerCluster(t) = volume;
    
    % Compute 2D convex hull (projected area on XY plane)
    [~, area] = convhull(clusterPoints(:,1), clusterPoints(:,2));
    areaPerCluster(t) = area;
    
    % Compute centroid (mean X,Y,Z)
    centroidPerCluster(t, :) = mean(clusterPoints, 1);
    
    % Store number of points
    numPointsPerCluster(t) = size(clusterPoints, 1);
end
fprintf('Parameter extraction complete.\n');

%% 9. VISUALIZATION - CLUSTER IDENTIFICATION MAP
figure('Name', 'Tree Identification Map');
plot(centroidPerCluster(:,1), centroidPerCluster(:,2), 'o', 'MarkerSize', 6);
grid on;
hold on;
for j = 1:numClusters
    text(centroidPerCluster(j,1), centroidPerCluster(j,2), num2str(j), ...
        'FontSize', 10, 'Color', 'red');
end
title('Olive Tree Identification (Cluster ID)', 'FontWeight', 'bold');
xlabel('X coordinate (m)'); ylabel('Y coordinate (m)');
hold off;

%% 10. VISUALIZATION - VOLUME MAP
figure('Name', 'Volume Map');
plot(centroidPerCluster(:,1), centroidPerCluster(:,2), 'o', 'MarkerSize', 6);
grid on;
hold on;
for j = 1:numClusters
    text(centroidPerCluster(j,1), centroidPerCluster(j,2), ...
        sprintf('%.1f', volumePerCluster(j)), 'FontSize', 10, 'Color', 'blue');
end
title('Crown Volume per Tree (m^3)', 'FontWeight', 'bold');
xlabel('X coordinate (m)'); ylabel('Y coordinate (m)');
hold off;

%% 11. VISUALIZATION - BUBBLE CHART (Volume as bubble size)
figure('Name', 'Bubble Chart of Volumes');
bubblechart(centroidPerCluster(:,1), centroidPerCluster(:,2), volumePerCluster);
grid on;
hold on;
for j = 1:numClusters
    text(centroidPerCluster(j,1), centroidPerCluster(j,2), ...
        sprintf('%.1f', volumePerCluster(j)), 'FontSize', 10, 'HorizontalAlignment', 'center');
end
title('Crown Volume Bubble Map (m^3)', 'FontWeight', 'bold');
xlabel('X coordinate (m)'); ylabel('Y coordinate (m)');
hold off;

%% 12. SAVE RESULTS TO FILE (optional)
% Uncomment to save parameters to a CSV file
% T = table(labelPerCluster, numPointsPerCluster, volumePerCluster, areaPerCluster, ...
%           centroidPerCluster(:,1), centroidPerCluster(:,2), centroidPerCluster(:,3), ...
%           'VariableNames', {'ID', 'NumPoints', 'Volume_m3', 'Area_m2', 'Centroid_X', 'Centroid_Y', 'Centroid_Z'});
% writetable(T, 'olive_parameters.csv');
% fprintf('Results saved to olive_parameters.csv\n');

fprintf('Script completed successfully.\n');
