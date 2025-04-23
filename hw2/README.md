# Mesh Processing and Intrinsic Symmetry Detection

This project implements several mesh processing algorithms, including geodesic path computation, farthest point sampling, patch visualization, and intrinsic reflectional symmetry detection using pairwise harmonic fields.

## Overview

The application provides a console menu to interact with a 3D mesh loaded into a Coin3D viewer. You can perform various analyses and visualizations on the mesh. The core functionality revolves around computing geodesic distances, sampling points, and analyzing intrinsic properties using pairwise harmonic fields, culminating in the detection and visualization of the mesh's intrinsic symmetry axis.

## Features / Menu Options

1.  **Compute geodesic path between two vertices:** Calculates and visualizes the shortest path along the mesh surface between two user-selected vertices using Dijkstra's algorithm.
2.  **Visualize farthest point sampling:** Selects a specified number of points on the mesh such that they are maximally far apart from each other in terms of geodesic distance. Visualizes the sample points and the paths between them.
3.  **Compute and visualize patches (using mesh 249.off):** Loads `249.off`, performs FPS, finds an optimal loop order for the FPS points, and visualizes both the loop paths (MAP 2) and star-like paths from the center to FPS points (MAP 1). Also attempts to visualize an interpolated patch based on the loop.
4.  **Compute and visualize patches (using mesh Armadillo.off):** Same as option 3, but uses the `Armadillo.off` mesh.
5.  **Test pairwise harmonics (Phase 1 & 2 Test):** Allows the user to select two vertices (`p`, `q`) and computes the pairwise harmonic fields (`f_pq`, `f_qp`) and their corresponding R (perimeter) and D (distance) descriptors. It visualizes the `f_pq` field and prints text-based histograms for all four descriptors (`R_pq`, `D_pq`, `R_qp`, `D_qp`) to the console.
6.  **Compute and visualize intrinsic symmetry (Phase 3):** Implements the intrinsic symmetry detection algorithm:
    *   Performs FPS to get sample points.
    *   Precomputes geodesic distances from FPS points.
    *   Iterates through pairs of FPS points, filtering pairs that are too close geodesically.
    *   Computes pairwise harmonic fields and R/D descriptors for valid pairs.
    *   Calculates the Path Intrinsic Symmetry (PIS) score for each pair.
    *   Pairs with a PIS score above a user-defined threshold (`pis_threshold`) "vote" for faces near their midpoint iso-curve.
    *   Visualizes the faces that received the highest votes (in green), representing the detected symmetry axis/region.
7.  **Exit:** Closes the application.

## Core Algorithms

*   **Geodesic Distance:** Uses Dijkstra's algorithm on the mesh graph where edge weights are Euclidean distances between connected vertices. Precomputation is used in the symmetry detection phase for efficiency.
*   **Farthest Point Sampling (FPS):** Iteratively selects points that maximize the minimum geodesic distance to already selected points.
*   **Pairwise Harmonics:** Solves a Laplace equation on the mesh with boundary conditions set at two points (`p=0`, `q=1`) to obtain a smooth function (`f_pq`) representing intrinsic "flow" between the points. The R descriptor measures the perimeter of iso-level curves, while the D descriptor measures the average geodesic distance from `p` and `q` along iso-level curves.
*   **Intrinsic Symmetry Detection:** Leverages the property that for intrinsically symmetric points `p` and `q`, the descriptors should exhibit mirror symmetry (e.g., `R_pq[i] ≈ R_qp[K-1-i]`). The PIS score quantifies this similarity. Pairs with high PIS scores vote for the midpoint region, and the aggregation of votes reveals the symmetry axis.

## Usage Notes

*   **Mesh Files:** The application expects mesh files in the `.off` format. By default, it loads `man0.off` on startup. Options 3, 4, and potentially others load specific meshes (`249.off`, `Armadillo.off`) from the hardcoded path: `C:/Users/cagopa/Desktop/Digital-Geometry-Processing/hw2/`. Ensure these files exist at that location or modify the paths in [`main.cpp`](main.cpp ).
*   **Performance - Vertex Count:** Algorithms involving geodesic distance computation (Dijkstra) and Laplacian matrix operations (Pairwise Harmonics, Symmetry Detection) are computationally intensive. Their runtime increases significantly with the number of vertices in the mesh. Using very large meshes can lead to long computation times, especially during the initial Laplacian factorization and the geodesic distance precomputation in Option 6.
*   **Performance - PIS Threshold (Option 6):** When computing symmetry (Option 6), you are prompted for a PIS threshold (default 0.7). A *lower* threshold means more pairs will be considered potentially symmetric. This leads to *longer* computation times as fewer pairs are filtered out, requiring more descriptor calculations and voting operations. A higher threshold filters more aggressively, speeding up computation but potentially missing weaker symmetries.
*   **Interpreting Histograms (Option 5):** When testing pairwise harmonics, observe the printed histograms. For a pair of points `(p, q)` that are intrinsically symmetric, you should expect to see approximate mirror symmetry between the `R_pq` and `R_qp` histograms, and similarly between `D_pq` and `D_qp`. Peaks in `R_pq` at index `i` should correspond to peaks in `R_qp` at index `K-1-i` (where K is the number of samples).
*   **Interpreting Symmetry Visualization (Option 6):** The green highlighted faces represent the region where the algorithm detected the highest likelihood of an intrinsic symmetry axis. It's a visualization of the aggregated votes.

## Dependencies

*   **Coin3D / SoWin:** Used for 3D visualization and interaction.
*   **Eigen:** Used for linear algebra operations (sparse matrices, vectors, solvers).