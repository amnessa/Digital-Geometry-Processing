# TODO List - Pairwise Harmonics Symmetry Detection

## High Priority / Core Functionality

-   **[x] Implement Robust `findClosestFace` for Voting (`main.cpp` / `Mesh.cpp`):**
    -   The current voting mechanism in [`computeAndVisualizeSymmetry`](c:\Users\cagopa\Desktop\Digital-Geometry-Processing\hw2\main.cpp#L690-L702) uses a placeholder that finds the face with the closest *centroid* to the isocurve segment midpoint.
    -   Replace this with a more accurate method, potentially in `Mesh.cpp` (e.g., `mesh->findClosestFace(segment_midpoint)` or even better, finding the face(s) the segment *intersects*). See comment at [`main.cpp:693`](c:\Users\cagopa\Desktop\Digital-Geometry-Processing\hw2\main.cpp#L693).
-   **[x] Complete Obtuse Voronoi Area Calculation (`helpers.cpp`):**
    -   The `if/else` block for handling obtuse triangles within [`PairwiseHarmonics::computeVoronoiArea`](c:\Users\cagopa\Desktop\Digital-Geometry-Processing\hw2\helpers.cpp#L119) is incomplete. Implement the correct area calculation for these cases (often involves using `TriangleArea / 2` or `TriangleArea / 4` depending on which angle is obtuse).

## Improvements / Robustness

-   **[ ] Implement Candidate Pair Filtering (`main.cpp`):**
    -   Uncomment and implement the optional filtering section in [`computeAndVisualizeSymmetry`](c:\Users\cagopa\Desktop\Digital-Geometry-Processing\hw2\main.cpp#L634-L639). Filter pairs (p, q) based on geodesic distance (using [`mesh->computeGeodesicDistances`](c:\Users\cagopa\Desktop\Digital-Geometry-Processing\hw2\Mesh.cpp#L338-L380)) or other heuristics mentioned in the paper to improve efficiency by skipping unlikely symmetric pairs.
-   **[x] Handle Degenerate Cases in Iso-curve Length (`helpers.cpp`):**
    -   Address the `TODO` in [`PairwiseHarmonics::computeIsoCurveLength`](c:\Users\cagopa\Desktop\Digital-Geometry-Processing\hw2\helpers.cpp#L460). Decide how to handle edges where both vertex harmonic values equal the `isoValue`.
-   **[x] Handle Iso-curve Points on p/q in D Descriptor (`helpers.cpp`):**
    -   Address the `TODO` in [`PairwiseHarmonics::computeDDescriptor`](c:\Users\cagopa\Desktop\Digital-Geometry-Processing\hw2\helpers.cpp#L553). Consider the implications if an isocurve point happens to land exactly on vertex `p` or `q`.
-   **[x] Improve Iso-curve Segment Extraction (`helpers.cpp`):**
    -   Address the `TODO` in [`PairwiseHarmonics::extractIsoCurveSegments`](c:\Users\cagopa\Desktop\Digital-Geometry-Processing\hw2\helpers.cpp#L686). The current implementation might be basic; enhance it to correctly handle multiple intersections per face or complex topologies near critical points if needed for robustness.
-   **[x] Implement Robust Normal Calculation (`Mesh.cpp`):**
    -   Address the `TODO` in [`Mesh::computeNormal`](c:\Users\cagopa\Desktop\Digital-Geometry-Processing\hw2\Mesh.cpp#L136). Implement a more robust method, like weighting face normals by angle, for better vertex normal estimation.
-   **[ ] Review Laplacian Boundary Edge Handling (`helpers.cpp`):**
    -   Double-check the implementation in [`PairwiseHarmonics::computeCotangentLaplacian`](c:\Users\cagopa\Desktop\Digital-Geometry-Processing\hw2\helpers.cpp#L161-L232) regarding the comment on handling boundary edges ([`helpers.cpp:198`](c:\Users\cagopa\Desktop\Digital-Geometry-Processing\hw2\helpers.cpp#L198)) to ensure it's correct (e.g., using half-cotangent weights).

## Optional / Performance

-   **[ ] Use Precomputed Geodesics (`helpers.cpp`):**
    -   Address the `TODO` in [`PairwiseHarmonics::computeDDescriptor`](c:\Users\cagopa\Desktop\Digital-Geometry-Processing\hw2\helpers.cpp#L571). If reliable precomputed geodesic distance matrices are available (like `geodesic_distance_matrix.txt`), modify the function to use them instead of recomputing distances on the fly, which could significantly speed up D descriptor calculation.
-   **[ ] Explore Alternative Geodesic Algorithms (`Mesh.cpp`):**
    -   Address the `TODO` in [`Mesh::computeGeodesicDistances`](c:\Users\cagopa\Desktop\Digital-Geometry-Processing\hw2\Mesh.cpp#L376). Consider implementing or integrating a different algorithm like Fast Marching Method if Dijkstra's proves too slow or inaccurate for the required geodesic distances.