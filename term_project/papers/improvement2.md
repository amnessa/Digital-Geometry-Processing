### Improvement 2: Robust Skeleton Connectivity via SDF and MST

This directly solves the "skeleton goes outside the mesh" problem and gives you a clean, connected final skeleton.

* **Concept:** After identifying your final joints (the endpoints of your `rigidSegments`), connect them using a Minimum Spanning Tree (MST). The crucial part is that the cost of an edge (a potential bone) is heavily penalized if it travels outside the mesh volume.
* **Inspiration:** The pipeline from "Predicting Animation Skeletons via Volumetric Nets", which uses a volumetric representation and an MST to ensure valid connectivity.
* **Actionable Steps:**

    1.  **Identify Joints:** Your joints are the start and end points of the `rigidSegments` you found in Step 2 of your pipeline.
    2.  **Compute a Signed Distance Function (SDF):** Create a 3D grid around your mesh and compute the SDF. This gives you an "inside/outside" map of the volume.
    3.  **Build a Cost-Weighted Graph of Joints:**
        * Create a graph where every joint is a node.
        * For every pair of joints `(j1, j2)`, calculate the cost of the edge connecting them.
        * To do this, trace the straight line between `j1` and `j2`, sampling points along it. For each sample point, check its value in the SDF grid.
        * `cost(j1, j2) = length(j1, j2) + BigPenalty * (number of samples outside the mesh)`
        * If the SDF value is positive, the point is outside, so you add a large penalty.

    4.  **Extract the Final Skeleton:** Run a standard MST algorithm (like Prim's or Kruskal's) on this graph. The result is a guaranteed-to-be-connected tree skeleton that strongly prefers to stay inside the mesh.

* **Benefit for Your Report:** You can show how you solved a key robustness issue of the original algorithm by integrating a volumetric data structure (SDF) to guide a graph-based skeletonization (MST), producing geometrically valid results.
