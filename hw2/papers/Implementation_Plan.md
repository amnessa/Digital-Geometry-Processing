## **Implementation Plan: Pairwise Harmonics for Intrinsic Symmetry Detection**

This plan outlines the steps to implement the intrinsic reflectional symmetry axis detection based on the "Pairwise Harmonics for Shape Analysis" paper (Zheng et al., TVCG 2013).  
**Reference Paper:** Zheng et al., "Pairwise Harmonics for Shape Analysis", TVCG 2013 \[cite: 1\]  
**Goal:** Compute the intrinsic reflectional symmetry axis/regions of a given 3D mesh.  
**Prerequisites:**

1. **Mesh Data Structure:** Your existing Mesh class (Mesh.h, Mesh.cpp) should provide access to:  
   * Vertices (positions)  
   * Faces (vertex indices)  
   * Edge information (or ability to compute it)  
   * Vertex/Face adjacency information (neighboring vertices/faces)  
2. **C++ Build System:** CMake configured (as indicated by CMakeLists.txt).  
3. **Linear Algebra Library:** Eigen (or a similar library) for sparse matrix operations and solving linear systems. Ensure it's correctly linked in your CMake configuration.  
4. **(Optional but Recommended) Geodesic Distance:** A function to compute geodesic distances between vertices. You might have this from HW1 or need to implement/integrate one (e.g., using Dijkstra's algorithm on the mesh graph or the Fast Marching Method). The presence of geodesic\_distance\_matrix.txt suggests you might already have precomputed distances for some meshes. You'll need this for the 'D' descriptor and potentially for sampling.  
5. **(Optional) Visualization:** Your Painter class (Painter.h, Painter.cpp) or similar mechanism to visualize the mesh, scalar fields (like the harmonic function), curves (iso-curves, symmetry axis), and selected points/faces.

**Implementation Steps:**  
**Phase 1: Core Harmonic Function Computation**

1. **Build Cotangent Laplacian Matrix (L):**  
   * Implement a function (e.g., Mesh::computeCotangentLaplacian()) that computes the cotangent Laplacian matrix L.  
   * For each edge (i, j), the corresponding entry L\_{ij} is typically calculated as \-(cot(\\alpha\_{ij}) \+ cot(\\beta\_{ij})) / 2, where \\alpha\_{ij} and \\beta\_{ij} are the angles opposite the edge (i, j) in the two triangles sharing it. Handle boundary edges appropriately (often using half the cotangent from the single adjacent triangle).  
   * The diagonal entry L\_{ii} is \-\\sum\_{j \\neq i} L\_{ij}.  
   * *Weighting:* The paper mentions weighting by the one-ring Voronoi area per vertex. You can incorporate this or start with the simpler uniform Laplacian (L\_{ij} \= \-1 for neighbors, L\_{ii} \= \\text{degree}(i)) and refine later. The cotangent weights are generally preferred.  
   * Store L as a sparse matrix (e.g., Eigen::SparseMatrix\<double\>).  
2. **Implement Pairwise Harmonic Solver:**  
   * Create a function computePairwiseHarmonic(vertex\_p\_idx, vertex\_q\_idx) that takes the indices of the two boundary vertices p and q.  
   * Inside this function:  
     * Make a copy of the global Laplacian L.  
     * Initialize the right-hand side vector b (e.g., Eigen::VectorXd) to zeros.  
     * **Modify L and b for Boundary Conditions:**  
       * Zero out the p-th row of the copied L. Set the diagonal element L(p, p) \= 1.0. Set b(p) \= 0.0.  
       * Zero out the q-th row of the copied L. Set the diagonal element L(q, q) \= 1.0. Set b(q) \= 1.0.  
     * **Solve the System:** Use a sparse linear solver from Eigen to solve Lf \= b for f. Choose an appropriate solver (e.g., Eigen::SparseLU or Eigen::BiCGSTAB might be suitable given the modifications for boundary conditions).  
     * Return the resulting vector f (the harmonic field values per vertex).

**Phase 2: Descriptor Computation**

3. **Implement Iso-Curve Extraction:**  
   * Create a function extractIsoCurve(harmonic\_field\_f, iso\_value) that takes the computed harmonic field f and a target iso\_value (between 0 and 1).  
   * For each edge (i, j) of the mesh, check if the iso\_value lies between f(i) and f(j).  
   * If it does, linearly interpolate to find the point x on the edge where the function value equals iso\_value. Store these intersection points.  
   * Connect these intersection points across faces to form iso-curve segments. This can be tricky; you might need to handle multiple intersections per face and different topologies. Start simple, perhaps just collecting the intersection points and their lengths initially.  
   * The output should be one or more lists of connected points representing the iso-curve(s) for the given iso\_value.  
4. **Compute R Descriptor (Perimeter):**  
   * Create a function computeRDescriptor(vertex\_p\_idx, vertex\_q\_idx, harmonic\_field\_f, num\_samples\_K).  
   * Sample K iso-values uniformly between 0 and 1 (e.g., t\_k \= k / (K+1) for k=1...K).  
   * For each t\_k, call extractIsoCurve(harmonic\_field\_f, t\_k).  
   * Calculate the total length of the extracted iso-curve(s) for t\_k. This is the k-th value of the R descriptor, r^k\_{p,q}.  
   * Return the vector of K perimeter values (the R descriptor).  
5. **Compute D Descriptor (Average Distance):**  
   * Create a function computeDDescriptor(vertex\_p\_idx, vertex\_q\_idx, harmonic\_field\_f, num\_samples\_K). Requires a geodesic distance function computeGeodesicDistance(v\_start, v\_end).  
   * Sample K iso-values uniformly as in step 4\.  
   * For each t\_k:  
     * Extract the iso-curve(s) using extractIsoCurve(harmonic\_field\_f, t\_k).  
     * For each point x on the iso-curve(s):  
       * Compute the geodesic distances d(x, p) and d(x, q). Note: Since x lies on an edge, you might need to approximate this or use distances from the edge's vertices.  
     * Calculate the average geodesic distance d^k\_{p,q} \= \\text{avg}\_{x \\in \\text{iso-curve}(t\_k)}(d(x, p) \+ d(x, q)).  
   * Return the vector of K average distance values (the D descriptor).

**Phase 3: Symmetry Detection and Voting**

6. **Compute Path Intrinsic Symmetry (PIS) Score:**  
   * Create a function computePIS(R\_pq, D\_pq, R\_qp, D\_qp) that takes the R and D descriptors for the pair (p, q) and the pair (q, p).  
   * Implement the PIS formula from the paper (Section 3.1), which compares r^k\_{p,q} with r^{K-k}\_{q,p} and d^k\_{p,q} with d^{K-k}\_{q,p}.  
   * Return the scalar PIS score for the pair (p, q).  
7. **Candidate Pair Sampling and Filtering:**  
   * **Sampling:** Select a set of candidate vertex pairs (p, q) to test.  
     * *Paper Method:* Implement geodesic entropy sampling (Section 4.1) to find points with high local variation.  
     * *Alternative:* Use your existing FPS implementation to get a set of well-distributed points, and consider pairs among them.  
     * *Simpler Start:* Randomly sample pairs of vertices.  
   * **Filtering:** (Optional, for efficiency) Filter pairs based on geodesic distance or other heuristics mentioned in the paper.  
8. **Implement Voting Scheme:**  
   * Initialize a vote accumulator for each face (or vertex) of the mesh to zero.  
   * For each candidate pair (p, q) that passes filtering:  
     * Compute f\_{p,q} (Step 2).  
     * Compute f\_{q,p} (Step 2).  
     * Compute R and D descriptors for both (p, q) and (q, p) (Steps 4, 5).  
     * Compute the PIS score for the pair (Step 6).  
     * If PIS score is high (above a threshold):  
       * Identify the "midpoint" iso-curve (e.g., corresponding to t\_k \\approx 0.5).  
       * Find faces that are close to or intersected by this midpoint iso-curve.  
       * Add a vote (weighted by the PIS score) to these faces.  
9. **Extract and Visualize Symmetry Axis/Regions:**  
   * Identify faces (or vertices/edges) with the highest accumulated votes. These form the potential intrinsic reflectional symmetry axis/axes or regions.  
   * Use your Painter class to visualize:  
     * The input mesh.  
     * Selected candidate pairs (p, q).  
     * The computed harmonic field f\_{p,q} (color-mapped onto the mesh).  
     * Extracted iso-curves (especially the midpoint curve).  
     * The final symmetry axis/regions (highlighted faces/edges).

**Integration into Project:**

* Add new methods to your Mesh class for Laplacian computation, geodesic distance (if needed), and potentially iso-curve extraction helpers.  
* Consider creating a new class (e.g., SymmetryAnalyzer) to encapsulate the steps related to harmonic computation, descriptor calculation, PIS scoring, and voting.  
* Modify main.cpp to orchestrate the process: load mesh, select pairs (or run sampling), compute symmetry, visualize results.  
* Update CMakeLists.txt to link against Eigen and any other new dependencies.

**Testing and Visualization Strategy:**

* **Unit Test:** Test the Laplacian computation on simple known meshes.  
* **Visualize f\_{p,q}:** Pick two points p, q manually and visualize the resulting harmonic field. Verify it goes from 0 to 1 smoothly.  
* **Visualize Iso-curves:** Visualize the extracted iso-curves for different t\_k values. Check their lengths and shapes.  
* **Debug Descriptors:** Print or plot the R and D descriptor values for known symmetric/asymmetric pairs to see if they behave as expected.  
* **Visualize Voting:** Color-map the accumulated votes onto the mesh faces to see where the symmetry axis is emerging.

**Potential Challenges & Simplifications:**

* **Robust Iso-curve Extraction:** Connecting intersection points correctly can be complex, especially near critical points of the harmonic function. Start with just calculating lengths based on edge intersections.  
* **Geodesic Distance:** Accurate geodesic computation can be slow or complex to implement. Use approximations or existing libraries if possible. Your precomputed matrices might suffice for testing.  
* **Sampling:** Geodesic entropy sampling adds complexity. Start with FPS or random sampling.  
* **Performance:** Computing harmonic fields and descriptors for many pairs can be slow. Implement optimizations like filtering pairs early.

This plan provides a structured approach. Start with Phase 1, ensure the harmonic field computation is correct, then move to descriptors and finally the voting scheme. Good luck\!