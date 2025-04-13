# Notes on Primary Papers

## Approach 1: Geodesic Iso-Curves from Extremities

This approach is based on the "Geodesic Iso-Curve Signature" (GICS) paper. The core idea is to use geodesic paths and their properties.

### Key Components:

1. **Find Shape Extremities:** 
   - Identify key points on the shape that represent its extremities (like fingertips, toes, nose tip, etc.)
   - The assignment suggests using the Farthest Point Sampling (FPS) algorithm for this
   - FPS typically starts with an arbitrary point, finds the farthest point from it, then iteratively finds points farthest from the set of already selected points
   - Note: Potentially discarding the initial random point is a refinement to ensure the selected extremities are more representative

2. **Grow Geodesic Iso-Curves:** 
   - From two chosen extremity points (potential symmetry pairs), compute geodesic iso-curves (or geodesic circles)
   - These are curves on the mesh surface where all points are at the same geodesic distance from the starting extremity point
   - Compute these curves for increasing radii
   - The GICS paper focuses on using the lengths of these iso-curves at different radii as a descriptor for points, capturing both local geometry and topology

3. **Find Intersection:**
   - The assignment requires "start geodesic isocurves [...] from two extremities and take the first intersection of them with sufficient overlap"
   - This means growing the iso-curves simultaneously from two candidate symmetric extremities
   - The symmetry axis would relate to where these expanding iso-curves "meet" or intersect significantly
   - Note: The GICS paper primarily discusses using the iso-curve lengths as a signature for matching points, rather than directly finding the axis via intersection

4. **Geodesic Paths:**
   - This method fundamentally relies on computing geodesic distances and paths to define the iso-curves

## Approach 2: Pairwise Harmonics

This approach is detailed in the "Pairwise Harmonics for Shape Analysis" paper. Instead of relying primarily on geodesic distance propagation from single points, it uses harmonic functions defined between pairs of points.

### Key Components:

1. **Define Pairwise Harmonic Function:** 
   - For a pair of points $(p, q)$ on the mesh surface, a specific harmonic function, denoted $f_{p,q}$, is computed
   - This function satisfies the Laplace equation ($\Delta f = 0$) with fixed boundary conditions: $f(p) = 0$ and $f(q) = 1$
   - Creates a smooth scalar field across the surface ranging from 0 to 1
   - The paper uses a discretization based on the cotangent Laplacian

2. **Analyze Iso-Curves of the Harmonic Function:**
   - Analyzes the iso-curves (level sets) of this harmonic function $f_{p,q}$
   - These differ from geodesic iso-curves; they represent lines where the harmonic function value is constant

3. **Compute Descriptors (R and D):**
   - Two key descriptors are extracted from these harmonic iso-curves, typically sampled uniformly between 0 and 1:
   
   a) **Perimeter Descriptor (R):** 
      - The distribution of the lengths of these iso-curves
   
   b) **Distance Descriptor (D):** 
      - The distribution of the average geodesic distances from points on the iso-curves back to the original points $p$ and $q$

4. **Identify Symmetry Axis:** 
   - A point pair $(p, q)$ is considered intrinsically symmetric if the distributions of their R and D descriptors exhibit symmetry
   - The paper defines a "Path Intrinsic Symmetry" (PIS) measure based on comparing corresponding R and D values ($r^i_{p,q}$ vs $r^{K-i}_{q,p}$ and $d^i_{p,q}$ vs $d^{K-i}_{q,p}$)
   - To find the overall symmetry axis/axes:
     - Use a sampling strategy (geodesic entropy to find candidate points)
     - Filter candidate pairs
     - Compute their PIS score
     - Apply a voting scheme where high-scoring pairs vote for faces on the potential symmetry axis

5. **Higher-Level Algorithms:** 
   - These R and D descriptors serve as foundational elements for various shape analysis tasks:
     - Computing the intrinsic reflectional symmetry axis
     - Matching extremities
     - Simultaneous segmentation/skeletonization

6. **Geodesic Paths:** 
   - While geodesic distance is used to compute the 'D' descriptor and in the sampling strategy
   - The core field generation relies on solving the Laplace equation (harmonic function) rather than directly propagating geodesic fronts
   - This method is primarily based on harmonic fields, not geodesic paths in the same way as GICS

## Summary:

- **Approach 1** uses Farthest Point Sampling to find extremities and analyzes the intersection of geodesic iso-curves grown from these points. It heavily relies on geodesic paths.
- **Approach 2** computes harmonic functions between pairs of points and analyzes the symmetry of descriptors derived from the level sets of these harmonic fields. It uses geodesic distance secondarily but is fundamentally based on solving the Laplace equation.