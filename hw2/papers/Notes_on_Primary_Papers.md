Approach 1: Geodesic Iso-Curves from Extremities

This approach is based on the "Geodesic Iso-Curve Signature" (GICS) paper. The core idea is to use geodesic paths and their properties.  

    Find Shape Extremities: First, you need to identify key points on the shape that represent its extremities (like fingertips, toes, nose tip, etc.). The assignment suggests using the Farthest Point Sampling (FPS) algorithm for this. As you noted, FPS typically starts with an arbitrary point, finds the farthest point from it, then iteratively finds points farthest from the set of already selected points. Your lecturer's note about potentially discarding the initial random point is a refinement to ensure the selected extremities are more representative.
    Grow Geodesic Iso-Curves: From two chosen extremity points (potential symmetry pairs), you compute geodesic iso-curves (or geodesic circles). These are curves on the mesh surface where all points are at the same geodesic distance (distance along the surface) from the starting extremity point. You compute these curves for increasing radii. The GICS paper focuses on using the lengths of these iso-curves at different radii as a descriptor for points, capturing both local geometry and topology.   

Find Intersection: The assignment requires you to "start geodesic isocurves [...] from two extremities and take the first intersection of them with sufficient overlap". This means you'd likely grow the iso-curves simultaneously from two candidate symmetric extremities. The symmetry axis would relate to where these expanding iso-curves "meet" or intersect significantly. The GICS paper itself primarily discusses using the iso-curve lengths as a signature for matching points, rather than directly finding the axis via intersection, but the concept relies heavily on computing these geodesic iso-curves emanating from points.  

    Geodesic Paths: Yes, as your lecturer mentioned, this method fundamentally relies on computing geodesic distances and paths to define the iso-curves.   

Approach 2: Pairwise Harmonics

This approach is detailed in the "Pairwise Harmonics for Shape Analysis" paper. Instead of relying primarily on geodesic distance propagation from single points, it uses harmonic functions defined between pairs of points.  

    Define Pairwise Harmonic Function: For a pair of points (p, q) on the mesh surface, a specific harmonic function, denoted fp,q​, is computed. This function satisfies the Laplace equation (Δf=0) and has fixed boundary conditions: f(p)=0 and f(q)=1. This creates a smooth scalar field across the surface ranging from 0 to 1. The paper uses a discretization based on the cotangent Laplacian.   

Analyze Iso-Curves of the Harmonic Function: The method then analyzes the iso-curves (level sets) of this harmonic function fp,q​. These are different from the geodesic iso-curves in the first approach; they represent lines where the harmonic function value is constant.  
Compute Descriptors (R and D): Two key descriptors are extracted from these harmonic iso-curves, typically sampled uniformly between the 0 and 1 values:

    Perimeter Descriptor (R): The distribution of the lengths of these iso-curves.   

Distance Descriptor (D): The distribution of the average geodesic distances from points on the iso-curves back to the original points p and q.  

 
Identify Symmetry Axis: A point pair (p, q) is considered intrinsically symmetric if the distributions of their R and D descriptors exhibit symmetry. The paper defines a "Path Intrinsic Symmetry" (PIS) measure based on comparing corresponding R and D values (rp,qi​ vs rq,pK−i​ and dp,qi​ vs dq,pK−i​).

    To find the overall symmetry axis/axes of the shape, the paper proposes a sampling strategy (using geodesic entropy to find candidate points), filtering candidate pairs, computing their PIS score, and using a voting scheme where high-scoring pairs vote for faces lying on the potential symmetry axis.   

 
Higher-Level Algorithms: As you noted, these R and D descriptors, derived from the pairwise harmonic field, serve as foundational elements for various shape analysis tasks, including computing the intrinsic reflectional symmetry axis, matching extremities, and simultaneous segmentation/skeletonization.  

    Geodesic Paths: While geodesic distance is used to compute the 'D' descriptor and in the sampling strategy, the core field generation relies on solving the Laplace equation (harmonic function) rather than directly propagating geodesic fronts like the first method. So, the lecturer's distinction seems apt – this method is primarily based on harmonic fields, not geodesic paths in the same way as GICS.   

In summary:

    Approach 1 uses Farthest Point Sampling to find extremities and analyzes the intersection of geodesic iso-curves (curves of constant geodesic distance) grown from these points. It heavily relies on geodesic paths.
    Approach 2 computes harmonic functions between pairs of points and analyzes the symmetry of descriptors (R - iso-curve lengths, D - average geodesic distance to endpoints) derived from the level sets (iso-curves) of these harmonic fields. It uses geodesic distance secondarily (for the D descriptor and sampling) but is fundamentally based on solving the Laplace equation.