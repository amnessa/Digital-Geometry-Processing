This system allows you to test individual components
of the segmentation algorithm before running the full pipeline.

Algorithm implements:
- Cotangent Laplacian computation
- FPS with arbitrary point start + discard
- Pairwise harmonic field computation
- Iso-curve extraction for skeletal segments
- PCA-based rigidity analysis
- Segmentation via isocurve boundaries

======================================
PAIRWISE HARMONICS SEGMENTATION SYSTEM
======================================
1. Load Mesh
2. Test Cotangent Laplacian
3. Test Farthest Point Sampling
4. Test Pairwise Harmonics
5. Test Iso-curve Extraction
6. Test Rigidity Analysis
7. Perform Full Segmentation
8. Clear Visualization
--- ENHANCED ANALYSIS ---
9. Improved Iso-curve Extraction (Dense Bracelets)
10. Enhanced Rigidity Analysis (All Nodes)
11. Visualize All Rigidity Points (Color-coded)
12. Interactive Rigidity Threshold Testing
--- LIBIGL ENHANCED FEATURES ---
13. Test Heat Geodesics (LibIGL)
14. Enhanced FPS with Heat Geodesics
15. Compare Geodesic Methods (Dijkstra vs Heat)
16. Enhanced R&D Descriptors (LibIGL)
17. Mesh Analysis (Normals, Curvature, Areas)
0. Exit
======================================
Choose option: 1

=== LOAD MESH ===
Available meshes:
  - 50.off
  - 249.off
  - 348.off
  - man0.off
  - Armadillo.off
Enter mesh filename: 249.off
No visualizations to clear.
Loading mesh: models/249.off...
Mesh normalized: Scale=1.018360, Center=(-0.013494,-0.120342,-0.035174)
Mesh loaded successfully!
  Vertices: 3714
  Triangles: 7424
  Edges: 11136

Initializing LibIGL for robust computations...
LibIGL initialization successful!
Precomputing heat geodesics for enhanced performance...
Error: Mesh not converted to LibIGL format. Call convertToLibIGL() first.
Warning: Heat geodesics precomputation failed. Using Dijkstra fallback.
Mesh ready for pairwise harmonics analysis with LibIGL support!


======================================
Choose option: 2

=== TESTING COTANGENT LAPLACIAN ===
Choose implementation:
1. libigl-based (recommended - more robust)
2. Custom implementation (original)
Enter choice (1 or 2): 1
Computing cotangent Laplacian using libigl...
SUCCESS: Cotangent Laplacian computed successfully!
  Implementation: libigl
  Computation time: 0.078 seconds
  Matrix size: 3714x3714
  Properties:
    - Intrinsic (uses cotangent weights)
    - libigl robust implementation
    - Includes mass matrix computation
    - Symmetric and positive semi-definite
    - Ready for harmonic field computation

    ======================================
PAIRWISE HARMONICS SEGMENTATION SYSTEM
======================================
1. Load Mesh
2. Test Cotangent Laplacian
3. Test Farthest Point Sampling
4. Test Pairwise Harmonics
5. Test Iso-curve Extraction
6. Test Rigidity Analysis
7. Perform Full Segmentation
8. Clear Visualization
--- ENHANCED ANALYSIS ---
9. Improved Iso-curve Extraction (Dense Bracelets)
10. Enhanced Rigidity Analysis (All Nodes)
11. Visualize All Rigidity Points (Color-coded)
12. Interactive Rigidity Threshold Testing
--- LIBIGL ENHANCED FEATURES ---
13. Test Heat Geodesics (LibIGL)
14. Enhanced FPS with Heat Geodesics
15. Compare Geodesic Methods (Dijkstra vs Heat)
16. Enhanced R&D Descriptors (LibIGL)
17. Mesh Analysis (Normals, Curvature, Areas)
0. Exit
======================================
Choose option: 3

=== TESTING FARTHEST POINT SAMPLING ===
Enter number of FPS samples (recommended: 25-40 for human models, 15-25 for simple shapes): 6
Computing FPS with 6 points...
Algorithm: 1) Pick arbitrary point -> 2) Find farthest -> 3) Discard arbitrary -> 4) Continue FPS
SUCCESS: FPS computed successfully!
  Computation time: 7.023 seconds
  Sample points: 2594 3073 3706 3 1682 1993
Only mesh present, nothing to clear.
FPS points visualized (first point is red, others blue).



=== TESTING PAIRWISE HARMONICS ===
Choose implementation:
1. libigl-based (recommended - more robust)
2. Custom implementation (original)
Enter choice (1 or 2): 1
Enter first vertex index (p, boundary value 0): 1
Enter second vertex index (q, boundary value 1): 100
Computing harmonic field between vertices 1 and 100...
Using libigl implementation...
SUCCESS: Pairwise harmonic computed successfully!
  Implementation: libigl
  Computation time: 0.235 seconds
  Field range: [0, 1]
Cleared 6 visualization elements, preserving mesh.
Visualizing scalar field: Harmonic Field
Field range: [0, 1]

Computing shape descriptors...
R descriptor (isocurve lengths): 0.0379457 0.0758913 0.1327 0.195053 0.268099 0.371128 1.137 0.23436 0.109265 0.0546325
Harmonic field visualized (blue = value 0, red = value 1).

WORKING BUT NEED HIGHER COLOR GRADIENT

======================================
Choose option: 5

=== TESTING ISO-CURVE EXTRACTION ===
Computing harmonic field between FPS vertices 2594 and 3073...
Extracting iso-curves at different levels...
Cleared 6 visualization elements, preserving mesh.
  Iso-value 0.2: 36 segments, total length = 0.291306
  Iso-value 0.4: 61 segments, total length = 0.586721
  Iso-value 0.6: 65 segments, total length = 0.585839
  Iso-value 0.8: 34 segments, total length = 0.246601
SUCCESS: Iso-curve extraction complete!
  Generated 4 skeletal nodes
Iso-curves visualized with color-coded levels.

VERY GOOD. BUT MAYBE MORE CURVES NEEDED


======================================
PAIRWISE HARMONICS SEGMENTATION SYSTEM
======================================
1. Load Mesh
2. Test Cotangent Laplacian
3. Test Farthest Point Sampling
4. Test Pairwise Harmonics
5. Test Iso-curve Extraction
6. Test Rigidity Analysis
7. Perform Full Segmentation
8. Clear Visualization
--- ENHANCED ANALYSIS ---
9. Improved Iso-curve Extraction (Dense Bracelets)
10. Enhanced Rigidity Analysis (All Nodes)
11. Visualize All Rigidity Points (Color-coded)
12. Interactive Rigidity Threshold Testing
--- LIBIGL ENHANCED FEATURES ---
13. Test Heat Geodesics (LibIGL)
14. Enhanced FPS with Heat Geodesics
15. Compare Geodesic Methods (Dijkstra vs Heat)
16. Enhanced R&D Descriptors (LibIGL)
17. Mesh Analysis (Normals, Curvature, Areas)
0. Exit
======================================
Choose option: 7
Cleared 4 visualization elements, preserving mesh.

=== SEGMENTATION PARAMETERS ===
Use default parameters? (y/n): Y

Starting pairwise harmonics segmentation...

=== PAIRWISE HARMONICS SEGMENTATION ===
Implementation based on 'Pairwise Harmonics for Shape Analysis'
by Zheng et al., IEEE TVCG 2013

Parameters:
  - FPS samples: 25
  - Isocurves per field (K): 50
  - Rigidity threshold: 0.9

--- STAGE 1: FARTHEST POINT SAMPLING ---
Generated 25 FPS samples

--- STAGE 2: SKELETAL SEGMENT GENERATION ---
Computing pairwise harmonic fields for 300 point pairs...
  Pair 1/300 (vertices 2594, 3073)
  Pair 2/300 (vertices 2594, 3706)
  Pair 3/300 (vertices 2594, 3)
  Pair 4/300 (vertices 2594, 1682)
  Pair 5/300 (vertices 2594, 1993)
  Pair 10/300 (vertices 2594, 1702)
  Pair 20/300 (vertices 2594, 1899)
  Pair 30/300 (vertices 3073, 1257)
  Pair 40/300 (vertices 3073, 1650)
  Pair 50/300 (vertices 3706, 1993)
  Pair 60/300 (vertices 3706, 1326)
  Pair 70/300 (vertices 3, 1682)
  Pair 80/300 (vertices 3, 405)
  Pair 90/300 (vertices 3, 1048)
  Pair 100/300 (vertices 1682, 405)
  Pair 110/300 (vertices 1682, 1048)
  Pair 120/300 (vertices 1993, 1326)
  Pair 130/300 (vertices 3307, 1257)
  Pair 140/300 (vertices 3307, 1650)
  Pair 150/300 (vertices 1257, 1702)
  Pair 160/300 (vertices 1257, 1899)
  Pair 170/300 (vertices 1777, 405)
  Pair 180/300 (vertices 1777, 1048)
  Pair 190/300 (vertices 1079, 177)
  Pair 200/300 (vertices 1702, 1326)
  Pair 210/300 (vertices 3352, 680)
  Pair 220/300 (vertices 3352, 185)
  Pair 230/300 (vertices 680, 1899)
  Pair 240/300 (vertices 3265, 177)
  Pair 250/300 (vertices 405, 177)
  Pair 260/300 (vertices 1326, 1899)
  Pair 270/300 (vertices 2646, 185)
  Pair 280/300 (vertices 3466, 177)
  Pair 290/300 (vertices 177, 1048)
  Pair 300/300 (vertices 307, 1048)
Generated 300 raw skeletal segments

--- STAGE 3: RIGIDITY ANALYSIS ---
Computing rigidity for 300 segments...
Selected 8 segments for partial skeleton

--- STAGE 4: MESH SEGMENTATION ---
Created 3 mesh components

--- STAGE 5: SKELETON COMPLETION ---
Completed skeleton with 3 nodes

=== SEGMENTATION COMPLETE ===

Visualization complete:
  - 8 skeletal segments (colored by rigidity)
  - 3 mesh components
  - 3 skeleton nodes (magenta points)

=== SEGMENTATION SUMMARY ===
Ô£ô Successfully segmented mesh into 3 components
Ô£ô Generated partial skeleton with 8 segments
Ô£ô Created complete skeleton with 3 nodes
Ô£ô Used 25 FPS sample points

OKAY WORKING. BUT FOR EXAMPLE FOR A BODY PART IT GENERATES PARALLEL SKELETON PARTS. ALSO SOME SKELETON PARTS GOES OUTSIDE THE MESH. IN ADDITION WE NEED TO COLOR SEGMENTED VERTICES BY DIFFERENT COLORS. RIGHT NOW THERE IS NO VISUAL INDICATION OF SEGMENTED VERTICES. ONLY SKELETON IS GENERATED. ANOTHER POINT IS OPTIMIZATION. WE NEED TO INVESTIGATE THE BIG O COMPLEXITY OF THE ALGORITHM.


======================================
PAIRWISE HARMONICS SEGMENTATION SYSTEM
======================================
1. Load Mesh
2. Test Cotangent Laplacian
3. Test Farthest Point Sampling
4. Test Pairwise Harmonics
5. Test Iso-curve Extraction
6. Test Rigidity Analysis
7. Perform Full Segmentation
8. Clear Visualization
--- ENHANCED ANALYSIS ---
9. Improved Iso-curve Extraction (Dense Bracelets)
10. Enhanced Rigidity Analysis (All Nodes)
11. Visualize All Rigidity Points (Color-coded)
12. Interactive Rigidity Threshold Testing
--- LIBIGL ENHANCED FEATURES ---
13. Test Heat Geodesics (LibIGL)
14. Enhanced FPS with Heat Geodesics
15. Compare Geodesic Methods (Dijkstra vs Heat)
16. Enhanced R&D Descriptors (LibIGL)
17. Mesh Analysis (Normals, Curvature, Areas)
0. Exit
======================================
Choose option: 9

=== IMPROVED ISO-CURVE EXTRACTION ===
Extracting robust isocurves using triangle marching and graph stitching
Computing harmonic field between FPS vertices 2594 and 3073...
Extracting 25 isocurves using robust triangle marching...
  Extracting isocurves at value 0.04 using robust triangle marching...
    Generated 12 triangle segments
    Unified to 12 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 12 points
  Isocurve 1 (value=0.04): 12 points in continuous polyline
  Extracting isocurves at value 0.08 using robust triangle marching...
    Generated 18 triangle segments
    Unified to 18 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 18 points
  Isocurve 2 (value=0.08): 18 points in continuous polyline
  Extracting isocurves at value 0.12 using robust triangle marching...
    Generated 28 triangle segments
    Unified to 28 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 28 points
  Isocurve 3 (value=0.12): 28 points in continuous polyline
  Extracting isocurves at value 0.16 using robust triangle marching...
    Generated 34 triangle segments
    Unified to 34 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 34 points
  Isocurve 4 (value=0.16): 34 points in continuous polyline
  Extracting isocurves at value 0.2 using robust triangle marching...
    Generated 36 triangle segments
    Unified to 36 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 36 points
  Isocurve 5 (value=0.2): 36 points in continuous polyline
  Extracting isocurves at value 0.24 using robust triangle marching...
    Generated 41 triangle segments
    Unified to 41 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 41 points
  Isocurve 6 (value=0.24): 41 points in continuous polyline
  Extracting isocurves at value 0.28 using robust triangle marching...
    Generated 46 triangle segments
    Unified to 46 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 46 points
  Isocurve 7 (value=0.28): 46 points in continuous polyline
  Extracting isocurves at value 0.32 using robust triangle marching...
    Generated 52 triangle segments
    Unified to 52 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 52 points
  Isocurve 8 (value=0.32): 52 points in continuous polyline
  Extracting isocurves at value 0.36 using robust triangle marching...
    Generated 57 triangle segments
    Unified to 57 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 57 points
  Isocurve 9 (value=0.36): 57 points in continuous polyline
  Extracting isocurves at value 0.4 using robust triangle marching...
    Generated 61 triangle segments
    Unified to 61 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 61 points
  Isocurve 10 (value=0.4): 61 points in continuous polyline
  Extracting isocurves at value 0.44 using robust triangle marching...
    Generated 64 triangle segments
    Unified to 64 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 64 points
  Isocurve 11 (value=0.44): 64 points in continuous polyline
  Extracting isocurves at value 0.48 using robust triangle marching...
    Generated 118 triangle segments
    Unified to 118 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 118 points
  Isocurve 12 (value=0.48): 118 points in continuous polyline
  Extracting isocurves at value 0.52 using robust triangle marching...
    Generated 70 triangle segments
    Unified to 70 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 70 points
  Isocurve 13 (value=0.52): 70 points in continuous polyline
  Extracting isocurves at value 0.56 using robust triangle marching...
    Generated 61 triangle segments
    Unified to 61 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 61 points
  Isocurve 14 (value=0.56): 61 points in continuous polyline
  Extracting isocurves at value 0.6 using robust triangle marching...
    Generated 65 triangle segments
    Unified to 65 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 65 points
  Isocurve 15 (value=0.6): 65 points in continuous polyline
  Extracting isocurves at value 0.64 using robust triangle marching...
    Generated 58 triangle segments
    Unified to 58 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 58 points
  Isocurve 16 (value=0.64): 58 points in continuous polyline
  Extracting isocurves at value 0.68 using robust triangle marching...
    Generated 52 triangle segments
    Unified to 52 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 52 points
  Isocurve 17 (value=0.68): 52 points in continuous polyline
  Extracting isocurves at value 0.72 using robust triangle marching...
    Generated 46 triangle segments
    Unified to 46 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 46 points
  Isocurve 18 (value=0.72): 46 points in continuous polyline
  Extracting isocurves at value 0.76 using robust triangle marching...
    Generated 40 triangle segments
    Unified to 40 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 40 points
  Isocurve 19 (value=0.76): 40 points in continuous polyline
  Extracting isocurves at value 0.8 using robust triangle marching...
    Generated 34 triangle segments
    Unified to 34 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 34 points
  Isocurve 20 (value=0.8): 34 points in continuous polyline
  Extracting isocurves at value 0.84 using robust triangle marching...
    Generated 26 triangle segments
    Unified to 26 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 26 points
  Isocurve 21 (value=0.84): 26 points in continuous polyline
  Extracting isocurves at value 0.88 using robust triangle marching...
    Generated 23 triangle segments
    Unified to 23 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 23 points
  Isocurve 22 (value=0.88): 23 points in continuous polyline
  Extracting isocurves at value 0.92 using robust triangle marching...
    Generated 17 triangle segments
    Unified to 17 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 17 points
  Isocurve 23 (value=0.92): 17 points in continuous polyline
  Extracting isocurves at value 0.96 using robust triangle marching...
    Generated 11 triangle segments
    Unified to 11 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 11 points
  Isocurve 24 (value=0.96): 11 points in continuous polyline

Computing rigidity scores for isocurve centroids...
  Centroid 0: rigidity = 0.997361
  Centroid 1: rigidity = 0.997361
  Centroid 2: rigidity = 0.997361
  Centroid 3: rigidity = 0.997361
  Centroid 4: rigidity = 0.998345
  Centroid 5: rigidity = 0.999467
  Centroid 6: rigidity = 0.998045
  Centroid 7: rigidity = 0.992311
  Centroid 8: rigidity = 0.981668
  Centroid 9: rigidity = 0.989061
  Centroid 10: rigidity = 0.991605
  Centroid 11: rigidity = 0.992545
  Centroid 12: rigidity = 0.992475
  Centroid 13: rigidity = 0.990213
  Centroid 14: rigidity = 0.980469
  Centroid 15: rigidity = 0.978231
  Centroid 16: rigidity = 0.982842
  Centroid 17: rigidity = 0.994917
  Centroid 18: rigidity = 0.998527
  Centroid 19: rigidity = 0.997876
  Centroid 20: rigidity = 0.998156
  Centroid 21: rigidity = 0.9989
  Centroid 22: rigidity = 0.997529
  Centroid 23: rigidity = 0.999042
Visualizing rigidity points on isocurve centroids...

SUCCESS: Generated skeletal segment with 24 nodes
Yellow line shows the skeletal segment connecting isocurve centroids
Colored spheres show rigidity points (green=rigid, red=non-rigid)
Rigidity range: [0.978231, 0.999467]
Total 24 robust isocurves extracted
Robust iso-curve extraction complete - showing continuous polylines!

THIS WORKS VERY WELL. PROBABLY WORKS ON 2 FARTHEST POINTS. CONFIRM THIS.

RIGIDITY ANALYSIS :


Rigidity Statistics:
  Total nodes analyzed: 26368
  Rigidity range: [0.516802, 1]
  Average rigidity: 0.981222
  Standard deviation: 0.0522148

Rigidity Distribution:
    >= 0.5: 26368 nodes (100%)
    >= 0.6: 26307 nodes (99.7687%)
    >= 0.7: 26102 nodes (98.9912%)
    >= 0.8: 25764 nodes (97.7093%)
    >= 0.9: 25029 nodes (94.9219%)
    >= 1: 0 nodes (0%)

Enhanced rigidity analysis complete!

WORKS WELL. GENERATES TOO MUCH VERBOSE OUTPUT. MAYBE PRINTING ONLY 1 OF 10 OUTPUTS WOULD BE BETTER.


16. Enhanced R&D Descriptors (LibIGL)
17. Mesh Analysis (Normals, Curvature, Areas)
0. Exit
======================================
Choose option: 11

=== VISUALIZING ALL RIGIDITY POINTS ===
Showing continuous rigidity distribution: Green = Low Rigidity (Junctions), Red = High Rigidity (Limbs)
Using downsampling factor: 10 (showing every 10th point)
Rigidity range: [0.516802, 1]
Color mapping: Green (Ôëñ0.661762) -> Yellow -> Red (ÔëÑ0.95168)
Visualized 2637 out of 26368 total rigidity points
Creating color legend...

Visualization complete!
Downsampled node distribution (showing every 10th point):
  Low rigidity (Green, potential junctions): 19 nodes shown
  Medium rigidity (Yellow): 67 nodes shown
  High rigidity (Red, limb centers): 2551 nodes shown
  Total visualized: 2637 out of 26368 total points

Legend spheres placed on the right side of the model

RED POINTS ARE ON THE SURFACE OF THE OBJECT. IN PAPER IMPLEMENTATION GREEN POINTS ARE MOST RIGID POINTS. FOR A CYLINDER GREEN POINTS SHOULD BE ON THE MIDDLE AXIS CONNECTING TOP AND BOTTOM FACES. RED POINTS SHOULD BE FARTHER OUT FROM THE GREEN POINTS BUT STILL IN THE MODEL NOT SURFACE.

Choose option: 12

=== INTERACTIVE RIGIDITY THRESHOLD TESTING ===
This will help you find the optimal threshold for detecting junction points
Paper recommendation: Use threshold 0.9 (range 0.85-0.95 works well)

Current rigidity range: [0.516802, 1]
Total skeletal nodes: 26368

--- THRESHOLD ANALYSIS ---
Enter a rigidity threshold to test (0.4-1.0, or 'q' to quit): 0.90

Results for threshold = 0.9:
  Rigid nodes (skeletal): 25029 (94.9219%)
  Non-rigid nodes (junctions): 1339 (5.07812%)

Visualization Guide:
  GREEN spheres (large): Non-rigid nodes = Junction candidates
  BLUE-to-RED spheres (small): Rigid nodes = Skeletal centers
  -> For human models, you want ~4-8 green junction points
  -> (shoulders, hips, neck, wrists, ankles, etc.)

  SAME EXPLANATION AS ABOVE. POINTS ARE TO BE INSIDE OF MODEL TO BE PICKED AS RIGIDITY POINTS FOR SKELETONIZATION. MOST RIGID BODY PARTS ARE CONNECTED TO THE BODY. FURTHER ANALYSIS IS NEEDED ACCORDING TO THE PAPER. PAPER USES PCA ANALYSIS TO DETECT RIGIDITY POINTS.


  16. Enhanced R&D Descriptors (LibIGL)
17. Mesh Analysis (Normals, Curvature, Areas)
0. Exit
======================================
Choose option: 13

=== TESTING HEAT GEODESICS ===
This test compares LibIGL heat geodesics with Dijkstra distances
Testing heat geodesics from vertex 0...
  Heat geodesic computation successful (size: 3714)
  Max distance: 0
  Min distance: 0
Testing heat geodesics from vertex 928...
  Heat geodesic computation successful (size: 3714)
  Max distance: 0
  Min distance: 0
Testing heat geodesics from vertex 1857...
  Heat geodesic computation successful (size: 3714)
  Max distance: 0
  Min distance: 0
Testing heat geodesics from vertex 2785...
  Heat geodesic computation successful (size: 3714)
  Max distance: 0
  Min distance: 0
Heat geodesics test completed!


THIS DO NOT WORK.


1. Visualize All Rigidity Points (Color-coded)
12. Interactive Rigidity Threshold Testing
--- LIBIGL ENHANCED FEATURES ---
13. Test Heat Geodesics (LibIGL)
14. Enhanced FPS with Heat Geodesics
15. Compare Geodesic Methods (Dijkstra vs Heat)
16. Enhanced R&D Descriptors (LibIGL)
17. Mesh Analysis (Normals, Curvature, Areas)
0. Exit
======================================
Choose option: 14

=== TESTING ENHANCED FPS ===
Testing Farthest Point Sampling with LibIGL heat geodesics
Testing FPS with 5 samples...
Error: Mesh not converted to LibIGL format. Call convertToLibIGL() first.
Warning: Heat geodesics not available, falling back to original FPS
  FPS generated 5 samples
  Sample vertices: 2594 3073 3706 3 1682
Testing FPS with 10 samples...
Error: Mesh not converted to LibIGL format. Call convertToLibIGL() first.
Warning: Heat geodesics not available, falling back to original FPS
  FPS generated 10 samples
  Sample vertices: 2594 3073 3706 3 1682 ...
Testing FPS with 20 samples...
Error: Mesh not converted to LibIGL format. Call convertToLibIGL() first.
Warning: Heat geodesics not available, falling back to original FPS
  FPS generated 20 samples
  Sample vertices: 2594 3073 3706 3 1682 ...
Testing FPS with 30 samples...
Error: Mesh not converted to LibIGL format. Call convertToLibIGL() first.
Warning: Heat geodesics not available, falling back to original FPS

OKAY IT IS NOT IMPLEMENTED.


12. Interactive Rigidity Threshold Testing
--- LIBIGL ENHANCED FEATURES ---
13. Test Heat Geodesics (LibIGL)
14. Enhanced FPS with Heat Geodesics
15. Compare Geodesic Methods (Dijkstra vs Heat)
16. Enhanced R&D Descriptors (LibIGL)
17. Mesh Analysis (Normals, Curvature, Areas)
0. Exit
======================================
Choose option: 17

=== TESTING MESH ANALYSIS ===
Running comprehensive mesh analysis with LibIGL
Mesh statistics:
  Vertices: 3714
  Faces: 7424
LibIGL mesh data:
  Vertex matrix: 3714 x 3
  Face matrix: 7424 x 3
Testing Laplacian computation...
Laplacian computation: SUCCESS
Analyzing mesh properties...
Mesh analysis completed!
Comprehensive mesh analysis completed!


OKAY LETS LOAD SMALLER MESH


0. Exit
======================================
Choose option: 1

=== LOAD MESH ===
Available meshes:
  - 50.off
  - 249.off
  - 348.off
  - man0.off
  - Armadillo.off
Enter mesh filename: man0.off
No visualizations to clear.
Loading mesh: models/man0.off...
Mesh normalized: Scale=0.014370, Center=(-3.890551,-14.728899,69.431908)
Mesh loaded successfully!
  Vertices: 502
  Triangles: 1000
  Edges: 1499

Initializing LibIGL for robust computations...
LibIGL initialization successful!
Precomputing heat geodesics for enhanced performance...
Error: Mesh not converted to LibIGL format. Call convertToLibIGL() first.
Warning: Heat geodesics precomputation failed. Using Dijkstra fallback.
Mesh ready for pairwise harmonics analysis with LibIGL support!

Parameters:
  - FPS samples: 5
  - Isocurves per field (K): 50
  - Rigidity threshold: 0.9

--- STAGE 1: FARTHEST POINT SAMPLING ---
Generated 5 FPS samples

--- STAGE 2: SKELETAL SEGMENT GENERATION ---
Computing pairwise harmonic fields for 10 point pairs...
  Pair 1/10 (vertices 218, 345)
  Pair 2/10 (vertices 218, 498)
  Pair 3/10 (vertices 218, 136)
  Pair 4/10 (vertices 218, 387)
  Pair 5/10 (vertices 345, 498)
  Pair 10/10 (vertices 136, 387)
Generated 10 raw skeletal segments

--- STAGE 3: RIGIDITY ANALYSIS ---
Computing rigidity for 10 segments...
Selected 4 segments for partial skeleton

--- STAGE 4: MESH SEGMENTATION ---
Created 3 mesh components

--- STAGE 5: SKELETON COMPLETION ---
Completed skeleton with 3 nodes

=== SEGMENTATION COMPLETE ===

Visualization complete:
  - 4 skeletal segments (colored by rigidity)
  - 3 mesh components
  - 3 skeleton nodes (magenta points)

=== SEGMENTATION SUMMARY ===
Ô£ô Successfully segmented mesh into 3 components
Ô£ô Generated partial skeleton with 4 segments
Ô£ô Created complete skeleton with 3 nodes
Ô£ô Used 5 FPS sample points

======================================
PAIRWISE HARMONICS SEGMENTATION SYSTEM
======================================
1. Load Mesh
2. Test Cotangent Laplacian
3. Test Farthest Point Sampling
4. Test Pairwise Harmonics
5. Test Iso-curve Extraction
6. Test Rigidity Analysis
7. Perform Full Segmentation
8. Clear Visualization
--- ENHANCED ANALYSIS ---
9. Improved Iso-curve Extraction (Dense Bracelets)
10. Enhanced Rigidity Analysis (All Nodes)
11. Visualize All Rigidity Points (Color-coded)
12. Interactive Rigidity Threshold Testing
--- LIBIGL ENHANCED FEATURES ---
13. Test Heat Geodesics (LibIGL)
14. Enhanced FPS with Heat Geodesics
15. Compare Geodesic Methods (Dijkstra vs Heat)
16. Enhanced R&D Descriptors (LibIGL)
17. Mesh Analysis (Normals, Curvature, Areas)
0. Exit
======================================
Choose option: 7
Cleared 5 visualization elements, preserving mesh.

=== SEGMENTATION PARAMETERS ===
Use default parameters? (y/n): n
Current FPS samples (5): 25
Current isocurves per field (50): 50
Current rigidity threshold (0.9): 0.95

Starting pairwise harmonics segmentation...

=== PAIRWISE HARMONICS SEGMENTATION ===
Implementation based on 'Pairwise Harmonics for Shape Analysis'
by Zheng et al., IEEE TVCG 2013

Parameters:
  - FPS samples: 25
  - Isocurves per field (K): 50
  - Rigidity threshold: 0.95

--- STAGE 1: FARTHEST POINT SAMPLING ---
Generated 25 FPS samples

--- STAGE 2: SKELETAL SEGMENT GENERATION ---
Computing pairwise harmonic fields for 300 point pairs...
  Pair 1/300 (vertices 218, 345)
  Pair 2/300 (vertices 218, 498)
  Pair 3/300 (vertices 218, 136)
  Pair 4/300 (vertices 218, 387)
  Pair 5/300 (vertices 218, 92)
  Pair 10/300 (vertices 218, 158)
  Pair 20/300 (vertices 218, 62)
  Pair 30/300 (vertices 345, 38)
  Pair 40/300 (vertices 345, 315)
  Pair 50/300 (vertices 498, 92)
  Pair 60/300 (vertices 498, 129)
  Pair 70/300 (vertices 136, 387)
  Pair 80/300 (vertices 136, 35)
  Pair 90/300 (vertices 136, 236)
  Pair 100/300 (vertices 387, 35)
  Pair 110/300 (vertices 387, 236)
  Pair 120/300 (vertices 92, 129)
  Pair 130/300 (vertices 262, 38)
  Pair 140/300 (vertices 262, 315)
  Pair 150/300 (vertices 38, 158)
  Pair 160/300 (vertices 38, 62)
  Pair 170/300 (vertices 465, 35)
  Pair 180/300 (vertices 465, 236)
  Pair 190/300 (vertices 65, 29)
  Pair 200/300 (vertices 158, 129)
  Pair 210/300 (vertices 260, 381)
  Pair 220/300 (vertices 260, 204)
  Pair 230/300 (vertices 381, 62)
  Pair 240/300 (vertices 21, 29)
  Pair 250/300 (vertices 35, 29)
  Pair 260/300 (vertices 129, 62)
  Pair 270/300 (vertices 296, 204)
  Pair 280/300 (vertices 444, 29)
  Pair 290/300 (vertices 29, 236)
  Pair 300/300 (vertices 227, 236)
Generated 300 raw skeletal segments

--- STAGE 3: RIGIDITY ANALYSIS ---
Computing rigidity for 300 segments...
Selected 8 segments for partial skeleton

--- STAGE 4: MESH SEGMENTATION ---
Created 3 mesh components

--- STAGE 5: SKELETON COMPLETION ---
Completed skeleton with 3 nodes

=== SEGMENTATION COMPLETE ===

Visualization complete:
  - 8 skeletal segments (colored by rigidity)
  - 3 mesh components
  - 3 skeleton nodes (magenta points)

=== SEGMENTATION SUMMARY ===
Ô£ô Successfully segmented mesh into 3 components
Ô£ô Generated partial skeleton with 8 segments
Ô£ô Created complete skeleton with 3 nodes
Ô£ô Used 25 FPS sample points

THIS CREATED BETTER SKELETON WITH LOWER SAMPLES NEVERTHELESS SOME SKELETON PARTS ARE OUTSIDE OF THE MESH. THOSE PARTS COULD BE CLUSTERED TOGETHER AND PCA APPLIED TO FIND MOST PROPER SKELETON PART TO PLACE THERE MAYBE. JUST A THOUGHT. PROPERLY FINDS SKELETAL SEGMENTS!

======================================
PAIRWISE HARMONICS SEGMENTATION SYSTEM
======================================
1. Load Mesh
2. Test Cotangent Laplacian
3. Test Farthest Point Sampling
4. Test Pairwise Harmonics
5. Test Iso-curve Extraction
6. Test Rigidity Analysis
7. Perform Full Segmentation
8. Clear Visualization
--- ENHANCED ANALYSIS ---
9. Improved Iso-curve Extraction (Dense Bracelets)
10. Enhanced Rigidity Analysis (All Nodes)
11. Visualize All Rigidity Points (Color-coded)
12. Interactive Rigidity Threshold Testing
--- LIBIGL ENHANCED FEATURES ---
13. Test Heat Geodesics (LibIGL)
14. Enhanced FPS with Heat Geodesics
15. Compare Geodesic Methods (Dijkstra vs Heat)
16. Enhanced R&D Descriptors (LibIGL)
17. Mesh Analysis (Normals, Curvature, Areas)
0. Exit
======================================
Choose option: 11

=== VISUALIZING ALL RIGIDITY POINTS ===
Showing continuous rigidity distribution: Green = Low Rigidity (Junctions), Red = High Rigidity (Limbs)
Using downsampling factor: 10 (showing every 10th point)
Rigidity range: [0.422222, 0.996293]
Color mapping: Green (Ôëñ0.594444) -> Yellow -> Red (ÔëÑ0.938886)
Visualized 384 out of 3833 total rigidity points
Creating color legend...

Visualization complete!
Downsampled node distribution (showing every 10th point):
  Low rigidity (Green, potential junctions): 74 nodes shown
  Medium rigidity (Yellow): 172 nodes shown
  High rigidity (Red, limb centers): 138 nodes shown
  Total visualized: 384 out of 3833 total points

Legend spheres placed on the right side of the model

AGAIN NODES ARE ON THE SURFACE. MAYBE THE ALGORTIHM USES NODES TO CALCULATE INNER POINTS TO USE AS SKELETON. ACTUALLY THIS MAKES SENSE!! SO IT WORKS AS INTENDED. JUST COLOR CODE IS WRONG. GREEN SHOULD BE MOST RIGID POINTS. RED SHOULD BE LESS RIGID POINTS.

=== IMPROVED ISO-CURVE EXTRACTION ===
Extracting robust isocurves using triangle marching and graph stitching
Computing harmonic field between FPS vertices 218 and 345...
Extracting 25 isocurves using robust triangle marching...
  Extracting isocurves at value 0.04 using robust triangle marching...
    Generated 5 triangle segments
    Unified to 5 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 5 points
  Isocurve 1 (value=0.04): 5 points in continuous polyline
  Extracting isocurves at value 0.08 using robust triangle marching...
    Generated 10 triangle segments
    Unified to 10 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 10 points
  Isocurve 2 (value=0.08): 10 points in continuous polyline
  Extracting isocurves at value 0.12 using robust triangle marching...
    Generated 13 triangle segments
    Unified to 13 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 13 points
  Isocurve 3 (value=0.12): 13 points in continuous polyline
  Extracting isocurves at value 0.16 using robust triangle marching...
    Generated 13 triangle segments
    Unified to 13 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 13 points
  Isocurve 4 (value=0.16): 13 points in continuous polyline
  Extracting isocurves at value 0.2 using robust triangle marching...
    Generated 13 triangle segments
    Unified to 13 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 13 points
  Isocurve 5 (value=0.2): 13 points in continuous polyline
  Extracting isocurves at value 0.24 using robust triangle marching...
    Generated 18 triangle segments
    Unified to 18 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 18 points
  Isocurve 6 (value=0.24): 18 points in continuous polyline
  Extracting isocurves at value 0.28 using robust triangle marching...
    Generated 12 triangle segments
    Unified to 12 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 12 points
  Isocurve 7 (value=0.28): 12 points in continuous polyline
  Extracting isocurves at value 0.32 using robust triangle marching...
    Generated 13 triangle segments
    Unified to 13 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 13 points
  Isocurve 8 (value=0.32): 13 points in continuous polyline
  Extracting isocurves at value 0.36 using robust triangle marching...
    Generated 13 triangle segments
    Unified to 13 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 13 points
  Isocurve 9 (value=0.36): 13 points in continuous polyline
  Extracting isocurves at value 0.4 using robust triangle marching...
    Generated 15 triangle segments
    Unified to 15 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 15 points
  Isocurve 10 (value=0.4): 15 points in continuous polyline
  Extracting isocurves at value 0.44 using robust triangle marching...
    Generated 15 triangle segments
    Unified to 15 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 15 points
  Isocurve 11 (value=0.44): 15 points in continuous polyline
  Extracting isocurves at value 0.48 using robust triangle marching...
    Generated 31 triangle segments
    Unified to 31 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 31 points
  Isocurve 12 (value=0.48): 31 points in continuous polyline
  Extracting isocurves at value 0.52 using robust triangle marching...
    Generated 30 triangle segments
    Unified to 30 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 30 points
  Isocurve 13 (value=0.52): 30 points in continuous polyline
  Extracting isocurves at value 0.56 using robust triangle marching...
    Generated 42 triangle segments
    Unified to 42 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 42 points
  Isocurve 14 (value=0.56): 42 points in continuous polyline
  Extracting isocurves at value 0.6 using robust triangle marching...
    Generated 32 triangle segments
    Unified to 32 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 32 points
  Isocurve 15 (value=0.6): 32 points in continuous polyline
  Extracting isocurves at value 0.64 using robust triangle marching...
    Generated 29 triangle segments
    Unified to 29 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 29 points
  Isocurve 16 (value=0.64): 29 points in continuous polyline
  Extracting isocurves at value 0.68 using robust triangle marching...
    Generated 18 triangle segments
    Unified to 18 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 18 points
  Isocurve 17 (value=0.68): 18 points in continuous polyline
  Extracting isocurves at value 0.72 using robust triangle marching...
    Generated 16 triangle segments
    Unified to 16 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 16 points
  Isocurve 18 (value=0.72): 16 points in continuous polyline
  Extracting isocurves at value 0.76 using robust triangle marching...
    Generated 21 triangle segments
    Unified to 21 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 21 points
  Isocurve 19 (value=0.76): 21 points in continuous polyline
  Extracting isocurves at value 0.8 using robust triangle marching...
    Generated 23 triangle segments
    Unified to 23 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 23 points
  Isocurve 20 (value=0.8): 23 points in continuous polyline
  Extracting isocurves at value 0.84 using robust triangle marching...
    Generated 23 triangle segments
    Unified to 23 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 23 points
  Isocurve 21 (value=0.84): 23 points in continuous polyline
  Extracting isocurves at value 0.88 using robust triangle marching...
    Generated 17 triangle segments
    Unified to 17 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 17 points
  Isocurve 22 (value=0.88): 17 points in continuous polyline
  Extracting isocurves at value 0.92 using robust triangle marching...
    Generated 15 triangle segments
    Unified to 15 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 15 points
  Isocurve 23 (value=0.92): 15 points in continuous polyline
  Extracting isocurves at value 0.96 using robust triangle marching...
    Generated 13 triangle segments
    Unified to 13 unique endpoints
    Extracted 1 continuous isocurves
      Isocurve 0: 13 points
  Isocurve 24 (value=0.96): 13 points in continuous polyline

Computing rigidity scores for isocurve centroids...
  Centroid 0: rigidity = 0.992858
  Centroid 1: rigidity = 0.992858
  Centroid 2: rigidity = 0.992858
  Centroid 3: rigidity = 0.992858
  Centroid 4: rigidity = 0.989789
  Centroid 5: rigidity = 0.983481
  Centroid 6: rigidity = 0.977482
  Centroid 7: rigidity = 0.977779
  Centroid 8: rigidity = 0.976463
  Centroid 9: rigidity = 0.788843
  Centroid 10: rigidity = 0.732144
  Centroid 11: rigidity = 0.874357
  Centroid 12: rigidity = 0.924629
  Centroid 13: rigidity = 0.937458
  Centroid 14: rigidity = 0.94152
  Centroid 15: rigidity = 0.942664
  Centroid 16: rigidity = 0.880987
  Centroid 17: rigidity = 0.86254
  Centroid 18: rigidity = 0.940747
  Centroid 19: rigidity = 0.991952
  Centroid 20: rigidity = 0.923284
  Centroid 21: rigidity = 0.898434
  Centroid 22: rigidity = 0.850606
  Centroid 23: rigidity = 0.787338
Visualizing rigidity points on isocurve centroids...

SUCCESS: Generated skeletal segment with 24 nodes
Yellow line shows the skeletal segment connecting isocurve centroids
Colored spheres show rigidity points (green=rigid, red=non-rigid)
Rigidity range: [0.732144, 0.992858]
Total 24 robust isocurves extracted
Robust iso-curve extraction complete - showing continuous polylines!

IMPROVED ISO-CURVE EXTRACTION WORKS WELL BUT GENERATES SINGLE CONTINUOUS POLYLINE. FOR 4 LIMBS IN THE SKELETON IT SHOULD GENERATE 4 POLYLINES + MAIN BODY POLYLINE.
