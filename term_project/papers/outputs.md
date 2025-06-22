=== LOAD MESH ===
Available meshes:
  - 50.off
  - 249.off
  - 348.off
  - man0.off
  - Armadillo.off
Enter mesh filename: Armadillo.off
No visualizations to clear.
Loading mesh: models/Armadillo.off...
Mesh normalized: Scale=0.013214, Center=(-0.030325,21.510603,0.102776)
Mesh loaded successfully!
  Vertices: 4326
  Triangles: 8648
  Edges: 12972

Converting mesh to LibIGL format...
Successfully computed face areas using LibIGL
Successfully computed face and vertex normals using LibIGL
Successfully computed face barycenters using LibIGL
Enhanced LibIGL conversion completed:
  Vertices: 4326, Faces: 8648
  Average edge length: 0.0460617
Initializing LibIGL for robust computations...
Using mesh's existing LibIGL data...
LibIGL initialization successful!
Precomputing heat geodesics for mesh with 4326 vertices and 8648 faces...
Successfully precomputed heat geodesics data
Heat geodesics precomputation successful!
  Fast geodesic distance computation is now available
Mesh ready for pairwise harmonics analysis with LibIGL support!
Mesh ready for pairwise harmonics analysis with LibIGL support!

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
Choose option: 2

=== TESTING COTANGENT LAPLACIAN ===
Choose implementation:
1. libigl-based (recommended - more robust)
2. Custom implementation (original)
Enter choice (1 or 2): 1
Computing cotangent Laplacian using libigl...
SUCCESS: Cotangent Laplacian computed successfully!
  Implementation: libigl
  Computation time: 0.092 seconds
  Matrix size: 4326x4326
  Properties:
    - Intrinsic (uses cotangent weights)
    - libigl robust implementation
    - Includes mass matrix computation
    - Symmetric and positive semi-definite
    - Ready for harmonic field computation

  ======================================
Choose option: 3

=== TESTING FARTHEST POINT SAMPLING ===
Enter number of FPS samples (recommended: 25-40 for human models, 15-25 for simple shapes): 20
Computing FPS with 20 points...
Algorithm: 1) Pick arbitrary point -> 2) Find farthest -> 3) Discard arbitrary -> 4) Continue FPS
SUCCESS: FPS computed successfully!
  Computation time: 32.236 seconds
  Sample points: 832 1 210 2190 3026 143 1196 2977 3801 3570 2123 2376 437 887 2272 659 830 1419 165 2325
Only mesh present, nothing to clear.
FPS points visualized (first point is red, others blue).

======================================
Choose option: 4

=== TESTING PAIRWISE HARMONICS ===
Choose implementation:
1. libigl-based (recommended - more robust)
2. Custom implementation (original)
Enter choice (1 or 2): 1
Enter first vertex index (p, boundary value 0): 1
Enter second vertex index (q, boundary value 1): 25
Computing harmonic field between vertices 1 and 25...
Using libigl implementation...
SUCCESS: Pairwise harmonic computed successfully!
  Implementation: libigl
  Computation time: 0.286 seconds
  Field range: [0, 1]
Only mesh present, nothing to clear.
Visualizing scalar field: Harmonic Field
Field range: [0, 1]

Computing shape descriptors...
R descriptor (isocurve lengths): 0.0861285 0.124375 0.14253 0.160896 0.178003 0.197967 0.430051 0.521219 0.239491 0.221666
Harmonic field visualized (blue = value 0, red = value 1).


======================================
Choose option: 5

=== TESTING ISO-CURVE EXTRACTION ===
Computing harmonic field between FPS vertices 832 and 1...
Extracting iso-curves at different levels...
Cleared 4326 visualization elements, preserving mesh.
  Iso-value 0.2: 46 segments, total length = 0.801963
  Iso-value 0.4: 98 segments, total length = 1.85018
  Iso-value 0.6: 43 segments, total length = 0.825326
  Iso-value 0.8: 13 segments, total length = 0.191536
SUCCESS: Iso-curve extraction complete!
  Generated 4 skeletal nodes
Iso-curves visualized with color-coded levels.


Choose option: 7
Cleared 4 visualization elements, preserving mesh.

=== SEGMENTATION PARAMETERS ===
Use default parameters? (y/n): n
Current FPS samples (25): 6
Current isocurves per field (50): 50
Current rigidity threshold (0.9): 0.9

Starting pairwise harmonics segmentation...

=== PAIRWISE HARMONICS SEGMENTATION ===
Implementation based on 'Pairwise Harmonics for Shape Analysis'
by Zheng et al., IEEE TVCG 2013

Parameters:
  - FPS samples: 6
  - Isocurves per field (K): 50
  - Rigidity threshold: 0.9

--- STAGE 1: FARTHEST POINT SAMPLING ---
Generated 6 FPS samples

--- STAGE 2: SKELETAL SEGMENT GENERATION ---
Computing pairwise harmonic fields for 15 point pairs...
  Pair 1/15 (vertices 832, 1)
  Pair 2/15 (vertices 832, 210)
  Pair 3/15 (vertices 832, 2190)
  Pair 4/15 (vertices 832, 3026)
  Pair 5/15 (vertices 832, 143)
  Pair 13/15 (vertices 2190, 3026)
  Pair 14/15 (vertices 2190, 143)
  Pair 15/15 (vertices 3026, 143)
Generated 15 raw skeletal segments

--- STAGE 3: RIGIDITY ANALYSIS ---
Computing rigidity for 15 segments...
Selected 5 segments for partial skeleton

--- STAGE 4: MESH SEGMENTATION ---
Created 4 mesh components

--- STAGE 5: SKELETON COMPLETION ---
Completed skeleton with 4 nodes

=== SEGMENTATION COMPLETE ===
Adding mesh component visualization...

Visualization complete:
  - 5 skeletal segments (colored by rigidity)
  - 4 mesh components
  - 4 skeleton nodes (magenta points)

=== SEGMENTATION SUMMARY ===
-> Successfully segmented mesh into 4 components
-> Generated partial skeleton with 5 segments
-> Created complete skeleton with 4 nodes
-> Used 6 FPS sample points

IN THIS SECTION WE NEED TO SHOW SEGMENTATIONS MORE VISUALLY. FOR EXAMPLE FOR 5 SKELETAL SEGMENTS WE SUGGEST 5 DIFFERENT COLORED CLUSTERS. SIMPLE K MEANS CLUSTERING USING DISTANCE TO THE SKELETAL SEGMENT TO THE VERTEX. EACH SKELETAL SEGMENT IS A CLUSTER CENTER. THEN WE CAN SHOW THE MESH WITH 5 DIFFERENT COLORS FOR EACH SKELETAL SEGMENT.

FOR IMPROVED ISO-CURVE EXTRACTION

Computing rigidity scores for isocurve centroids...
  Centroid 0: rigidity = 0.848628
  Centroid 1: rigidity = 0.848628
  Centroid 2: rigidity = 0.848628
  Centroid 3: rigidity = 0.848628
  Centroid 4: rigidity = 0.913058
  Centroid 5: rigidity = 0.958368
  Centroid 6: rigidity = 0.904619
  Centroid 7: rigidity = 0.939351
  Centroid 8: rigidity = 0.946904
  Centroid 9: rigidity = 0.923767
  Centroid 10: rigidity = 0.900022
  Centroid 11: rigidity = 0.87686
  Centroid 12: rigidity = 0.856523
  Centroid 13: rigidity = 0.947327
  Centroid 14: rigidity = 0.936835
  Centroid 15: rigidity = 0.980043
  Centroid 16: rigidity = 0.983334
  Centroid 17: rigidity = 0.976875
  Centroid 18: rigidity = 0.982561
  Centroid 19: rigidity = 0.970835
  Centroid 20: rigidity = 0.971643
  Centroid 21: rigidity = 0.980093
  Centroid 22: rigidity = 0.996268
  Centroid 23: rigidity = 0.992227
Visualizing rigidity points on isocurve centroids...

SUCCESS: Generated skeletal segment with 24 nodes
Yellow line shows the skeletal segment connecting isocurve centroids
Colored spheres show rigidity points (green=rigid, red=non-rigid)
Rigidity range: [0.848628, 0.996268]
Total 24 robust isocurves extracted
Robust iso-curve extraction complete - showing continuous polylines!

LETS INVESTIGATE IF WE CAN USE INTERACTIVE WHY DO WE PICK 2 POINTS? "Extracting robust isocurves using triangle marching and graph stitching
Computing harmonic field between FPS vertices 832 and 1..." CAN WE INTERACTIVELY INCREASE POINT NUMBERS? 1 and 832 seems to be 2 farthest points.


ENCHANCED RIGIDITY ANALYSIS SHOWS

Rigidity Statistics:
  Total nodes analyzed: 37366
  Rigidity range: [0.394136, 0.999857]
  Average rigidity: 0.755193
  Standard deviation: 0.125968

Rigidity Distribution:
    >= 0.5: 36838 nodes (98.587%)
    >= 0.6: 32593 nodes (87.2264%)
    >= 0.7: 24206 nodes (64.7808%)
    >= 0.8: 14283 nodes (38.2246%)
    >= 0.9: 5481 nodes (14.6684%)
    >= 1: 0 nodes (0%)

Enhanced rigidity analysis complete!

================================
Choose option: 11

=== VISUALIZING ALL RIGIDITY POINTS ===
Showing continuous rigidity distribution: GREEN = High Rigidity (Limb Centers), RED = Low Rigidity (Junctions)
Using downsampling factor: 10 (showing every 10th point)
Rigidity range: [0.394136, 0.999857]
Color mapping: GREEN (ÔëÑ0.939285) -> Yellow -> RED (Ôëñ0.575853)
Visualized 3737 out of 37366 total rigidity points
Creating color legend...

Visualization complete!
Downsampled node distribution (showing every 10th point):
  Low rigidity (Red, potential junctions): 441 nodes shown
  Medium rigidity (Yellow): 1846 nodes shown
  High rigidity (Green, skeleton centers): 1450 nodes shown
  Total visualized: 3737 out of 37366 total points

Legend spheres placed on the right side of the model

Choose option: 12

=== INTERACTIVE RIGIDITY THRESHOLD TESTING ===
This will help you find the optimal threshold for detecting junction points
Paper recommendation: Use threshold 0.9 (range 0.85-0.95 works well)

Current rigidity range: [0.394136, 0.999857]
Total skeletal nodes: 37366

--- THRESHOLD ANALYSIS ---
Enter a rigidity threshold to test (0.4-1.0, or 'q' to quit): 0.95

Results for threshold = 0.95:
  Rigid nodes (skeletal): 2446 (6.54606%)
  Non-rigid nodes (junctions): 34920 (93.4539%)

Visualization Guide:
  GREEN spheres (large): Non-rigid nodes = Junction candidates
  BLUE-to-RED spheres (small): Rigid nodes = Skeletal centers
  -> For human models, you want ~4-8 green junction points
  -> (shoulders, hips, neck, wrists, ankles, etc.)

--- THRESHOLD ANALYSIS ---
Enter a rigidity threshold to test (0.4-1.0, or 'q' to quit):

THIS SHOWS HIGH RIGIDITY AS RED LOW RIGIDITY AS GREEN. WE NEED TO REVERSE THIS.

======================================
Choose option: 13

=== TESTING HEAT GEODESICS ===
This test compares LibIGL heat geodesics with Dijkstra distances
Testing heat geodesics from vertex 0...
Computed heat geodesic distances from vertex 0 (range: 0 to 2.826)
  Heat geodesic computation successful (size: 4326)
  Max distance: 2.826
  Min distance: 0
Testing heat geodesics from vertex 1081...
Computed heat geodesic distances from vertex 1081 (range: 0 to 2.23682)
  Heat geodesic computation successful (size: 4326)
  Max distance: 2.23682
  Min distance: 0
Testing heat geodesics from vertex 2163...
Computed heat geodesic distances from vertex 2163 (range: 0 to 2.59967)
  Heat geodesic computation successful (size: 4326)
  Max distance: 2.59967
  Min distance: 0
Testing heat geodesics from vertex 3244...
Computed heat geodesic distances from vertex 3244 (range: 0 to 2.02842)
  Heat geodesic computation successful (size: 4326)
  Max distance: 2.02842
  Min distance: 0
Heat geodesics test completed!

MAY NEED VISUALIZATION FOR THIS.

======================================
Choose option: 14

=== TESTING ENHANCED FPS ===
Testing Farthest Point Sampling with LibIGL heat geodesics
Testing FPS with 5 samples...
Computed heat geodesic distances from vertex 0 (range: 0 to 2.826)
Computed heat geodesic distances from vertex 2835 (range: 0 to 2.81486)
Computed heat geodesic distances from vertex 180 (range: 0 to 2.88516)
Computed heat geodesic distances from vertex 2494 (range: 0 to 2.84989)
Computed heat geodesic distances from vertex 3934 (range: 0 to 2.24879)
LibIGL-enhanced FPS completed: 5 samples generated
  FPS generated 5 samples
  Sample vertices: 0 2835 180 2494 3934
Testing FPS with 10 samples...
Computed heat geodesic distances from vertex 0 (range: 0 to 2.826)
Computed heat geodesic distances from vertex 2835 (range: 0 to 2.81486)
Computed heat geodesic distances from vertex 180 (range: 0 to 2.88516)
Computed heat geodesic distances from vertex 2494 (range: 0 to 2.84989)
Computed heat geodesic distances from vertex 3934 (range: 0 to 2.24879)
Computed heat geodesic distances from vertex 94 (range: 0 to 2.3698)
Computed heat geodesic distances from vertex 467 (range: 0 to 2.12704)
Computed heat geodesic distances from vertex 3745 (range: 0 to 1.8917)
Computed heat geodesic distances from vertex 2647 (range: 0 to 2.06802)
Computed heat geodesic distances from vertex 3942 (range: 0 to 2.15119)
LibIGL-enhanced FPS completed: 10 samples generated
  FPS generated 10 samples
  Sample vertices: 0 2835 180 2494 3934 ...
Testing FPS with 20 samples...
Computed heat geodesic distances from vertex 0 (range: 0 to 2.826)
Computed heat geodesic distances from vertex 2835 (range: 0 to 2.81486)
Computed heat geodesic distances from vertex 180 (range: 0 to 2.88516)
Computed heat geodesic distances from vertex 2494 (range: 0 to 2.84989)
Computed heat geodesic distances from vertex 3934 (range: 0 to 2.24879)
Computed heat geodesic distances from vertex 94 (range: 0 to 2.3698)
Computed heat geodesic distances from vertex 467 (range: 0 to 2.12704)
Computed heat geodesic distances from vertex 3745 (range: 0 to 1.8917)
Computed heat geodesic distances from vertex 2647 (range: 0 to 2.06802)
Computed heat geodesic distances from vertex 3942 (range: 0 to 2.15119)
Computed heat geodesic distances from vertex 3000 (range: 0 to 2.17902)
Computed heat geodesic distances from vertex 770 (range: 0 to 2.58803)
Computed heat geodesic distances from vertex 930 (range: 0 to 2.58079)
Computed heat geodesic distances from vertex 973 (range: 0 to 2.33262)
Computed heat geodesic distances from vertex 1836 (range: 0 to 1.70166)
Computed heat geodesic distances from vertex 2782 (range: 0 to 1.67991)
Computed heat geodesic distances from vertex 2445 (range: 0 to 2.43738)
Computed heat geodesic distances from vertex 4142 (range: 0 to 2.4761)
Computed heat geodesic distances from vertex 2943 (range: 0 to 2.33279)
Computed heat geodesic distances from vertex 2121 (range: 0 to 1.77513)
LibIGL-enhanced FPS completed: 20 samples generated
  FPS generated 20 samples
  Sample vertices: 0 2835 180 2494 3934 ...
Testing FPS with 30 samples...
Computed heat geodesic distances from vertex 0 (range: 0 to 2.826)
Computed heat geodesic distances from vertex 2835 (range: 0 to 2.81486)
Computed heat geodesic distances from vertex 180 (range: 0 to 2.88516)
Computed heat geodesic distances from vertex 2494 (range: 0 to 2.84989)
Computed heat geodesic distances from vertex 3934 (range: 0 to 2.24879)
Computed heat geodesic distances from vertex 94 (range: 0 to 2.3698)
Computed heat geodesic distances from vertex 467 (range: 0 to 2.12704)
Computed heat geodesic distances from vertex 3745 (range: 0 to 1.8917)
Computed heat geodesic distances from vertex 2647 (range: 0 to 2.06802)
Computed heat geodesic distances from vertex 3942 (range: 0 to 2.15119)
Computed heat geodesic distances from vertex 3000 (range: 0 to 2.17902)
Computed heat geodesic distances from vertex 770 (range: 0 to 2.58803)
Computed heat geodesic distances from vertex 930 (range: 0 to 2.58079)
Computed heat geodesic distances from vertex 973 (range: 0 to 2.33262)
Computed heat geodesic distances from vertex 1836 (range: 0 to 1.70166)
Computed heat geodesic distances from vertex 2782 (range: 0 to 1.67991)
Computed heat geodesic distances from vertex 2445 (range: 0 to 2.43738)
Computed heat geodesic distances from vertex 4142 (range: 0 to 2.4761)
Computed heat geodesic distances from vertex 2943 (range: 0 to 2.33279)
Computed heat geodesic distances from vertex 2121 (range: 0 to 1.77513)
Computed heat geodesic distances from vertex 1991 (range: 0 to 2.60684)
Computed heat geodesic distances from vertex 844 (range: 0 to 2.37593)
Computed heat geodesic distances from vertex 3743 (range: 0 to 1.97164)
Computed heat geodesic distances from vertex 905 (range: 0 to 2.05049)
Computed heat geodesic distances from vertex 3528 (range: 0 to 2.26488)
Computed heat geodesic distances from vertex 4251 (range: 0 to 1.70325)
Computed heat geodesic distances from vertex 4191 (range: 0 to 2.06222)
Computed heat geodesic distances from vertex 4106 (range: 0 to 2.27672)
Computed heat geodesic distances from vertex 1412 (range: 0 to 2.12841)
Computed heat geodesic distances from vertex 1332 (range: 0 to 2.37817)
LibIGL-enhanced FPS completed: 30 samples generated
  FPS generated 30 samples
  Sample vertices: 0 2835 180 2494 3934 ...
Enhanced FPS test completed!

DEFINITELY NEED VISUALIZATION FOR THIS. SIMILAR TO FARTHEST POINT SAMPLING.

======================================
Choose option: 15

=== COMPARING GEODESIC METHODS ===
Comparing LibIGL heat geodesics with other methods
Computing heat geodesics from vertex 0...
Computed heat geodesic distances from vertex 0 (range: 0 to 2.826)
Heat geodesics computed in 37 ms
Result statistics:
  Size: 4326
  Max distance: 2.826
  Min distance: 0
  Mean distance: 1.65872
Geodesic methods comparison completed!


I DON'T UNDERSTAND THIS. WHAT DOES THIS COMPARE? NEED SOME CLARIFICATION.

Choose option: 16

=== TESTING ENHANCED DESCRIPTORS ===
Testing LibIGL-based R and D descriptors
Computing harmonic field between vertices 0 and 2163...
Harmonic field computed successfully (size: 4326)
Computing R descriptor...
R descriptor size: 50
Computing D descriptor...
D descriptor size: 50
Enhanced descriptors test completed!

NEED SOME CLARIFICATION FOR THIS ASWELL. WHERE DOES THIS COMPUTATIONS GO WHERE DO WE USE THEM?

Choose option: 17

=== TESTING MESH ANALYSIS ===
Running comprehensive mesh analysis with LibIGL
Mesh statistics:
  Vertices: 4326
  Faces: 8648
LibIGL mesh data:
  Vertex matrix: 4326 x 3
  Face matrix: 8648 x 3
Testing Laplacian computation...
Laplacian computation: SUCCESS
Analyzing mesh properties...
Mesh analysis completed!
Comprehensive mesh analysis completed!

OKAY THIS DOES SOME VERTEX FACE EXTRACTION I SEE.


LAST NOTES: LETS ADD OPTION FOR HEAT GEODESICS IN THE FARTHEST POINT SAMPLING. LETS REMOVE CUSTOM IMPLEMENTATION FOR COTANGENT LAPLACIAN AND PAIRWISE HARMONICS.

LETS GENERATE MORE ISOCURVES FOR TEST ISOCURVE EXTRACTION. LETS INCREASE IT 4 TO 8.