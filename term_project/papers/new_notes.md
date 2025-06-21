okay I have tested further. I see that we create skeletal segments very good. I don't see parallel skeleton parts in limbs. But in main body as expected I see parallel skeletons. lets check this #file:cpp_segment_code_search.md again and #file:Pairwise_Harmonics_for_Shape_Analysis.txt . in the paper I see that it has generally higher rigidity values (above 0.95 in the paper) comparing to main body parts. so researchers in some sense treat main body as cluster. and put simpler skeleton in the middle. I don't know how exactly. I see 4 skeleton parts in man model for each arm and leg then they connect the skeleton in the middle body part. also taking consideration head aswell. which created 6 segments. we have 5 segments for armadillo. which has similar shape to man. "Choose option: 7
Only mesh present, nothing to clear.

=== SEGMENTATION PARAMETERS ===
Use default parameters? (y/n): n
Current FPS samples (6): 6
Current isocurves per field (20): 100
Current rigidity threshold (0.9): 0.9

Starting pairwise harmonics segmentation...

=== PAIRWISE HARMONICS SEGMENTATION ===
Implementation based on 'Pairwise Harmonics for Shape Analysis'
by Zheng et al., IEEE TVCG 2013

Parameters:
  - FPS samples: 6
  - Isocurves per field (K): 100
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
  Pair 10/15 (vertices 210, 2190)
Generated 15 raw skeletal segments

--- STAGE 3: RIGIDITY ANALYSIS ---
Computing rigidity for 15 segments...
Selected 5 segments for partial skeleton

--- STAGE 4: MESH SEGMENTATION ---
Created 3 mesh components

--- STAGE 5: SKELETON COMPLETION ---
Completed skeleton with 3 nodes

=== SEGMENTATION COMPLETE ===

Visualization complete:
  - 5 skeletal segments (colored by rigidity)
  - 3 mesh components
  - 3 skeleton nodes (magenta points)

=== SEGMENTATION SUMMARY ===
Ô£ô Successfully segmented mesh into 3 components
Ô£ô Generated partial skeleton with 5 segments
Ô£ô Created complete skeleton with 3 nodes
Ô£ô Used 6 FPS sample points
"