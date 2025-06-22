# Rigidity-Based Segmentation Fix

## Problem Identified

Based on user analysis and research papers, the original implementation had a fundamental misunderstanding of rigidity values and thresholding:

### Original Issues:
1. **Skeletal segments were being cut too aggressively** - segments weren't reaching limb endpoints
2. **Wrong threshold interpretation** - using 0.9 threshold was cutting ALL skeletal nodes (0.55-0.83 range)
3. **Mismatched paper theory** - treating limb centers as "low rigidity" instead of preserving them

### Data Analysis:
- **Global rigidity range**: [0.40569, 0.999857]
- **Skeletal segment rigidities**: 0.55-0.83 (these ARE the limb centers!)
- **Original threshold**: 0.9 (cutting all skeletal nodes incorrectly)

## Theoretical Foundation (from Research Papers)

From "how_to_separate_segments.md" and other research:

### Correct Rigidity Interpretation:
- **High Rigidity (>0.8)**: Limb centers, skeletal paths - should be PRESERVED as complete segments
- **Medium Rigidity (0.6-0.8)**: Transition zones, potential junction areas
- **Low Rigidity (<0.6)**: Main body/torso - these are the "leftovers" not assigned to limbs

### Segmentation Approach:
1. **Preserve limb segments**: Keep skeletal paths as complete segments from extremity to junction
2. **Conservative junction detection**: Only cut at true anatomical junctions (torso connections)
3. **Torso as leftovers**: Vertices not assigned to limbs become torso/body segments

## Implementation Changes

### 1. Updated Parameters (`PairwiseHarmonicsSegmentation.h`)
```cpp
struct SegmentationParams {
    double rigidityThreshold = 0.8;   // HIGH-rigidity threshold (was 0.9)
    double junctionThreshold = 0.6;  // Anatomical junction threshold (was 0.7)
    // ...
};
```

### 2. CRITICAL FIX: Include FPS Endpoints in Skeletal Paths

**Problem**: Original `extractIsocurveCentroids` only extracted intermediate isocurves, NOT the actual FPS endpoints!
```cpp
// OLD (BROKEN): Only intermediate points
for (int k = 1; k < params.numIsocurves; k++) { // Missing k=0 and k=numIsocurves!
```

**Solution**: Always include source and target FPS points as skeletal endpoints
```cpp
// NEW (FIXED): Include actual FPS endpoints
centroids.push_back(sourcePoint);  // Add source FPS (hand/foot)
for (int k = 1; k < params.numIsocurves; k++) { // Intermediate isocurves
    // ... extract isocurves ...
}
centroids.push_back(targetPoint);   // Add target FPS (junction/torso)
```

This ensures skeletal paths actually reach hands, feet, and other extremities!

### 3. Improved Endpoint Protection

**Problem**: Cutting logic was too aggressive near limb extremities
```cpp
// OLD: Could cut anywhere along path
for (int i = 1; i < rigidities.size() - 1; i++) {
```

**Solution**: Protect endpoints from cutting
```cpp
// NEW: Never cut near endpoints (preserve limb extremities)
for (int i = 3; i < rigidities.size() - 3; i++) { // Skip first/last 3 points
```

### 4. Enhanced Segment Selection (Diversity Fix)

**Problem**: Multiple segments originated from same FPS point (e.g., 4 out of 5 from FPS 832)

**Solution**: Two-phase selection ensuring spatial diversity
```cpp
// Phase 1: Select best segment from each unique source FPS point
map<int, vector<SkeletalSegment>> segmentsBySource;
// Phase 2: Final selection with strict diversity constraints
```

## Expected Results

With these changes, the algorithm should:

1. **Preserve complete limb segments** - skeletal paths from hands to shoulders, feet to hips
2. **Reduce over-segmentation** - fewer, more meaningful segments
3. **Better anatomical correspondence** - limbs as single segments, torso as separate component
4. **Match research expectations** - high-rigidity paths preserved, low-rigidity areas become torso

## Testing

Run the segmentation and check:
- Option 20: "Analyze Skeletal Segments" - should show fewer, longer segments
- Option 21: "Detailed Rigidity Analysis" - should show fewer or no cuts per segment
- Option 18/19: Visualization should show more complete limb segments

The segments should now reach from extremities (hands/feet) to true junction points (shoulders/hips) rather than being cut into small pieces.
