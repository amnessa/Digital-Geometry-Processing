# Rigidity-Based Skeletal Segment Cutting: Theoretical Foundation

## Problem with Original Implementation

The original algorithm created full-length skeletal segments between FPS point pairs, resulting in:
- All segments spanning entire limbs
- No consideration of anatomical junctions
- Segments that are too long and not anatomically meaningful
- All segments potentially starting from similar central locations

## Paper-Based Solution: Rigidity Analysis for Segment Cutting

According to "Pairwise Harmonics for Shape Analysis" by Zheng et al., the rigidity analysis should be used to:

### 1. **Identify Limb Centers (High Rigidity)**
- **Cylindrical limbs** (arms, legs) have high rigidity values (ξ ≈ 0.8-0.9)
- These areas should form the **core** of skeletal segments
- High rigidity indicates linear/cylindrical local geometry

### 2. **Detect Junctions (Low Rigidity)**
- **Joint areas** (shoulders, elbows, hips, knees) have low rigidity (ξ ≈ 0.3-0.6)
- These areas should **terminate** skeletal segments
- Low rigidity indicates complex/branching local geometry

### 3. **Segment Cutting Strategy**
Instead of creating one segment per FPS pair:
```
Original: FPS_A ──────────────────── FPS_B (one long segment)

New:      FPS_A ───── junction ───── FPS_B (multiple shorter segments)
                      ↑
                  (low rigidity)
```

## Mathematical Foundation

### Rigidity Computation (PCA-based)
For a local neighborhood of skeletal nodes:
```
ξ = λ₁ / (λ₁ + λ₂ + λ₃)
```
Where λ₁ ≥ λ₂ ≥ λ₃ are eigenvalues of the covariance matrix.

### Interpretation:
- **ξ ≈ 1.0**: Perfect linear structure (limb center)
- **ξ ≈ 0.33**: Isotropic structure (junction/branching)
- **ξ ≈ 0.5**: Planar structure (transition area)

### Cutting Threshold
- **Paper recommendation**: 0.85-0.95 for very strict cutting
- **Our implementation**: 0.7 for more sensitive junction detection
- **Effect**: Lower threshold = more cuts = shorter segments

## Expected Anatomical Results

### Before (Original Algorithm):
- **Arm segment**: Shoulder → Hand (entire arm)
- **Leg segment**: Hip → Foot (entire leg)
- **Few segments**: Only 3-5 major body parts

### After (Rigidity-Based Cutting):
- **Upper arm**: Shoulder → Elbow
- **Forearm**: Elbow → Wrist
- **Upper leg**: Hip → Knee
- **Lower leg**: Knee → Ankle
- **More segments**: 8-12 anatomical parts

## Implementation Details

### New Algorithm Flow:
1. **Extract isocurve centroids** from harmonic field
2. **Compute rigidity** at each centroid using local PCA
3. **Identify cut points** where rigidity < threshold
4. **Create multiple segments** between cut points
5. **Filter by length** and quality metrics

### Key Code Changes:
- `extractRigidityBasedSegments()`: New method for cutting
- `rigidityThreshold = 0.7`: Lowered from 0.9
- Multiple segments per FPS pair instead of one

This approach should generate skeletal segments that better correspond to anatomical body parts and eliminate the "all segments start from same point" problem.
