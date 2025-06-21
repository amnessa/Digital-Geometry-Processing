# Pairwise Harmonics Segmentation - Clean Implementation

This is a focused, cleaned-up implementation of the "Pairwise Harmonics for Shape Analysis" paper by Zheng et al., IEEE TVCG 2013, specifically focusing on the **Simultaneous Skeletonization and Segmentation** algorithm described in Section 3.3.

## What Was Cleaned Up

### Removed Components:
- `main.cpp` - Replaced with deprecated placeholder
- Redundant visualization modules (`VisualizationUtils`, `IsocurveAnalysis`, `RigidityAnalysis`)
- Complex menu systems with too many testing options
- Assignment-specific legacy code
- Symmetry detection functions (not part of segmentation)
- Redundant geodesic computation methods

### Kept Core Components:
- `main_clean.cpp` - Streamlined main implementation
- `PairwiseHarmonicsSegmentation.cpp/h` - Core segmentation algorithm
- `helpers.cpp/h` - PairwiseHarmonics class with LibIGL integration
- `Mesh.cpp/h` - Core mesh operations and FPS
- `Painter.cpp/h` - Visualization utilities

## Algorithm Implementation

The cleaned implementation follows the 4-stage algorithm from the paper:

### Stage 1: Farthest Point Sampling (FPS)
- Uses max-min sampling strategy as described in the paper
- Generates sparse sample points on prominent shape parts
- Default: 25 samples (adjustable)

### Stage 2: Skeletal Segment Generation
- Computes pairwise harmonic fields for all FPS point pairs
- Extracts K isocurves from each harmonic field (default K=50)
- Connects isocurve centroids to form skeletal segments
- Uses LibIGL for robust harmonic field computation

### Stage 3: Rigidity Analysis & Partial Skeleton Construction
- Analyzes skeletal nodes using PCA-based rigidity scores
- Selects high-quality segments using rigidity threshold (default 0.9)
- Builds connected partial skeleton using greedy selection

### Stage 4: Initial Segmentation & Skeleton Completion
- Creates initial mesh segmentation based on harmonic field boundaries
- Completes skeleton by filling gaps between components
- Refines segmentation based on completed skeleton

## Key Mathematical Components

### Pairwise Harmonic Fields
- Solves Laplace equation ΔF = 0 with Dirichlet boundary conditions
- Uses cotangent-weighted Laplacian discretization
- Boundary conditions: f(p) = 0, f(q) = 1

### Isocurve Extraction
- Extracts level sets at regular intervals: k/K for k = 1,2,...,K-1
- Uses triangle marching for robust curve extraction
- Computes centroids as skeletal nodes

### Rigidity Analysis
- Uses PCA on skeletal node neighborhoods
- Rigidity score: ξ = (λ₁ + λ₂ + λ₃) / max(λ₁, λ₂, λ₃)
- Higher scores indicate more linear (rigid) arrangements

## Build Instructions

### Prerequisites
- CMake 3.10+
- Visual Studio 2019+ (Windows)
- Coin3D library (already included)
- Eigen3 library (already included)
- libigl library (already included)

### Building
1. Use the clean CMakeLists:
   ```bash
   cp CMakeLists_clean.txt CMakeLists.txt
   ```

2. Build with CMake:
   ```bash
   mkdir build_clean
   cd build_clean
   cmake ..
   cmake --build . --config Debug
   ```

3. Run the executable:
   ```bash
   cd Debug
   ./PairwiseHarmonicsSegmentation.exe
   ```

## Usage

### Simple Workflow:
1. **Load mesh** (option 1) - Load a .off mesh file
2. **Run segmentation** (option 2) - Execute the complete algorithm
3. **Visualize results** (option 3) - See skeletal segments and mesh components

### Testing Individual Components:
Use option 4 to test:
- FPS sampling
- Pairwise harmonic computation
- Isocurve extraction
- Rigidity analysis

### Parameter Tuning:
- **FPS samples**: More samples = finer skeleton (15-30 recommended)
- **Isocurves (K)**: More isocurves = smoother skeleton (30-50 recommended)
- **Rigidity threshold**: Higher = more selective skeleton (0.85-0.95 recommended)

## Paper Reference

The implementation follows:
> Y. Zheng, C.-L. Tai, E. Zhang, and P. Xu, "Pairwise harmonics for shape analysis,"
> IEEE Transactions on Visualization and Computer Graphics, vol. 19, no. 7, pp. 1172-1184, 2013.

**Key contribution**: Novel pairwise analysis framework using harmonic functions for simultaneous skeletonization and segmentation.

## File Structure

```
├── main_clean.cpp              # Clean main implementation
├── PairwiseHarmonicsSegmentation.h/cpp  # Core algorithm
├── helpers.h/cpp               # PairwiseHarmonics class
├── Mesh.h/cpp                  # Mesh operations & FPS
├── Painter.h/cpp               # Visualization
├── CMakeLists_clean.txt        # Clean build configuration
├── models/                     # Test mesh files
│   ├── man0.off
│   ├── Armadillo.off
│   └── *.off
└── README_CLEAN.md            # This file
```

## Results

The algorithm produces:
- **Skeletal segments**: 1D skeleton representing shape structure
- **Mesh components**: 3D surface segmentation
- **Quality metrics**: Rigidity scores for skeleton validation

**Visualization**:
- Red lines: Selected skeletal segments
- Colored spheres: FPS sample points
- Different colors: Segmented mesh components
