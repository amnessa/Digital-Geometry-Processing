#pragma once

#include <vector>
#include <Eigen/Dense>
#include "Mesh.h"
#include "helpers.h"

// Forward declarations
class SoSeparator;
class SoWinExaminerViewer;
class Painter;

/**
 * Pairwise Harmonics Segmentation Algorithm
 * Implementation of the complete algorithm from "Pairwise Harmonics for Shape Analysis"
 * by Zheng et al., IEEE TVCG 2013
 */
class PairwiseHarmonicsSegmentation {
public:    struct SegmentationParams {
        int numFPSSamples = 25;           // Number of FPS samples
        int numIsocurves = 50;            // K: number of isocurves per harmonic field
        double rigidityThreshold = 0.8;   // Threshold for high-rigidity classification (limb centers)
        double isoTolerance = 0.03;       // Tolerance for isocurve bracelet thickness
        double minSegmentLength = 0.3;    // Minimum segment length (relative to endpoint distance)
        int minSkeletalNodes = 4;         // Minimum nodes for valid skeletal segment
        int maxSkeletonSegments = 10;      // Maximum segments in partial skeleton
        double junctionThreshold = 0.6;  // Threshold for anatomical junctions (body connections)
    };

    struct SkeletalSegment {
        std::vector<Eigen::Vector3d> nodes;   // Skeletal nodes (isocurve centroids)
        int sourceIdx, targetIdx;             // FPS point indices (p, q)
        double avgRigidity;                   // Average rigidity score
        double length;                        // Total length of segment
        double quality;                       // Quality metric (rigidity * length)
        std::vector<double> nodeRigidities;   // Rigidity score for each node
    };

    struct MeshComponent {
        std::vector<int> vertexIndices;       // Vertices in this component
        Eigen::Vector3d center;               // Geometric center
        int skeletonSegmentId;                // Associated skeleton segment (-1 if none)
    };    struct SegmentationResult {
        std::vector<SkeletalSegment> partialSkeleton;
        std::vector<MeshComponent> meshComponents;
        std::vector<Eigen::Vector3d> skeletonNodes;  // Complete skeleton nodes
        std::vector<int> fpsPoints;                  // FPS sample points used
        bool success;
        std::string errorMessage;
    };

private:
    Mesh* mesh;
    PairwiseHarmonics* harmonics;
    SegmentationParams params;

    // Global rigidity analysis results
    std::vector<double> globalVertexRigidity;  // Rigidity value for each mesh vertex
    bool rigidityComputed;

    // Internal methods
    std::vector<int> computeFPSSamples();
    std::vector<SkeletalSegment> generateSkeletalSegments(const std::vector<int>& fpsPoints);
    void computeRigidityAnalysis(std::vector<SkeletalSegment>& segments);
    void computeGlobalVertexRigidity();  // NEW: Compute rigidity for all mesh vertices
    std::vector<SkeletalSegment> selectPartialSkeleton(const std::vector<SkeletalSegment>& segments);
    std::vector<MeshComponent> createInitialSegmentation(const std::vector<SkeletalSegment>& skeleton);
    void completeSkeleton(std::vector<MeshComponent>& components, std::vector<Eigen::Vector3d>& skeletonNodes);
    void refineSegmentation(std::vector<MeshComponent>& components, const std::vector<Eigen::Vector3d>& skeleton);

    // Helper methods
    double computeNodeRigidity(const std::vector<Eigen::Vector3d>& neighborhood);
    std::vector<Eigen::Vector3d> extractIsocurveCentroids(const Eigen::VectorXd& harmonicField, int sourceIdx, int targetIdx);
    std::vector<SkeletalSegment> extractRigidityBasedSegments(const Eigen::VectorXd& harmonicField, int sourceIdx, int targetIdx);
    std::vector<MeshComponent> createImprovedSegmentation(const std::vector<SkeletalSegment>& skeleton); // New improved approach

public:
    /**
     * Constructor
     * @param m Pointer to the mesh
     * @param h Pointer to the pairwise harmonics object
     */
    PairwiseHarmonicsSegmentation(Mesh* m, PairwiseHarmonics* h);

    /**
     * Set segmentation parameters
     * @param p New parameters
     */
    void setParameters(const SegmentationParams& p) { params = p; }

    /**
     * Get current parameters
     * @return Current parameters
     */
    const SegmentationParams& getParameters() const { return params; }

    /**
     * Perform the complete 4-stage segmentation algorithm
     * @return Segmentation result containing all output data
     */
    SegmentationResult performSegmentation();    /**
     * Visualize segmentation results
     * @param result The segmentation result to visualize
     * @param root Scene graph root
     * @param painter Painter object for visualization
     * @param viewer 3D viewer for rendering
     * @param mesh Mesh object for vertex access
     */
    static void visualizeResults(
        const SegmentationResult& result,
        SoSeparator* root,
        Painter* painter,
        SoWinExaminerViewer* viewer,
        Mesh* mesh
    );

    /**
     * Test individual components of the algorithm
     */
    struct ComponentTest {
        static void testFPS(Mesh* mesh, SoSeparator* root, Painter* painter);
        static void testHarmonicFields(Mesh* mesh, PairwiseHarmonics* harmonics, SoSeparator* root, Painter* painter);
        static void testIsocurveExtraction(Mesh* mesh, PairwiseHarmonics* harmonics, SoSeparator* root, Painter* painter);
        static void testRigidityAnalysis(Mesh* mesh, PairwiseHarmonics* harmonics, SoSeparator* root, Painter* painter);
    };
};
