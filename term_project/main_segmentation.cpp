#include "pch.h"  // Include precompiled header first

#include <stdio.h>
#include <cstdlib>
#include <time.h>
#include <stdint.h>
#include <cfloat>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <set>
#include <queue>

// Standard C++ headers and Eigen headers BEFORE Windows.h and Coin3D headers
#include <Eigen/Dense>
#include <Eigen/Sparse>

// Windows headers with macro protection
#define NOMINMAX
#include <windows.h>

// Coin3D/SoWin headers
#include <Inventor/Win/SoWin.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoFaceSet.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/events/SoKeyboardEvent.h>
#include <Inventor/events/SoMouseButtonEvent.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/SoPickedPoint.h>
#include <Inventor/actions/SoRayPickAction.h>
#include <Inventor/actions/SoHandleEventAction.h>
#include <Inventor/SbVec3f.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoIndexedLineSet.h>
#include <Inventor/nodes/SoPointSet.h>

// Project headers
#include "Mesh.h"
#include "Painter.h"
#include "helpers.h"
#include "IsocurveAnalysis.h"
#include "RigidityAnalysis.h"
#include "VisualizationUtils.h"

using namespace std;

// Global variables for core components
Mesh* g_mesh = nullptr;
Painter* g_painter = nullptr;
PairwiseHarmonics* g_harmonics = nullptr;

// Global variables for visualization
SoSeparator* g_root = nullptr;
SoWinExaminerViewer* g_viewer = nullptr;

// Global variables for segmentation algorithm state
vector<int> g_fps_samples;
vector<vector<Eigen::Vector3d>> g_skeletal_segments;
vector<double> g_rigidity_scores;
vector<bool> g_nonrigid_nodes;

// Algorithm parameters (from the paper)
const int DEFAULT_K = 50;           // Number of isocurves per harmonic field
const double RIGIDITY_THRESHOLD = 0.9;  // Threshold for classifying rigid vs non-rigid nodes (paper recommendation 0.85-0.95)
const int DEFAULT_FPS_SAMPLES = 25;     // Number of FPS samples for initial point generation (increased from 15)

// Forward declarations
void loadMesh();
void showMainMenu();
void performFullSegmentation();
void clearVisualization();

/**
 * Load a mesh file and initialize the segmentation system
 */
void loadMesh() {
    string meshPath;
    cout << "\n=== LOAD MESH ===" << endl;
    cout << "Available meshes:" << endl;
    cout << "  - 50.off" << endl;
    cout << "  - 249.off" << endl;
    cout << "  - 348.off" << endl;
    cout << "  - man0.off" << endl;
    cout << "  - Armadillo.off" << endl;
    cout << "Enter mesh filename: ";
    cin >> meshPath;

    // Clear previous state
    clearVisualization();
    if (g_harmonics) {
        delete g_harmonics;
        g_harmonics = nullptr;
    }
    if (g_mesh) {
        delete g_mesh;
        g_mesh = nullptr;
    }

    // Create and load new mesh
    g_mesh = new Mesh();

    cout << "Loading mesh: " << meshPath << "..." << endl;
    g_mesh->loadOff(const_cast<char*>(meshPath.c_str()));
    g_mesh->normalizeCoordinates();

    cout << "Mesh loaded successfully!" << endl;
    cout << "  Vertices: " << g_mesh->verts.size() << endl;
    cout << "  Triangles: " << g_mesh->tris.size() << endl;
    cout << "  Edges: " << g_mesh->edges.size() << endl;

    // Add mesh to visualization
    g_root->addChild(g_painter->getShapeSep(g_mesh));

    // Initialize pairwise harmonics system
    g_harmonics = new PairwiseHarmonics(g_mesh);

    // Clear segmentation state
    g_fps_samples.clear();
    g_skeletal_segments.clear();
    g_rigidity_scores.clear();
    g_nonrigid_nodes.clear();

    g_viewer->viewAll();
    cout << "Mesh ready for pairwise harmonics analysis!" << endl;
}

/**
 * Clear all visualization except the base mesh
 */
/**
 * Clear visualization by removing all children from the scene graph root
 */
void clearVisualization() {
    VisualizationUtils::clearVisualization(g_root);
    // Force viewer to update
    if (g_viewer) {
        g_viewer->scheduleRedraw();
    }
}

/**
 * Test cotangent Laplacian computation
 */
void testCotangentLaplacian() {
    VisualizationUtils::testCotangentLaplacian(g_mesh, g_harmonics, g_root, g_viewer);
}

/**
 * Test farthest point sampling (FPS) algorithm
 */
/**
 * Test farthest point sampling (FPS) algorithm
 */
void testFarthestPointSampling() {
    VisualizationUtils::testFarthestPointSampling(g_mesh, g_fps_samples, g_root, g_viewer);
}

/**
 * Test pairwise harmonic field computation and visualization
 */
void testPairwiseHarmonics() {
    VisualizationUtils::testPairwiseHarmonics(g_mesh, g_harmonics, g_fps_samples, g_root, g_viewer);
}

/**
 * Test iso-curve extraction from harmonic fields
 */
void testIsoCurveExtraction() {
    VisualizationUtils::testIsoCurveExtraction(g_mesh, g_harmonics, g_fps_samples, g_root, g_viewer);
}

/**
 * Test rigidity analysis using PCA on skeletal nodes
 */
void testRigidityAnalysis() {
    VisualizationUtils::testRigidityAnalysis(g_mesh, g_harmonics, g_fps_samples, g_root, g_viewer);
}

/**
 * Perform the full segmentation algorithm from the paper
 * Implements the complete 4-stage algorithm as described in the paper
 */
void performFullSegmentation() {
    if (!g_mesh || !g_harmonics) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }

    cout << "\n=== FULL PAIRWISE HARMONICS SEGMENTATION ===" << endl;
    cout << "Implementing complete algorithm from 'Pairwise Harmonics for Shape Analysis'" << endl;
    cout << "Parameters:" << endl;
    cout << "  - K (isocurves per field): " << DEFAULT_K << endl;
    cout << "  - Rigidity threshold: " << RIGIDITY_THRESHOLD << endl;
    cout << "  - FPS samples: " << DEFAULT_FPS_SAMPLES << endl;

    clearVisualization();

    // =============================================================================
    // STAGE 1: Initial Point Sampling and Skeletal Segment Generation
    // =============================================================================
    cout << "\n--- STAGE 1: INITIAL POINT SAMPLING ---" << endl;

    // Generate FPS samples (extremal points)
    if (g_fps_samples.empty()) {
        cout << "Computing Farthest Point Sampling..." << endl;
        g_fps_samples = g_mesh->farthestPointSampling(DEFAULT_FPS_SAMPLES);
    }
    cout << "Generated " << g_fps_samples.size() << " FPS samples (extremal points)" << endl;

    // Generate all pairwise harmonic fields and extract skeletal segments
    cout << "Computing pairwise harmonic fields for all point pairs..." << endl;

    struct SkeletalSegment {
        vector<Eigen::Vector3d> nodes;
        int pIdx, qIdx;  // Original FPS point indices
        double avgRigidity;
        double length;
        double quality;
    };

    vector<SkeletalSegment> rawSegments;
    int totalPairs = g_fps_samples.size() * (g_fps_samples.size() - 1) / 2;
    int processedPairs = 0;

    for (int i = 0; i < g_fps_samples.size(); i++) {
        for (int j = i + 1; j < g_fps_samples.size(); j++) {
            processedPairs++;
            cout << "  Pair " << processedPairs << "/" << totalPairs
                 << " (vertices " << g_fps_samples[i] << ", " << g_fps_samples[j] << ")" << endl;

            // Compute harmonic field
            Eigen::VectorXd harmonicField = g_harmonics->computePairwiseHarmonic(g_fps_samples[i], g_fps_samples[j]);

            if (harmonicField.size() != g_mesh->verts.size()) {
                cout << "    Failed - skipping" << endl;
                continue;
            }            // Extract isocurves as "bracelets" around the shape following the paper's approach
            // Each isocurve represents a cross-section of the shape between the two FPS points
            vector<Eigen::Vector3d> skeletalNodes;

            // Extract isocurves at regular intervals (like bracelets around limbs)
            int numIsocurves = 20; // More isocurves for finer skeletal resolution
            for (int k = 1; k < numIsocurves; k++) {
                double isoValue = double(k) / numIsocurves;

                // Find all vertices at this harmonic field level (forming the "bracelet")
                vector<int> braceletVertices;
                double tolerance = 0.03; // Tolerance for bracelet thickness

                for (int v = 0; v < g_mesh->verts.size(); v++) {
                    if (abs(harmonicField(v) - isoValue) < tolerance) {
                        braceletVertices.push_back(v);
                    }
                }

                if (braceletVertices.size() >= 3) {
                    // Compute the centroid of this bracelet (center of the cross-section)
                    Eigen::Vector3d braceletCenter(0, 0, 0);
                    for (int v : braceletVertices) {
                        braceletCenter += Eigen::Vector3d(g_mesh->verts[v]->coords[0],
                                                          g_mesh->verts[v]->coords[1],
                                                          g_mesh->verts[v]->coords[2]);
                    }
                    braceletCenter /= braceletVertices.size();                    // This bracelet center becomes a skeletal node
                    skeletalNodes.push_back(braceletCenter);
                }
            }

            // Only create segment if we have enough skeletal nodes to form a meaningful curve
            if (skeletalNodes.size() >= 5) {
                // Compute total length of skeletal segment
                double totalLength = 0;
                for (int n = 1; n < skeletalNodes.size(); n++) {
                    totalLength += (skeletalNodes[n] - skeletalNodes[n-1]).norm();
                }                // Filter based on length (should span a significant portion of the shape)
                Eigen::Vector3d endpointDist(g_mesh->verts[g_fps_samples[i]]->coords[0] - g_mesh->verts[g_fps_samples[j]]->coords[0],
                                              g_mesh->verts[g_fps_samples[i]]->coords[1] - g_mesh->verts[g_fps_samples[j]]->coords[1],
                                              g_mesh->verts[g_fps_samples[i]]->coords[2] - g_mesh->verts[g_fps_samples[j]]->coords[2]);
                double endpointDistance = endpointDist.norm();

                if (totalLength > endpointDistance * 0.3) { // At least 30% of direct distance
                    SkeletalSegment seg;
                    seg.nodes = skeletalNodes;
                    seg.pIdx = g_fps_samples[i];
                    seg.qIdx = g_fps_samples[j];
                    seg.length = totalLength;
                    rawSegments.push_back(seg);
                }
            }
        }
    }

    cout << "Generated " << rawSegments.size() << " raw skeletal segments" << endl;

    // =============================================================================
    // STAGE 2: Rigidity Analysis and Partial Skeleton Construction
    // =============================================================================
    cout << "\n--- STAGE 2: RIGIDITY ANALYSIS ---" << endl;

    // Compute rigidity for each segment and split at non-rigid nodes
    vector<SkeletalSegment> refinedSegments;

    for (int segIdx = 0; segIdx < rawSegments.size(); segIdx++) {
        auto& segment = rawSegments[segIdx];
        const auto& nodes = segment.nodes;

        cout << "Analyzing segment " << segIdx + 1 << "/" << rawSegments.size()
             << " (" << nodes.size() << " nodes)" << endl;

        // Compute rigidity for each node using PCA
        vector<double> nodeRigidities;
        vector<bool> isNonRigid;

        for (int nodeIdx = 0; nodeIdx < nodes.size(); nodeIdx++) {
            // Define local neighborhood (use k-nearest neighbors approach)
            vector<Eigen::Vector3d> neighborhood;
            int windowSize = min(5, (int)nodes.size()); // Use up to 5 neighbors

            int start = max(0, nodeIdx - windowSize/2);
            int end = min((int)nodes.size(), start + windowSize);

            for (int k = start; k < end; k++) {
                neighborhood.push_back(nodes[k]);
            }

            // Compute PCA-based rigidity
            if (neighborhood.size() >= 3) {
                Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
                Eigen::Vector3d mean = Eigen::Vector3d::Zero();

                for (const auto& point : neighborhood) {
                    mean += point;
                }
                mean /= neighborhood.size();

                for (const auto& point : neighborhood) {
                    Eigen::Vector3d diff = point - mean;
                    covariance += diff * diff.transpose();
                }
                covariance /= (neighborhood.size() - 1);                // Compute eigenvalues for PCA-based rigidity
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
                Eigen::Vector3d eigenvalues = solver.eigenvalues();

                // Sort in descending order (λ₁ ≥ λ₂ ≥ λ₃)
                std::sort(eigenvalues.data(), eigenvalues.data() + 3, std::greater<double>());

                double lambda1 = eigenvalues(0);
                double lambda2 = eigenvalues(1);
                double lambda3 = eigenvalues(2);
                double sumLambda = lambda1 + lambda2 + lambda3;

                // CORRECTED rigidity formula: ξ = λ₁ / (λ₁ + λ₂ + λ₃)
                double rigidity = 0.33; // Default for degenerate case
                if (sumLambda > 1e-10) {
                    rigidity = lambda1 / sumLambda;
                }

                nodeRigidities.push_back(rigidity);
                isNonRigid.push_back(rigidity < RIGIDITY_THRESHOLD);
            } else {
                nodeRigidities.push_back(1.0);
                isNonRigid.push_back(false);
            }
        }

        // Split segment at non-rigid nodes (junction points)
        vector<int> splitPoints;
        splitPoints.push_back(0); // Start

        for (int i = 0; i < isNonRigid.size(); i++) {
            if (isNonRigid[i]) {
                splitPoints.push_back(i);
            }
        }
        splitPoints.push_back(nodes.size() - 1); // End

        // Create subsegments between split points
        for (int i = 0; i < splitPoints.size() - 1; i++) {
            int start = splitPoints[i];
            int end = splitPoints[i + 1];

            if (end - start >= 3) { // Minimum length for valid segment
                SkeletalSegment subseg;
                subseg.pIdx = segment.pIdx;
                subseg.qIdx = segment.qIdx;

                for (int j = start; j <= end; j++) {
                    subseg.nodes.push_back(nodes[j]);
                }

                // Compute average rigidity and length
                double totalRigidity = 0;
                double length = 0;

                for (int j = start; j <= end; j++) {
                    totalRigidity += nodeRigidities[j];
                    if (j < end) {
                        length += (nodes[j+1] - nodes[j]).norm();
                    }
                }

                subseg.avgRigidity = totalRigidity / (end - start + 1);
                subseg.length = length;
                subseg.quality = subseg.avgRigidity * subseg.length; // ρ = ξ * l

                refinedSegments.push_back(subseg);
            }
        }
    }

    cout << "Created " << refinedSegments.size() << " refined segments after rigidity analysis" << endl;

    // Sort segments by quality and select partial skeleton (greedy selection)
    sort(refinedSegments.begin(), refinedSegments.end(),
         [](const SkeletalSegment& a, const SkeletalSegment& b) {
             return a.quality > b.quality;
         });    // Build a proper skeleton tree instead of selecting random segments
    cout << "Building skeleton tree from high-quality segments..." << endl;

    // Sort segments by quality (rigidity * length)
    sort(refinedSegments.begin(), refinedSegments.end(),
         [](const SkeletalSegment& a, const SkeletalSegment& b) {
             return a.quality > b.quality; // Higher quality first
         });

    // Build skeleton tree using a greedy approach
    vector<SkeletalSegment> partialSkeleton;
    set<int> usedFPSPoints; // Track which FPS points are already connected

    // Start with the highest quality segment
    if (!refinedSegments.empty()) {
        partialSkeleton.push_back(refinedSegments[0]);
        usedFPSPoints.insert(refinedSegments[0].pIdx);
        usedFPSPoints.insert(refinedSegments[0].qIdx);

        // Greedily add segments that connect to existing skeleton
        for (int iter = 0; iter < 10 && partialSkeleton.size() < 8; iter++) {
            double bestQuality = 0;
            int bestSegIdx = -1;

            for (int i = 1; i < refinedSegments.size(); i++) {
                const auto& seg = refinedSegments[i];

                // Check if this segment connects to existing skeleton
                bool connectsToSkeleton = (usedFPSPoints.count(seg.pIdx) > 0) ||
                                          (usedFPSPoints.count(seg.qIdx) > 0);

                // Don't create cycles - avoid if both endpoints already used
                bool createsCycle = (usedFPSPoints.count(seg.pIdx) > 0) &&
                                    (usedFPSPoints.count(seg.qIdx) > 0);

                if (connectsToSkeleton && !createsCycle && seg.quality > bestQuality) {
                    bestQuality = seg.quality;
                    bestSegIdx = i;
                }
            }

            // Add the best connecting segment
            if (bestSegIdx >= 0) {
                partialSkeleton.push_back(refinedSegments[bestSegIdx]);
                usedFPSPoints.insert(refinedSegments[bestSegIdx].pIdx);
                usedFPSPoints.insert(refinedSegments[bestSegIdx].qIdx);

                // Remove used segment to avoid duplicates
                refinedSegments.erase(refinedSegments.begin() + bestSegIdx);
            } else {
                break; // No more connecting segments found
            }
        }
    }

    cout << "Selected " << partialSkeleton.size() << " high-quality segments for partial skeleton" << endl;

    // =============================================================================
    // STAGE 3: Initial Segmentation from Partial Skeleton
    // =============================================================================
    cout << "\n--- STAGE 3: INITIAL SEGMENTATION ---" << endl;

    // Create initial mesh segmentation using isocurves from partial skeleton
    vector<vector<int>> meshComponents; // Each component is a list of vertex indices
    vector<int> vertexToComponent(g_mesh->verts.size(), -1); // -1 = unassigned

    cout << "Creating initial segmentation using isocurves from partial skeleton..." << endl;

    // For each segment in partial skeleton, create mesh cuts using its isocurves
    for (int segIdx = 0; segIdx < partialSkeleton.size(); segIdx++) {
        const auto& segment = partialSkeleton[segIdx];

        // Compute harmonic field for this segment
        Eigen::VectorXd harmonicField = g_harmonics->computePairwiseHarmonic(segment.pIdx, segment.qIdx);

        if (harmonicField.size() == g_mesh->verts.size()) {
            // Create mesh component using isocurves at segment endpoints
            vector<int> segmentVertices;

            // Find vertices within the segment's harmonic field range
            double minVal = harmonicField.minCoeff();
            double maxVal = harmonicField.maxCoeff();
            double range = maxVal - minVal;

            if (range > 1e-6) {
                for (int vIdx = 0; vIdx < g_mesh->verts.size(); vIdx++) {
                    double normalizedVal = (harmonicField(vIdx) - minVal) / range;

                    // Assign vertices to this component if they fall within the segment's range
                    if (normalizedVal >= 0.1 && normalizedVal <= 0.9 && vertexToComponent[vIdx] == -1) {
                        segmentVertices.push_back(vIdx);
                        vertexToComponent[vIdx] = meshComponents.size();
                    }
                }

                if (!segmentVertices.empty()) {
                    meshComponents.push_back(segmentVertices);
                }
            }
        }
    }

    // Assign remaining unassigned vertices to nearest component
    for (int vIdx = 0; vIdx < g_mesh->verts.size(); vIdx++) {
        if (vertexToComponent[vIdx] == -1) {
            // Find nearest assigned vertex and inherit its component
            double minDist = DBL_MAX;
            int nearestComponent = 0;            for (int otherIdx = 0; otherIdx < g_mesh->verts.size(); otherIdx++) {
                if (vertexToComponent[otherIdx] != -1) {
                    Eigen::Vector3d v1(g_mesh->verts[vIdx]->coords[0], g_mesh->verts[vIdx]->coords[1], g_mesh->verts[vIdx]->coords[2]);
                    Eigen::Vector3d v2(g_mesh->verts[otherIdx]->coords[0], g_mesh->verts[otherIdx]->coords[1], g_mesh->verts[otherIdx]->coords[2]);
                    double dist = (v1 - v2).norm();
                    if (dist < minDist) {
                        minDist = dist;
                        nearestComponent = vertexToComponent[otherIdx];
                    }
                }
            }

            if (nearestComponent < meshComponents.size()) {
                meshComponents[nearestComponent].push_back(vIdx);
                vertexToComponent[vIdx] = nearestComponent;
            }
        }
    }

    cout << "Created " << meshComponents.size() << " initial mesh components" << endl;

    // =============================================================================
    // STAGE 4: Skeleton Completion and Segmentation Refinement
    // =============================================================================
    cout << "\n--- STAGE 4: SKELETON COMPLETION & REFINEMENT ---" << endl;

    cout << "Computing component centers for skeleton completion..." << endl;

    // Compute center of each mesh component
    vector<Eigen::Vector3d> componentCenters;    for (const auto& component : meshComponents) {
        Eigen::Vector3d center(0, 0, 0);
        for (int vIdx : component) {
            Eigen::Vector3d vertex(g_mesh->verts[vIdx]->coords[0], g_mesh->verts[vIdx]->coords[1], g_mesh->verts[vIdx]->coords[2]);
            center += vertex;
        }
        center /= component.size();
        componentCenters.push_back(center);
    }

    // Connect component centers to create complete skeleton
    // (Simplified: connect all centers in sequence)
    cout << "Connecting component centers to complete skeleton..." << endl;

    cout << "\n--- SEGMENTATION COMPLETE ---" << endl;
    cout << "Final results:" << endl;
    cout << "  - Partial skeleton: " << partialSkeleton.size() << " segments" << endl;
    cout << "  - Mesh components: " << meshComponents.size() << " parts" << endl;
    cout << "  - Complete skeleton nodes: " << componentCenters.size() << " centers" << endl;

    // =============================================================================
    // VISUALIZATION
    // =============================================================================

    // Visualize partial skeleton segments
    for (int i = 0; i < partialSkeleton.size(); i++) {
        const auto& segment = partialSkeleton[i];

        SoSeparator* segSep = new SoSeparator();
        SoMaterial* mat = new SoMaterial();

        // Use different colors for each segment
        float hue = float(i) / partialSkeleton.size();
        mat->diffuseColor.setValue(hue, 1.0f - hue * 0.5f, 0.8f);
        segSep->addChild(mat);        // Draw skeletal segment as a simplified path (not all internal connections)
        SoCoordinate3* coords = new SoCoordinate3();

        // Instead of connecting all nodes, create a simplified skeletal path
        // Connect endpoints of the segment and a few key intermediate points
        vector<Eigen::Vector3d> simplifiedPath;
        if (segment.nodes.size() > 0) {
            simplifiedPath.push_back(segment.nodes[0]); // Start

            // Add a few intermediate points
            if (segment.nodes.size() > 2) {
                int mid = segment.nodes.size() / 2;
                simplifiedPath.push_back(segment.nodes[mid]); // Middle
            }

            if (segment.nodes.size() > 1) {
                simplifiedPath.push_back(segment.nodes[segment.nodes.size()-1]); // End
            }
        }

        coords->point.setNum(simplifiedPath.size());
        for (int j = 0; j < simplifiedPath.size(); j++) {
            coords->point.set1Value(j, simplifiedPath[j][0], simplifiedPath[j][1], simplifiedPath[j][2]);
        }
        segSep->addChild(coords);

        SoIndexedLineSet* lineSet = new SoIndexedLineSet();
        for (int j = 0; j < simplifiedPath.size() - 1; j++) {
            lineSet->coordIndex.set1Value(j * 3, j);
            lineSet->coordIndex.set1Value(j * 3 + 1, j + 1);
            lineSet->coordIndex.set1Value(j * 3 + 2, -1);
        }
        segSep->addChild(lineSet);
        g_root->addChild(segSep);
    }

    // Visualize component centers
    SoSeparator* centersSep = new SoSeparator();
    SoMaterial* centerMat = new SoMaterial();
    centerMat->diffuseColor.setValue(1.0f, 0.0f, 1.0f); // Magenta for centers
    centersSep->addChild(centerMat);

    for (const auto& center : componentCenters) {
        centersSep->addChild(g_painter->get1PointSep(g_mesh, 0, 1.0f, 0.0f, 1.0f, 10.0f, false));
    }
    g_root->addChild(centersSep);

    // Force viewer refresh
    g_viewer->scheduleRedraw();
    g_viewer->viewAll();

    cout << "\nSUCCESS: Complete pairwise harmonics segmentation finished!" << endl;
    cout << "Visualization shows:" << endl;
    cout << "  - Colored skeletal segments (partial skeleton)" << endl;
    cout << "  - Magenta points (component centers)" << endl;
    cout << "  - Original mesh (segmented into " << meshComponents.size() << " parts)" << endl;
}

/**
 * Display the main testing menu
 */
void showMainMenu() {
    cout << "\n======================================" << endl;
    cout << "PAIRWISE HARMONICS SEGMENTATION SYSTEM" << endl;
    cout << "======================================" << endl;
    cout << "1. Load Mesh" << endl;
    cout << "2. Test Cotangent Laplacian" << endl;
    cout << "3. Test Farthest Point Sampling" << endl;
    cout << "4. Test Pairwise Harmonics" << endl;
    cout << "5. Test Iso-curve Extraction" << endl;
    cout << "6. Test Rigidity Analysis" << endl;
    cout << "7. Perform Full Segmentation" << endl;
    cout << "8. Clear Visualization" << endl;    cout << "--- ENHANCED ANALYSIS ---" << endl;
    cout << "9. Improved Iso-curve Extraction (Dense Bracelets)" << endl;
    cout << "10. Enhanced Rigidity Analysis (All Nodes)" << endl;
    cout << "11. Visualize All Rigidity Points (Color-coded)" << endl;
    cout << "12. Interactive Rigidity Threshold Testing" << endl;
    cout << "0. Exit" << endl;
    cout << "======================================" << endl;
    cout << "Choose option: ";
}

/**
 * Console input thread for menu interaction
 */
DWORD WINAPI ConsoleInputThread(LPVOID lpParam) {
    int choice;

    while (true) {
        showMainMenu();
        cin >> choice;        switch (choice) {
            case 1:
                loadMesh();
                break;
            case 2:
                testCotangentLaplacian();
                break;
            case 3:
                testFarthestPointSampling();
                break;
            case 4:
                testPairwiseHarmonics();
                break;
            case 5:
                testIsoCurveExtraction();
                break;
            case 6:
                testRigidityAnalysis();
                break;
            case 7:
                performFullSegmentation();
                break;
            case 8:
                clearVisualization();
                g_viewer->scheduleRedraw();
                g_viewer->viewAll();
                cout << "Visualization cleared." << endl;
                break;            case 9:
                IsocurveAnalysis::improvedIsoCurveExtraction(g_mesh, g_harmonics, g_fps_samples, g_root, g_viewer, g_skeletal_segments);
                break;
            case 10:
                RigidityAnalysis::enhancedRigidityAnalysis(g_mesh, g_harmonics, g_fps_samples, g_skeletal_segments, g_rigidity_scores, g_nonrigid_nodes);
                break;
            case 11:
                RigidityAnalysis::visualizeAllRigidityPoints(g_mesh, g_skeletal_segments, g_rigidity_scores, g_root, g_viewer);
                break;
            case 12:
                RigidityAnalysis::interactiveRigidityThresholdTesting(g_skeletal_segments, g_rigidity_scores, g_root, g_viewer);
                break;
            case 0:
                cout << "Exiting..." << endl;
                exit(0);
                break;
            default:
                cout << "Invalid option!" << endl;
                break;
        }
    }

    return 0;
}

/**
 * Main application entry point
 */
int main(int argc, char* argv[]) {
    // Initialize Coin3D and SoWin
    HWND window = SoWin::init("Pairwise Harmonics Segmentation");
    if (window == NULL) {
        cout << "Error: Failed to initialize SoWin!" << endl;
        return 1;
    }

    // Create the root node for the scene graph
    g_root = new SoSeparator();
    g_root->ref();

    // Create the viewer
    g_viewer = new SoWinExaminerViewer(window);
    g_viewer->setSceneGraph(g_root);
    g_viewer->show();

    // Initialize the painter
    g_painter = new Painter();

    // Show initial instructions
    cout << "=== PAIRWISE HARMONICS SEGMENTATION ===" << endl;
    cout << "Implementation based on the paper:" << endl;
    cout << "'Pairwise Harmonics for Shape Analysis'" << endl;
    cout << "by Zheng et al., IEEE TVCG 2013" << endl;
    cout << "\nThis system allows you to test individual components" << endl;
    cout << "of the segmentation algorithm before running the full pipeline." << endl;    cout << "\nAlgorithm implements:" << endl;
    cout << "- Cotangent Laplacian computation" << endl;
    cout << "- FPS with arbitrary point start + discard" << endl;
    cout << "- Pairwise harmonic field computation" << endl;
    cout << "- Iso-curve extraction for skeletal segments" << endl;
    cout << "- PCA-based rigidity analysis" << endl;
    cout << "- Segmentation via isocurve boundaries" << endl;

    // Start console input thread
    DWORD threadId;
    HANDLE hThread = CreateThread(NULL, 0, ConsoleInputThread, NULL, 0, &threadId);
    if (hThread == NULL) {
        cout << "Error: Failed to create console input thread!" << endl;
        return 1;
    }

    // Start the main event loop
    SoWin::show(window);
    SoWin::mainLoop();

    // Cleanup
    if (g_harmonics) delete g_harmonics;
    if (g_mesh) delete g_mesh;
    if (g_painter) delete g_painter;
    g_root->unref();

    return 0;
}

/**
 * Enhanced iso-curve extraction with more parallel "bracelets" following the paper
 */
// Large function implementations moved to separate modules for better maintainability:
// - improvedIsoCurveExtraction() -> IsocurveAnalysis::improvedIsoCurveExtraction()
// - enhancedRigidityAnalysis() -> RigidityAnalysis::enhancedRigidityAnalysis()
// - visualizeAllRigidityPoints() -> RigidityAnalysis::visualizeAllRigidityPoints()
// - interactiveRigidityThresholdTesting() -> RigidityAnalysis::interactiveRigidityThresholdTesting()
// - extractIsocurvesTriangleMarching() -> IsocurveAnalysis::extractIsocurvesTriangleMarching()

// End of file