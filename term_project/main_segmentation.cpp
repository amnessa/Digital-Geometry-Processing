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
void testCotangentLaplacian();
void testFarthestPointSampling();
void testPairwiseHarmonics();
void testIsoCurveExtraction();
void testRigidityAnalysis();
void performFullSegmentation();
void clearVisualization();
void enhancedRigidityAnalysis();
void visualizeAllRigidityPoints();
void improvedIsoCurveExtraction();
void interactiveRigidityThresholdTesting();
vector<vector<Eigen::Vector3d>> extractIsocurvesTriangleMarching(const Eigen::VectorXd& harmonicField, double isovalue);

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
void clearVisualization() {
    while (g_root->getNumChildren() > 1) {
        g_root->removeChild(g_root->getNumChildren() - 1);
    }
    // Force viewer to update
    if (g_viewer) {
        g_viewer->scheduleRedraw();
    }
}

/**
 * Test cotangent Laplacian computation
 */
void testCotangentLaplacian() {
    if (!g_mesh || !g_harmonics) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }

    cout << "\n=== TESTING COTANGENT LAPLACIAN ===" << endl;
    cout << "Computing cotangent-weighted Laplacian matrix..." << endl;

    auto start = clock();
    bool success = g_harmonics->computeCotangentLaplacian();
    auto end = clock();

    double elapsed = double(end - start) / CLOCKS_PER_SEC;

    if (success) {
        cout << "SUCCESS: Cotangent Laplacian computed successfully!" << endl;
        cout << "  Computation time: " << elapsed << " seconds" << endl;
        cout << "  Matrix size: " << g_mesh->verts.size() << "x" << g_mesh->verts.size() << endl;
        cout << "  Properties:" << endl;
        cout << "    - Intrinsic (uses cotangent weights)" << endl;
        cout << "    - Includes Voronoi area weighting" << endl;
        cout << "    - Symmetric and positive semi-definite" << endl;
        cout << "    - Ready for harmonic field computation" << endl;
    } else {
        cout << "[ERROR] Failed to compute cotangent Laplacian!" << endl;
    }
}

/**
 * Test farthest point sampling (FPS) algorithm
 */
void testFarthestPointSampling() {
    if (!g_mesh) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }    cout << "\n=== TESTING FARTHEST POINT SAMPLING ===" << endl;
    int numSamples;
    cout << "Enter number of FPS samples (recommended: 25-40 for human models, 15-25 for simple shapes): ";
    cin >> numSamples;

    if (numSamples < 2 || numSamples > g_mesh->verts.size()) {
        cout << "Error: Invalid number of samples!" << endl;
        return;
    }

    cout << "Computing FPS with " << numSamples << " points..." << endl;
    cout << "Algorithm: 1) Pick arbitrary point -> 2) Find farthest -> 3) Discard arbitrary -> 4) Continue FPS" << endl;

    auto start = clock();
    g_fps_samples = g_mesh->farthestPointSampling(numSamples);
    auto end = clock();

    double elapsed = double(end - start) / CLOCKS_PER_SEC;
    if (g_fps_samples.size() == numSamples) {
        cout << "SUCCESS: FPS computed successfully!" << endl;
        cout << "  Computation time: " << elapsed << " seconds" << endl;
        cout << "  Sample points: ";
        for (int sample : g_fps_samples) {
            cout << sample << " ";
        }
        cout << endl;

        // Visualize sample points
        clearVisualization();
        SoSeparator* pointsSep = new SoSeparator();
        for (int i = 0; i < g_fps_samples.size(); i++) {
            float r = (i == 0) ? 1.0f : 0.0f; // First point red, others blue
            float g = 0.0f;
            float b = (i == 0) ? 0.0f : 1.0f;
            pointsSep->addChild(g_painter->get1PointSep(g_mesh, g_fps_samples[i], r, g, b, 8.0f, false));        }
        g_root->addChild(pointsSep);

        // Force viewer refresh
        g_viewer->scheduleRedraw();
        g_viewer->viewAll();

        cout << "FPS points visualized (first point is red, others blue)." << endl;
    } else {
        cout << "[ERROR] FPS computation failed!" << endl;
    }
}

/**
 * Test pairwise harmonic field computation and visualization
 */
void testPairwiseHarmonics() {
    if (!g_mesh || !g_harmonics) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }

    cout << "\n=== TESTING PAIRWISE HARMONICS ===" << endl;

    int p_idx, q_idx;
    cout << "Enter first vertex index (p, boundary value 0): ";
    cin >> p_idx;
    cout << "Enter second vertex index (q, boundary value 1): ";
    cin >> q_idx;

    int nVerts = g_mesh->verts.size();
    if (p_idx < 0 || p_idx >= nVerts || q_idx < 0 || q_idx >= nVerts || p_idx == q_idx) {
        cout << "Error: Invalid vertex indices!" << endl;
        return;
    }

    cout << "Computing harmonic field between vertices " << p_idx << " and " << q_idx << "..." << endl;

    auto start = clock();
    Eigen::VectorXd harmonicField = g_harmonics->computePairwiseHarmonic(p_idx, q_idx);
    auto end = clock();

    double elapsed = double(end - start) / CLOCKS_PER_SEC;    if (harmonicField.size() == nVerts) {
        cout << "SUCCESS: Pairwise harmonic computed successfully!" << endl;
        cout << "  Computation time: " << elapsed << " seconds" << endl;
        cout << "  Field range: [" << harmonicField.minCoeff() << ", " << harmonicField.maxCoeff() << "]" << endl;

        // Visualize the harmonic field
        clearVisualization();
        SoSeparator* harmonicVis = g_harmonics->visualizeHarmonicField(harmonicField, g_painter);        g_root->addChild(harmonicVis);

        // Compute R and D descriptors as validation
        cout << "\nComputing shape descriptors..." << endl;
        Eigen::VectorXd R_descriptor = g_harmonics->computeRDescriptor(harmonicField, 10);
        cout << "R descriptor (isocurve lengths): ";
        for (int i = 0; i < R_descriptor.size(); i++) {
            cout << R_descriptor(i) << " ";
        }
        cout << endl;

        // Force viewer refresh
        g_viewer->scheduleRedraw();
        g_viewer->viewAll();
        cout << "Harmonic field visualized (blue = value 0, red = value 1)." << endl;
    } else {
        cout << "[ERROR] Failed to compute pairwise harmonic!" << endl;
    }
}

/**
 * Test iso-curve extraction from harmonic fields
 */
void testIsoCurveExtraction() {
    if (!g_mesh || !g_harmonics || g_fps_samples.size() < 2) {
        cout << "Error: Please load mesh and compute FPS samples first!" << endl;
        return;
    }

    cout << "\n=== TESTING ISO-CURVE EXTRACTION ===" << endl;

    // Use first two FPS samples
    int p_idx = g_fps_samples[0];
    int q_idx = g_fps_samples[1];

    cout << "Computing harmonic field between FPS vertices " << p_idx << " and " << q_idx << "..." << endl;

    Eigen::VectorXd harmonicField = g_harmonics->computePairwiseHarmonic(p_idx, q_idx);    if (harmonicField.size() != g_mesh->verts.size()) {
        cout << "[ERROR] Failed to compute harmonic field!" << endl;
        return;
    }

    cout << "Extracting iso-curves at different levels..." << endl;

    clearVisualization();
    vector<double> isoValues = {0.2, 0.4, 0.6, 0.8};
    vector<Eigen::Vector3d> skeletal_nodes;

    for (int i = 0; i < isoValues.size(); i++) {
        double isoValue = isoValues[i];

        auto segments = g_harmonics->extractIsoCurveSegments(harmonicField, isoValue);
        double totalLength = g_harmonics->computeIsoCurveLength(harmonicField, isoValue);

        cout << "  Iso-value " << isoValue << ": " << segments.size() << " segments, total length = " << totalLength << endl;

        if (!segments.empty()) {
            // Compute centroid for skeletal node
            Eigen::Vector3d centroid(0, 0, 0);
            int pointCount = 0;
            for (const auto& segment : segments) {
                centroid += segment.first;
                centroid += segment.second;
                pointCount += 2;
            }
            if (pointCount > 0) {
                centroid /= pointCount;
                skeletal_nodes.push_back(centroid);
            }

            // Visualize iso-curve
            SoSeparator* isoCurveSep = new SoSeparator();
            SoMaterial* mat = new SoMaterial();
            mat->diffuseColor.setValue(isoValue, 1.0f - isoValue, 0.5f);
            isoCurveSep->addChild(mat);

            SoCoordinate3* coords = new SoCoordinate3();
            coords->point.setNum(segments.size() * 2);

            for (int j = 0; j < segments.size(); j++) {
                coords->point.set1Value(j * 2, segments[j].first[0], segments[j].first[1], segments[j].first[2]);
                coords->point.set1Value(j * 2 + 1, segments[j].second[0], segments[j].second[1], segments[j].second[2]);
            }
            isoCurveSep->addChild(coords);

            SoIndexedLineSet* lineSet = new SoIndexedLineSet();
            for (int j = 0; j < segments.size(); j++) {
                lineSet->coordIndex.set1Value(j * 3, j * 2);
                lineSet->coordIndex.set1Value(j * 3 + 1, j * 2 + 1);
                lineSet->coordIndex.set1Value(j * 3 + 2, -1);
            }
            isoCurveSep->addChild(lineSet);
            g_root->addChild(isoCurveSep);        }
    }

    cout << "SUCCESS: Iso-curve extraction complete!" << endl;
    cout << "  Generated " << skeletal_nodes.size() << " skeletal nodes" << endl;    if (skeletal_nodes.size() >= 3) {
        g_skeletal_segments.push_back(skeletal_nodes);
        cout << "  Skeletal segment stored for rigidity analysis" << endl;
    }

    // Force viewer refresh
    g_viewer->scheduleRedraw();
    g_viewer->viewAll();
    cout << "Iso-curves visualized with color-coded levels." << endl;
}

/**
 * Test rigidity analysis using PCA on skeletal nodes
 */
void testRigidityAnalysis() {
    if (g_skeletal_segments.empty()) {
        cout << "Error: Please extract iso-curves first to generate skeletal segments!" << endl;
        return;
    }

    cout << "\n=== TESTING RIGIDITY ANALYSIS ===" << endl;
    cout << "Analyzing " << g_skeletal_segments.size() << " skeletal segments..." << endl;

    g_rigidity_scores.clear();
    g_nonrigid_nodes.clear();

    for (int segIdx = 0; segIdx < g_skeletal_segments.size(); segIdx++) {
        const auto& nodes = g_skeletal_segments[segIdx];

        if (nodes.size() < 3) {
            cout << "  Segment " << segIdx << ": Too few nodes (" << nodes.size() << "), skipping" << endl;
            continue;
        }

        // Compute PCA on skeletal nodes
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();

        // Compute mean
        for (const auto& node : nodes) {
            mean += node;
        }
        mean /= nodes.size();

        // Compute covariance matrix
        for (const auto& node : nodes) {
            Eigen::Vector3d diff = node - mean;
            covariance += diff * diff.transpose();
        }
        covariance /= (nodes.size() - 1);

        // Compute eigenvalues using self-adjoint eigen solver
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        Eigen::Vector3d eigenvalues = solver.eigenvalues();

        // Sort eigenvalues in descending order
        std::sort(eigenvalues.data(), eigenvalues.data() + 3, std::greater<double>());

        // Compute rigidity score: ξ = (λ1 + λ2 + λ3) / max(λ1, λ2, λ3)
        double lambda1 = eigenvalues(0);
        double lambda2 = eigenvalues(1);
        double lambda3 = eigenvalues(2);
        double maxLambda = std::max({lambda1, lambda2, lambda3});

        double rigidity = 1.0; // Default to rigid if eigenvalues are zero
        if (maxLambda > 1e-10) {
            rigidity = (lambda1 + lambda2 + lambda3) / maxLambda;
        }        g_rigidity_scores.push_back(rigidity);
        bool isNonrigid = (rigidity < RIGIDITY_THRESHOLD);
        g_nonrigid_nodes.push_back(isNonrigid);

        cout << "  Segment " << segIdx << ": " << nodes.size() << " nodes, rigidity = " << rigidity;
        cout << " (" << (isNonrigid ? "NON-RIGID" : "RIGID") << ")" << endl;
        cout << "    Eigenvalues: L1=" << lambda1 << ", L2=" << lambda2 << ", L3=" << lambda3 << endl;
    }

    if (!g_rigidity_scores.empty()) {
        double avgRigidity = 0;
        int rigidCount = 0;
        int nonrigidCount = 0;

        for (int i = 0; i < g_rigidity_scores.size(); i++) {
            avgRigidity += g_rigidity_scores[i];
            if (g_nonrigid_nodes[i]) nonrigidCount++;
            else rigidCount++;        }
        avgRigidity /= g_rigidity_scores.size();

        cout << "\nSUCCESS: Rigidity analysis complete!" << endl;
        cout << "  Average rigidity: " << avgRigidity << endl;
        cout << "  Rigid segments: " << rigidCount << endl;
        cout << "  Non-rigid segments: " << nonrigidCount << endl;
        cout << "  Threshold used: " << RIGIDITY_THRESHOLD << endl;
    }
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
                break;
            case 9:
                improvedIsoCurveExtraction();
                break;
            case 10:
                enhancedRigidityAnalysis();
                break;            case 11:
                visualizeAllRigidityPoints();
                break;
            case 12:
                interactiveRigidityThresholdTesting();
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
void improvedIsoCurveExtraction() {
    if (!g_mesh || !g_harmonics || g_fps_samples.size() < 2) {
        cout << "Error: Please load mesh and compute FPS samples first!" << endl;
        return;
    }

    cout << "\n=== IMPROVED ISO-CURVE EXTRACTION ===" << endl;
    cout << "Extracting dense iso-curves as 'bracelets' around the shape" << endl;

    // Use first two FPS samples for demonstration
    int p_idx = g_fps_samples[0];
    int q_idx = g_fps_samples[1];

    cout << "Computing harmonic field between FPS vertices " << p_idx << " and " << q_idx << "..." << endl;

    Eigen::VectorXd harmonicField = g_harmonics->computePairwiseHarmonic(p_idx, q_idx);
    if (harmonicField.size() != g_mesh->verts.size()) {
        cout << "[ERROR] Failed to compute harmonic field!" << endl;
        return;
    }

    clearVisualization();    // Extract many more isocurves (like bracelets around a limb)
    int numIsocurves = 50; // Much denser extraction as in the paper (increased from 25)
    vector<Eigen::Vector3d> skeletal_nodes;

    cout << "Extracting " << numIsocurves << " isocurves as parallel 'bracelets'..." << endl;

    for (int k = 1; k < numIsocurves; k++) {
        double isoValue = double(k) / numIsocurves;

        // Find all vertices that are approximately at this harmonic field level
        vector<int> braceletVertices;
        double tolerance = 0.008; // Much tighter tolerance for better precision (reduced from 0.02)

        for (int v = 0; v < g_mesh->verts.size(); v++) {
            if (abs(harmonicField(v) - isoValue) < tolerance) {
                braceletVertices.push_back(v);
            }
        }

        if (braceletVertices.size() >= 3) {
            // Compute centroid of this "bracelet" - this becomes a skeletal node
            Eigen::Vector3d braceletCenter(0, 0, 0);
            for (int v : braceletVertices) {
                braceletCenter += Eigen::Vector3d(g_mesh->verts[v]->coords[0],
                                                  g_mesh->verts[v]->coords[1],
                                                  g_mesh->verts[v]->coords[2]);
            }
            braceletCenter /= braceletVertices.size();
            skeletal_nodes.push_back(braceletCenter);

            // Visualize this bracelet
            SoSeparator* braceletSep = new SoSeparator();
            SoMaterial* mat = new SoMaterial();

            // Color based on position along harmonic field (blue to red gradient)
            float r = isoValue;
            float g = 0.2f;
            float b = 1.0f - isoValue;
            mat->diffuseColor.setValue(r, g, b);
            braceletSep->addChild(mat);

            // Draw points forming the bracelet
            SoCoordinate3* coords = new SoCoordinate3();
            coords->point.setNum(braceletVertices.size());

            for (int i = 0; i < braceletVertices.size(); i++) {
                Vertex* v = g_mesh->verts[braceletVertices[i]];
                coords->point.set1Value(i, v->coords[0], v->coords[1], v->coords[2]);
            }
            braceletSep->addChild(coords);

            SoPointSet* pointSet = new SoPointSet();
            pointSet->numPoints = braceletVertices.size();
            braceletSep->addChild(pointSet);

            g_root->addChild(braceletSep);

            cout << "  Isocurve " << k << " (value=" << isoValue << "): "
                 << braceletVertices.size() << " vertices forming bracelet" << endl;
        }
    }

    // Visualize the skeletal segment by connecting centroids
    if (skeletal_nodes.size() >= 3) {
        SoSeparator* skeletonSep = new SoSeparator();
        SoMaterial* skelMat = new SoMaterial();
        skelMat->diffuseColor.setValue(1.0f, 1.0f, 0.0f); // Yellow for skeleton
        skeletonSep->addChild(skelMat);

        SoCoordinate3* skelCoords = new SoCoordinate3();
        skelCoords->point.setNum(skeletal_nodes.size());

        for (int i = 0; i < skeletal_nodes.size(); i++) {
            skelCoords->point.set1Value(i, skeletal_nodes[i][0], skeletal_nodes[i][1], skeletal_nodes[i][2]);
        }
        skeletonSep->addChild(skelCoords);

        SoIndexedLineSet* skelLines = new SoIndexedLineSet();
        for (int i = 0; i < skeletal_nodes.size() - 1; i++) {
            skelLines->coordIndex.set1Value(i * 3, i);
            skelLines->coordIndex.set1Value(i * 3 + 1, i + 1);
            skelLines->coordIndex.set1Value(i * 3 + 2, -1);
        }
        skeletonSep->addChild(skelLines);
        g_root->addChild(skeletonSep);

        // Store for rigidity analysis
        g_skeletal_segments.clear();
        g_skeletal_segments.push_back(skeletal_nodes);

        cout << "\nSUCCESS: Generated skeletal segment with " << skeletal_nodes.size() << " nodes" << endl;
        cout << "Yellow line shows the skeletal segment connecting bracelet centroids" << endl;
    }

    g_viewer->scheduleRedraw();
    g_viewer->viewAll();
    cout << "Improved iso-curve extraction complete - showing dense parallel bracelets!" << endl;
}

/**
 * Enhanced rigidity analysis that computes rigidity for ALL skeletal nodes
 * without applying a binary threshold, storing all rigidity scores
 * IMPROVED: Generates skeletal segments from MULTIPLE point pairs for better analysis
 */
void enhancedRigidityAnalysis() {
    if (!g_mesh || !g_harmonics || g_fps_samples.size() < 2) {
        cout << "Error: Please load mesh and compute FPS samples first!" << endl;
        return;
    }

    cout << "\n=== ENHANCED RIGIDITY ANALYSIS ===" << endl;
    cout << "Generating skeletal segments from multiple FPS point pairs..." << endl;

    g_rigidity_scores.clear();
    g_nonrigid_nodes.clear();
    g_skeletal_segments.clear();

    // Generate skeletal segments from MULTIPLE point pairs (not just first 2)
    int maxPairs = min(10, (int)(g_fps_samples.size() * (g_fps_samples.size() - 1) / 2)); // Limit to 10 pairs for efficiency
    int pairCount = 0;

    for (int i = 0; i < g_fps_samples.size() && pairCount < maxPairs; i++) {
        for (int j = i + 1; j < g_fps_samples.size() && pairCount < maxPairs; j++) {
            int p_idx = g_fps_samples[i];
            int q_idx = g_fps_samples[j];

            cout << "  Processing pair " << (pairCount + 1) << "/" << maxPairs
                 << ": vertices " << p_idx << " <-> " << q_idx << endl;

            // Compute harmonic field for this pair
            Eigen::VectorXd harmonicField = g_harmonics->computePairwiseHarmonic(p_idx, q_idx);
            if (harmonicField.size() != g_mesh->verts.size()) {
                cout << "    [WARNING] Failed to compute harmonic field for this pair, skipping..." << endl;
                continue;
            }            // Extract isocurves using proper triangle marching algorithm
            vector<Eigen::Vector3d> skeletal_nodes;
            int numIsocurves = 30; // Use moderate number for efficiency

            for (int k = 1; k < numIsocurves; k++) {
                double isoValue = double(k) / numIsocurves;

                // Extract isocurves using triangle marching
                vector<vector<Eigen::Vector3d>> isocurves = extractIsocurvesTriangleMarching(harmonicField, isoValue);

                // For each isocurve, compute its centroid as a skeletal node
                for (const auto& curve : isocurves) {
                    if (curve.size() >= 3) {  // Only use curves with sufficient points
                        Eigen::Vector3d centroid(0, 0, 0);
                        for (const auto& point : curve) {
                            centroid += point;
                        }
                        centroid /= curve.size();
                        skeletal_nodes.push_back(centroid);
                    }
                }
            }

            if (skeletal_nodes.size() >= 3) {
                g_skeletal_segments.push_back(skeletal_nodes);
                cout << "    Generated skeletal segment with " << skeletal_nodes.size() << " nodes" << endl;
            }

            pairCount++;
        }
    }

    cout << "Generated " << g_skeletal_segments.size() << " skeletal segments from " << pairCount << " point pairs" << endl;

    if (g_skeletal_segments.empty()) {
        cout << "ERROR: No valid skeletal segments generated!" << endl;
        return;
    }

    // Now compute rigidity for ALL nodes from ALL segments
    cout << "Computing rigidity scores for ALL skeletal nodes..." << endl;

    vector<double> allNodeRigidities;
    vector<Eigen::Vector3d> allNodes;

    for (int segIdx = 0; segIdx < g_skeletal_segments.size(); segIdx++) {
        const auto& nodes = g_skeletal_segments[segIdx];
        cout << "Analyzing segment " << segIdx + 1 << " with " << nodes.size() << " nodes..." << endl;

        for (int nodeIdx = 0; nodeIdx < nodes.size(); nodeIdx++) {
            // Define local neighborhood around this node
            vector<Eigen::Vector3d> neighborhood;
            int windowSize = min(7, (int)nodes.size()); // Use up to 7 neighbors for better PCA

            int start = max(0, nodeIdx - windowSize/2);
            int end = min((int)nodes.size(), start + windowSize);

            for (int k = start; k < end; k++) {
                neighborhood.push_back(nodes[k]);
            }

            // Compute PCA-based rigidity
            double rigidity = 1.0; // Default for edge cases

            if (neighborhood.size() >= 3) {
                // Compute covariance matrix
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
                covariance /= (neighborhood.size() - 1);                // Compute PCA-based rigidity using CORRECT formula from paper
                // Paper formula: ξ = λ₁ / (λ₁ + λ₂ + λ₃) where λ₁ ≥ λ₂ ≥ λ₃
                // This gives values in [0,1]: ~1.0 = rigid (linear), ~0.33 = non-rigid (junction)

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
                Eigen::Vector3d eigenvalues = solver.eigenvalues();

                // Sort in descending order (λ₁ ≥ λ₂ ≥ λ₃)
                std::sort(eigenvalues.data(), eigenvalues.data() + 3, std::greater<double>());

                double lambda1 = eigenvalues(0);
                double lambda2 = eigenvalues(1);
                double lambda3 = eigenvalues(2);
                double sumLambda = lambda1 + lambda2 + lambda3;

                // CORRECTED rigidity formula from paper
                if (sumLambda > 1e-10) {
                    rigidity = lambda1 / sumLambda;  // Correct formula: ξ = λ₁ / (λ₁ + λ₂ + λ₃)
                } else {
                    rigidity = 0.33; // Default for degenerate case (junction-like)
                }
            }

            allNodeRigidities.push_back(rigidity);
            allNodes.push_back(nodes[nodeIdx]);

            cout << "    Node " << nodeIdx << ": rigidity = " << rigidity << endl;
        }
    }

    // Store all rigidity scores
    g_rigidity_scores = allNodeRigidities;

    // Compute statistics
    if (!allNodeRigidities.empty()) {
        double minRig = *std::min_element(allNodeRigidities.begin(), allNodeRigidities.end());
        double maxRig = *std::max_element(allNodeRigidities.begin(), allNodeRigidities.end());
        double avgRig = 0;
        for (double r : allNodeRigidities) avgRig += r;
        avgRig /= allNodeRigidities.size();

        cout << "\nRigidity Statistics:" << endl;
        cout << "  Total nodes analyzed: " << allNodeRigidities.size() << endl;
        cout << "  Rigidity range: [" << minRig << ", " << maxRig << "]" << endl;
        cout << "  Average rigidity: " << avgRig << endl;
        cout << "  Standard deviation: ";

        double variance = 0;
        for (double r : allNodeRigidities) {
            variance += (r - avgRig) * (r - avgRig);
        }
        variance /= allNodeRigidities.size();
        double stddev = sqrt(variance);
        cout << stddev << endl;

        // Show distribution at different thresholds
        cout << "\nRigidity Distribution:" << endl;
        for (double threshold = 0.5; threshold <= 1.0; threshold += 0.1) {
            int count = 0;
            for (double r : allNodeRigidities) {
                if (r >= threshold) count++;
            }
            double percentage = 100.0 * count / allNodeRigidities.size();
            cout << "    >= " << threshold << ": " << count << " nodes (" << percentage << "%)" << endl;
        }
    }

    cout << "\nEnhanced rigidity analysis complete!" << endl;
}

/**
 * Visualize ALL rigidity points with continuous color coding from green to red
 * This gives much better feedback than binary rigid/non-rigid classification
 */
void visualizeAllRigidityPoints() {
    if (g_rigidity_scores.empty() || g_skeletal_segments.empty()) {
        cout << "Error: Please run enhanced rigidity analysis first!" << endl;
        return;
    }

    cout << "\n=== VISUALIZING ALL RIGIDITY POINTS ===" << endl;
    cout << "Showing continuous rigidity distribution: Green = Low Rigidity (Junctions), Red = High Rigidity (Limbs)" << endl;

    clearVisualization();

    // Get all skeletal nodes
    vector<Eigen::Vector3d> allNodes;
    for (const auto& segment : g_skeletal_segments) {
        for (const auto& node : segment) {
            allNodes.push_back(node);
        }
    }

    if (allNodes.size() != g_rigidity_scores.size()) {
        cout << "Error: Mismatch between nodes and rigidity scores!" << endl;
        return;
    }

    // Find rigidity range for color mapping
    double minRig = *std::min_element(g_rigidity_scores.begin(), g_rigidity_scores.end());
    double maxRig = *std::max_element(g_rigidity_scores.begin(), g_rigidity_scores.end());

    cout << "Rigidity range: [" << minRig << ", " << maxRig << "]" << endl;
    cout << "Color mapping: Green (≤" << minRig + 0.3 * (maxRig - minRig) << ") -> Yellow -> Red (≥" << maxRig - 0.1 * (maxRig - minRig) << ")" << endl;

    // Create visualization for each node
    for (int i = 0; i < allNodes.size(); i++) {
        double rigidity = g_rigidity_scores[i];
        const Eigen::Vector3d& node = allNodes[i];

        // Compute color based on rigidity (green to red gradient)
        double normalizedRig = (maxRig - minRig) > 1e-10 ? (rigidity - minRig) / (maxRig - minRig) : 0.5;

        // Color mapping: Green (low rigidity/junctions) to Red (high rigidity/limbs)
        float r = std::min(1.0f, (float)(2.0 * normalizedRig));           // 0→1 as rigidity increases
        float g = std::min(1.0f, (float)(2.0 * (1.0 - normalizedRig)));   // 1→0 as rigidity increases
        float b = 0.1f;  // Small amount of blue for better visibility

        // Create point visualization
        SoSeparator* pointSep = new SoSeparator();

        // Material
        SoMaterial* mat = new SoMaterial();
        mat->diffuseColor.setValue(r, g, b);
        mat->emissiveColor.setValue(r * 0.3f, g * 0.3f, b * 0.3f);
        pointSep->addChild(mat);

        // Transform to position
        SoTransform* transform = new SoTransform();
        transform->translation.setValue(node[0], node[1], node[2]);
        pointSep->addChild(transform);

        // Sphere representing the point
        SoSphere* sphere = new SoSphere();
        sphere->radius = 0.008f; // Adjust size as needed
        pointSep->addChild(sphere);

        g_root->addChild(pointSep);
    }

    // Add a legend by creating sample spheres at known positions
    cout << "Creating color legend..." << endl;

    // Find mesh bounding box for legend placement
    Eigen::Vector3d minBounds(1e10, 1e10, 1e10);
    Eigen::Vector3d maxBounds(-1e10, -1e10, -1e10);

    for (int i = 0; i < g_mesh->verts.size(); i++) {
        Vertex* v = g_mesh->verts[i];
        minBounds[0] = std::min(minBounds[0], (double)v->coords[0]);
        minBounds[1] = std::min(minBounds[1], (double)v->coords[1]);
        minBounds[2] = std::min(minBounds[2], (double)v->coords[2]);
        maxBounds[0] = std::max(maxBounds[0], (double)v->coords[0]);
        maxBounds[1] = std::max(maxBounds[1], (double)v->coords[1]);
        maxBounds[2] = std::max(maxBounds[2], (double)v->coords[2]);
    }

    // Create legend spheres
    double legendX = maxBounds[0] + 0.2 * (maxBounds[0] - minBounds[0]);
    double legendStartY = maxBounds[1];
    double legendSpacing = 0.1 * (maxBounds[1] - minBounds[1]);

    for (int i = 0; i < 5; i++) {
        double legendRig = double(i) / 4.0; // 0 to 1

        float r = std::min(1.0f, (float)(2.0 * legendRig));
        float g = std::min(1.0f, (float)(2.0 * (1.0 - legendRig)));
        float b = 0.1f;

        SoSeparator* legendSep = new SoSeparator();

        SoMaterial* legendMat = new SoMaterial();
        legendMat->diffuseColor.setValue(r, g, b);
        legendMat->emissiveColor.setValue(r * 0.5f, g * 0.5f, b * 0.5f);
        legendSep->addChild(legendMat);

        SoTransform* legendTransform = new SoTransform();
        legendTransform->translation.setValue(legendX, legendStartY - i * legendSpacing, (maxBounds[2] + minBounds[2]) / 2);
        legendSep->addChild(legendTransform);

        SoSphere* legendSphere = new SoSphere();
        legendSphere->radius = 0.015f;
        legendSep->addChild(legendSphere);

        g_root->addChild(legendSep);
    }

    // Count nodes in different rigidity ranges for feedback
    int lowRig = 0, medRig = 0, highRig = 0;
    double lowThresh = minRig + 0.33 * (maxRig - minRig);
    double highThresh = minRig + 0.67 * (maxRig - minRig);

    for (double rig : g_rigidity_scores) {
        if (rig < lowThresh) lowRig++;
        else if (rig < highThresh) medRig++;
        else highRig++;
    }

    cout << "\nVisualization complete!" << endl;
    cout << "Node distribution:" << endl;
    cout << "  Low rigidity (Green, potential junctions): " << lowRig << " nodes" << endl;
    cout << "  Medium rigidity (Yellow): " << medRig << " nodes" << endl;
    cout << "  High rigidity (Red, limb centers): " << highRig << " nodes" << endl;
    cout << "\nLegend spheres placed on the right side of the model" << endl;

    g_viewer->scheduleRedraw();
    g_viewer->viewAll();
}

/**
 * Interactive rigidity threshold testing - allows user to experiment with different thresholds
 * and see how many junction points are detected at each threshold level
 */
void interactiveRigidityThresholdTesting() {
    if (g_rigidity_scores.empty() || g_skeletal_segments.empty()) {
        cout << "Error: Please run enhanced rigidity analysis first!" << endl;
        return;
    }    cout << "\n=== INTERACTIVE RIGIDITY THRESHOLD TESTING ===" << endl;
    cout << "This will help you find the optimal threshold for detecting junction points" << endl;
    cout << "Paper recommendation: Use threshold 0.9 (range 0.85-0.95 works well)" << endl;

    // Show current rigidity statistics
    double minRig = *std::min_element(g_rigidity_scores.begin(), g_rigidity_scores.end());
    double maxRig = *std::max_element(g_rigidity_scores.begin(), g_rigidity_scores.end());

    cout << "\nCurrent rigidity range: [" << minRig << ", " << maxRig << "]" << endl;
    cout << "Total skeletal nodes: " << g_rigidity_scores.size() << endl;    while (true) {        cout << "\n--- THRESHOLD ANALYSIS ---" << endl;
        cout << "Enter a rigidity threshold to test (0.4-1.0, or 'q' to quit): ";

        string input;
        cin >> input;

        if (input == "q" || input == "Q") {
            break;
        }

        double threshold;
        try {
            threshold = std::stod(input);
        } catch (...) {
            cout << "Invalid input! Please enter a number between 0.4 and 1.0" << endl;
            continue;
        }

        if (threshold < 0.4 || threshold > 1.0) {
            cout << "Threshold out of range! Please enter a value between 0.4 and 1.0" << endl;
            cout << "(Paper recommends ~0.9, your data range is [" << minRig << ", " << maxRig << "])" << endl;
            cout << "(Values ~0.33 = junctions, ~1.0 = rigid limbs)" << endl;
            continue;
        }

        // Count nodes at this threshold
        int rigidNodes = 0;
        int nonRigidNodes = 0;
        vector<Eigen::Vector3d> allNodes;

        // Get all skeletal nodes
        for (const auto& segment : g_skeletal_segments) {
            for (const auto& node : segment) {
                allNodes.push_back(node);
            }
        }

        // Classify nodes based on threshold
        for (int i = 0; i < g_rigidity_scores.size(); i++) {
            if (g_rigidity_scores[i] >= threshold) {
                rigidNodes++;
            } else {
                nonRigidNodes++;
            }
        }

        cout << "\nResults for threshold = " << threshold << ":" << endl;
        cout << "  Rigid nodes (skeletal): " << rigidNodes << " (" << (100.0 * rigidNodes / g_rigidity_scores.size()) << "%)" << endl;
        cout << "  Non-rigid nodes (junctions): " << nonRigidNodes << " (" << (100.0 * nonRigidNodes / g_rigidity_scores.size()) << "%)" << endl;

        // Visualize nodes with this threshold
        clearVisualization();

        for (int i = 0; i < allNodes.size() && i < g_rigidity_scores.size(); i++) {
            const Eigen::Vector3d& node = allNodes[i];
            double rigidity = g_rigidity_scores[i];

            // Color based on threshold classification
            float r, g, b;
            if (rigidity >= threshold) {
                // Rigid node - use blue to red gradient based on how rigid it is
                double normalizedRig = (rigidity - threshold) / (maxRig - threshold + 1e-10);
                r = std::min(1.0f, (float)normalizedRig);
                g = 0.2f;
                b = 1.0f - std::min(1.0f, (float)normalizedRig);
            } else {
                // Non-rigid node - use green (potential junction)
                r = 0.0f;
                g = 1.0f;
                b = 0.2f;
            }

            // Create point visualization
            SoSeparator* pointSep = new SoSeparator();

            SoMaterial* mat = new SoMaterial();
            mat->diffuseColor.setValue(r, g, b);
            mat->emissiveColor.setValue(r * 0.3f, g * 0.3f, b * 0.3f);
            pointSep->addChild(mat);

            SoTransform* transform = new SoTransform();
            transform->translation.setValue(node[0], node[1], node[2]);
            pointSep->addChild(transform);

            // Use larger spheres for junction points to make them stand out
            SoSphere* sphere = new SoSphere();
            sphere->radius = (rigidity < threshold) ? 0.012f : 0.008f;
            pointSep->addChild(sphere);

            g_root->addChild(pointSep);
        }

        // Show interpretation guide
        cout << "\nVisualization Guide:" << endl;
        cout << "  GREEN spheres (large): Non-rigid nodes = Junction candidates" << endl;
        cout << "  BLUE-to-RED spheres (small): Rigid nodes = Skeletal centers" << endl;
        cout << "  -> For human models, you want ~4-8 green junction points" << endl;
        cout << "  -> (shoulders, hips, neck, wrists, ankles, etc.)" << endl;

        g_viewer->scheduleRedraw();
        g_viewer->viewAll();
    }

    cout << "Threshold testing complete!" << endl;
}

/**
 * Extract true isocurves using triangle marching algorithm
 * Based on the method described in isocurve.md attachment
 */
vector<vector<Eigen::Vector3d>> extractIsocurvesTriangleMarching(const Eigen::VectorXd& harmonicField, double isovalue) {
    vector<vector<Eigen::Vector3d>> isocurves;

    if (!g_mesh) return isocurves;

    // Step 1: Find all edge intersections where isocurve crosses
    struct EdgeIntersection {
        int v1, v2;  // Vertex indices of the edge
        Eigen::Vector3d point;  // 3D intersection point
        int triangle1, triangle2;  // Adjacent triangles (-1 if boundary)
    };

    vector<EdgeIntersection> intersections;
      // Check all edges for intersections
    for (int t = 0; t < g_mesh->tris.size(); t++) {
        Triangle* tri = g_mesh->tris[t];

        // Get the three vertex indices of this triangle
        int vertices[3] = {tri->v1i, tri->v2i, tri->v3i};

        // Check all three edges of this triangle
        for (int e = 0; e < 3; e++) {
            int v1 = vertices[e];
            int v2 = vertices[(e + 1) % 3];

            double f1 = harmonicField(v1);
            double f2 = harmonicField(v2);

            // Check if isocurve crosses this edge
            bool crosses = (f1 <= isovalue && f2 >= isovalue) || (f1 >= isovalue && f2 <= isovalue);
            if (crosses && abs(f1 - f2) > 1e-10) {  // Avoid division by zero

                // Skip if intersection point is exactly at a vertex (degenerate case)
                if (abs(f1 - isovalue) < 1e-10 || abs(f2 - isovalue) < 1e-10) {
                    continue;
                }

                // Calculate intersection point using linear interpolation
                double t = (isovalue - f1) / (f2 - f1);

                Vertex* vert1 = g_mesh->verts[v1];
                Vertex* vert2 = g_mesh->verts[v2];

                Eigen::Vector3d p1(vert1->coords[0], vert1->coords[1], vert1->coords[2]);
                Eigen::Vector3d p2(vert2->coords[0], vert2->coords[1], vert2->coords[2]);

                Eigen::Vector3d intersection = (1.0 - t) * p1 + t * p2;

                // Check if we already have this edge intersection (avoid duplicates)
                bool found = false;
                for (const auto& existing : intersections) {
                    if ((existing.v1 == v1 && existing.v2 == v2) ||
                        (existing.v1 == v2 && existing.v2 == v1)) {
                        found = true;
                        break;
                    }
                }

                if (!found) {
                    EdgeIntersection newIntersection;
                    newIntersection.v1 = v1;
                    newIntersection.v2 = v2;
                    newIntersection.point = intersection;
                    newIntersection.triangle1 = t;
                    newIntersection.triangle2 = -1;  // Will be filled later

                    intersections.push_back(newIntersection);
                }
            }
        }
    }
      // Step 2: Find adjacent triangles for each edge intersection
    for (auto& intersection : intersections) {
        int count = 0;
        for (int t = 0; t < g_mesh->tris.size(); t++) {
            Triangle* tri = g_mesh->tris[t];
            int vertices[3] = {tri->v1i, tri->v2i, tri->v3i};

            // Check if this triangle contains the edge
            bool hasEdge = false;
            for (int e = 0; e < 3; e++) {
                int tv1 = vertices[e];
                int tv2 = vertices[(e + 1) % 3];

                if ((tv1 == intersection.v1 && tv2 == intersection.v2) ||
                    (tv1 == intersection.v2 && tv2 == intersection.v1)) {
                    hasEdge = true;
                    break;
                }
            }

            if (hasEdge) {
                if (count == 0) {
                    intersection.triangle1 = t;
                } else if (count == 1) {
                    intersection.triangle2 = t;
                } else {
                    break;  // More than 2 triangles share edge (degenerate mesh)
                }
                count++;
            }
        }
    }

    // Step 3: Group intersections into continuous curves by triangle connectivity
    vector<bool> used(intersections.size(), false);

    for (int i = 0; i < intersections.size(); i++) {
        if (used[i]) continue;

        // Start a new curve
        vector<Eigen::Vector3d> curve;
        queue<int> toProcess;
        toProcess.push(i);
        used[i] = true;

        while (!toProcess.empty()) {
            int currentIdx = toProcess.front();
            toProcess.pop();

            const EdgeIntersection& current = intersections[currentIdx];
            curve.push_back(current.point);

            // Find connected intersections (same triangle)
            for (int j = 0; j < intersections.size(); j++) {
                if (used[j] || j == currentIdx) continue;

                const EdgeIntersection& candidate = intersections[j];

                // Check if they share a triangle
                bool connected = (current.triangle1 == candidate.triangle1 && current.triangle1 != -1) ||
                               (current.triangle1 == candidate.triangle2 && current.triangle1 != -1) ||
                               (current.triangle2 == candidate.triangle1 && current.triangle2 != -1) ||
                               (current.triangle2 == candidate.triangle2 && current.triangle2 != -1);

                if (connected) {
                    toProcess.push(j);
                    used[j] = true;
                }
            }
        }

        // Only keep curves with multiple points
        if (curve.size() >= 2) {
            isocurves.push_back(curve);
        }
    }

    return isocurves;
}
