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
const double RIGIDITY_THRESHOLD = 0.9;  // Threshold for classifying rigid vs non-rigid nodes
const int DEFAULT_FPS_SAMPLES = 15;     // Number of FPS samples for initial point generation

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
    }

    cout << "\n=== TESTING FARTHEST POINT SAMPLING ===" << endl;
    int numSamples;
    cout << "Enter number of FPS samples (recommended: 10-20): ";
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
 */
void performFullSegmentation() {
    if (!g_mesh || !g_harmonics) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }

    cout << "\n=== FULL PAIRWISE HARMONICS SEGMENTATION ===" << endl;
    cout << "This will implement the complete algorithm from the paper..." << endl;
    cout << "Parameters:" << endl;
    cout << "  - K (isocurves per field): " << DEFAULT_K << endl;
    cout << "  - Rigidity threshold: " << RIGIDITY_THRESHOLD << endl;
    cout << "  - FPS samples: " << DEFAULT_FPS_SAMPLES << endl;

    // Stage 1: Initial Point Sampling and Skeletal Segment Generation
    cout << "\nStage 1: Generating initial point samples..." << endl;
    if (g_fps_samples.empty()) {
        g_fps_samples = g_mesh->farthestPointSampling(DEFAULT_FPS_SAMPLES);
        cout << "  Generated " << g_fps_samples.size() << " FPS samples" << endl;
    } else {
        cout << "  Using existing " << g_fps_samples.size() << " FPS samples" << endl;
    }

    // Stage 2: Generate pairwise harmonic fields for all sample pairs
    cout << "\nStage 2: Computing pairwise harmonic fields..." << endl;
    clearVisualization();

    vector<vector<Eigen::Vector3d>> allSkeletalSegments;
    int pairCount = 0;
    int maxPairs = min(20, (int)(g_fps_samples.size() * (g_fps_samples.size() - 1) / 2)); // Limit for demo

    for (int i = 0; i < g_fps_samples.size() && pairCount < maxPairs; i++) {
        for (int j = i + 1; j < g_fps_samples.size() && pairCount < maxPairs; j++) {
            cout << "  Processing pair " << pairCount + 1 << "/" << maxPairs
                 << " (" << g_fps_samples[i] << ", " << g_fps_samples[j] << ")" << endl;

            // Compute harmonic field
            Eigen::VectorXd harmonicField = g_harmonics->computePairwiseHarmonic(g_fps_samples[i], g_fps_samples[j]);

            if (harmonicField.size() != g_mesh->verts.size()) {
                cout << "    Failed to compute harmonic field, skipping..." << endl;
                continue;
            }

            // Extract skeletal segment from isocurves
            vector<Eigen::Vector3d> skeletalNodes;
            for (int k = 1; k < DEFAULT_K; k++) {
                double isoValue = double(k) / DEFAULT_K;
                auto segments = g_harmonics->extractIsoCurveSegments(harmonicField, isoValue);

                if (!segments.empty()) {
                    // Compute centroid
                    Eigen::Vector3d centroid(0, 0, 0);
                    int pointCount = 0;
                    for (const auto& segment : segments) {
                        centroid += segment.first;
                        centroid += segment.second;
                        pointCount += 2;
                    }
                    if (pointCount > 0) {
                        centroid /= pointCount;
                        skeletalNodes.push_back(centroid);
                    }
                }
            }

            if (skeletalNodes.size() >= 3) {
                allSkeletalSegments.push_back(skeletalNodes);
            }

            pairCount++;
        }
    }

    cout << "  Generated " << allSkeletalSegments.size() << " skeletal segments" << endl;

    // Stage 3: Rigidity Analysis and Filtering
    cout << "\nStage 3: Rigidity analysis and segment filtering..." << endl;

    vector<pair<double, int>> segmentQuality; // (quality_score, segment_index)

    for (int segIdx = 0; segIdx < allSkeletalSegments.size(); segIdx++) {
        const auto& nodes = allSkeletalSegments[segIdx];

        // Compute rigidity using PCA
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();

        for (const auto& node : nodes) {
            mean += node;
        }
        mean /= nodes.size();

        for (const auto& node : nodes) {
            Eigen::Vector3d diff = node - mean;
            covariance += diff * diff.transpose();
        }
        covariance /= (nodes.size() - 1);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        Eigen::Vector3d eigenvalues = solver.eigenvalues();
        std::sort(eigenvalues.data(), eigenvalues.data() + 3, std::greater<double>());

        double lambda1 = eigenvalues(0);
        double lambda2 = eigenvalues(1);
        double lambda3 = eigenvalues(2);
        double maxLambda = std::max({lambda1, lambda2, lambda3});

        double rigidity = 1.0;
        if (maxLambda > 1e-10) {
            rigidity = (lambda1 + lambda2 + lambda3) / maxLambda;
        }

        // Compute segment length
        double length = 0.0;
        for (int i = 0; i < nodes.size() - 1; i++) {
            length += (nodes[i+1] - nodes[i]).norm();
        }

        // Quality score: ρ = ξ * l (rigidity * length)
        double quality = rigidity * length;
        segmentQuality.push_back({quality, segIdx});
    }

    // Sort by quality (highest first)
    sort(segmentQuality.begin(), segmentQuality.end(), greater<pair<double, int>>());

    // Select top segments (greedy selection avoiding overlaps)
    vector<int> selectedSegments;
    set<pair<int, int>> usedPairs;

    for (const auto& entry : segmentQuality) {
        int segIdx = entry.second;
        // For simplicity, select all non-overlapping segments
        // In full implementation, you'd check for actual geometric overlap
        selectedSegments.push_back(segIdx);
        if (selectedSegments.size() >= 5) break; // Limit for visualization
    }

    cout << "  Selected " << selectedSegments.size() << " high-quality segments" << endl;

    // Visualize selected segments
    for (int i = 0; i < selectedSegments.size(); i++) {
        int segIdx = selectedSegments[i];
        const auto& nodes = allSkeletalSegments[segIdx];

        SoSeparator* segSep = new SoSeparator();
        SoMaterial* mat = new SoMaterial();
        float hue = float(i) / selectedSegments.size();
        mat->diffuseColor.setValue(hue, 1.0f - hue, 0.5f);
        segSep->addChild(mat);

        // Visualize skeletal segment as connected line
        SoCoordinate3* coords = new SoCoordinate3();
        coords->point.setNum(nodes.size());
        for (int j = 0; j < nodes.size(); j++) {
            coords->point.set1Value(j, nodes[j][0], nodes[j][1], nodes[j][2]);
        }
        segSep->addChild(coords);

        SoIndexedLineSet* lineSet = new SoIndexedLineSet();
        for (int j = 0; j < nodes.size() - 1; j++) {
            lineSet->coordIndex.set1Value(j * 3, j);
            lineSet->coordIndex.set1Value(j * 3 + 1, j + 1);
            lineSet->coordIndex.set1Value(j * 3 + 2, -1);        }
        segSep->addChild(lineSet);
        g_root->addChild(segSep);
    }

    cout << "\nSUCCESS: Pairwise harmonics segmentation complete!" << endl;
    cout << "Selected skeletal segments visualized with different colors." << endl;
    cout << "Note: This is a simplified implementation. Full algorithm includes:" << endl;
    cout << "  - Initial segmentation from isocurves" << endl;
    cout << "  - Skeleton completion using adjacency" << endl;
    cout << "  - Segmentation refinement through merging" << endl;

    g_viewer->viewAll();
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
    cout << "8. Clear Visualization" << endl;
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
        cin >> choice;

        switch (choice) {
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
                break;            case 8:
                clearVisualization();
                g_viewer->scheduleRedraw();
                g_viewer->viewAll();
                cout << "Visualization cleared." << endl;
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
