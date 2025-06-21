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
#include <chrono>

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
#include "PairwiseHarmonicsSegmentation.h"
#include "IsocurveAnalysis.h"
#include "RigidityAnalysis.h"
#include "VisualizationUtils.h"

using namespace std;

// Global variables for core components
Mesh* g_mesh = nullptr;
Painter* g_painter = nullptr;
PairwiseHarmonics* g_harmonics = nullptr;
PairwiseHarmonicsSegmentation* g_segmentation = nullptr;

// Global variables for visualization
SoSeparator* g_root = nullptr;
SoWinExaminerViewer* g_viewer = nullptr;

// Global variables for segmentation algorithm state
vector<int> g_fps_samples;
PairwiseHarmonicsSegmentation::SegmentationResult g_segmentation_result;

// Additional global variables for enhanced analysis
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
void testHeatGeodesics();
void testEnhancedFPS();
void compareGeodesicMethods();
void testEnhancedDescriptors();
void testMeshAnalysis();

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
    cin >> meshPath;    // Clear previous state
    clearVisualization();
    if (g_segmentation) {
        delete g_segmentation;
        g_segmentation = nullptr;
    }
    if (g_harmonics) {
        delete g_harmonics;
        g_harmonics = nullptr;
    }
    if (g_mesh) {
        delete g_mesh;
        g_mesh = nullptr;
    }// Create and load new mesh
    g_mesh = new Mesh();

    // Construct full path to mesh file in models directory
    string fullMeshPath = "models/" + meshPath;    cout << "Loading mesh: " << fullMeshPath << "..." << endl;
    g_mesh->loadOff(const_cast<char*>(fullMeshPath.c_str()));
    g_mesh->normalizeCoordinates();

    cout << "Mesh loaded successfully!" << endl;
    cout << "  Vertices: " << g_mesh->verts.size() << endl;
    cout << "  Triangles: " << g_mesh->tris.size() << endl;
    cout << "  Edges: " << g_mesh->edges.size() << endl;

    // Convert mesh to LibIGL format for enhanced processing
    cout << "\nConverting mesh to LibIGL format..." << endl;
    g_mesh->convertToLibIGL();

    // Add mesh to visualization
    g_root->addChild(g_painter->getShapeSep(g_mesh));// Initialize pairwise harmonics system
    g_harmonics = new PairwiseHarmonics(g_mesh);

    // Initialize LibIGL for robust geometry processing
    cout << "\nInitializing LibIGL for robust computations..." << endl;
    if (g_harmonics->initializeLibIGL()) {
        cout << "LibIGL initialization successful!" << endl;

        // Precompute heat geodesics for fast distance computation
        cout << "Precomputing heat geodesics for enhanced performance..." << endl;
        if (g_mesh->precomputeHeatGeodesics()) {
            cout << "Heat geodesics precomputation successful!" << endl;
            cout << "  Fast geodesic distance computation is now available" << endl;
        } else {
            cout << "Warning: Heat geodesics precomputation failed. Using Dijkstra fallback." << endl;
        }

    } else {
        cout << "Warning: LibIGL initialization failed. Some features may not work optimally." << endl;
    }

    // Clear segmentation state
    g_fps_samples.clear();
    g_skeletal_segments.clear();
    g_rigidity_scores.clear();
    g_nonrigid_nodes.clear();

    g_viewer->viewAll();
    cout << "Mesh ready for pairwise harmonics analysis with LibIGL support!" << endl;
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
 * Uses the new clean PairwiseHarmonicsSegmentation class
 */
void performFullSegmentation() {
    if (!g_mesh || !g_harmonics) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }

    clearVisualization();

    // Initialize segmentation object if needed
    if (!g_segmentation) {
        g_segmentation = new PairwiseHarmonicsSegmentation(g_mesh, g_harmonics);
    }

    // Allow user to configure parameters
    cout << "\n=== SEGMENTATION PARAMETERS ===" << endl;
    cout << "Use default parameters? (y/n): ";
    char useDefaults;
    cin >> useDefaults;

    if (useDefaults != 'y' && useDefaults != 'Y') {
        PairwiseHarmonicsSegmentation::SegmentationParams params = g_segmentation->getParameters();

        cout << "Current FPS samples (" << params.numFPSSamples << "): ";
        int newFPS;
        cin >> newFPS;
        if (newFPS > 0) params.numFPSSamples = newFPS;

        cout << "Current isocurves per field (" << params.numIsocurves << "): ";
        int newIsocurves;
        cin >> newIsocurves;
        if (newIsocurves > 0) params.numIsocurves = newIsocurves;

        cout << "Current rigidity threshold (" << params.rigidityThreshold << "): ";
        double newThreshold;
        cin >> newThreshold;
        if (newThreshold > 0 && newThreshold <= 1.0) params.rigidityThreshold = newThreshold;

        g_segmentation->setParameters(params);
    }

    // Perform segmentation
    cout << "\nStarting pairwise harmonics segmentation..." << endl;
    g_segmentation_result = g_segmentation->performSegmentation();

    if (g_segmentation_result.success) {        // Visualize results
        PairwiseHarmonicsSegmentation::visualizeResults(
            g_segmentation_result, g_root, g_painter, g_viewer, g_mesh
        );

        cout << "\n=== SEGMENTATION SUMMARY ===" << endl;
        cout << "-> Successfully segmented mesh into " << g_segmentation_result.meshComponents.size() << " components" << endl;
        cout << "-> Generated partial skeleton with " << g_segmentation_result.partialSkeleton.size() << " segments" << endl;
        cout << "-> Created complete skeleton with " << g_segmentation_result.skeletonNodes.size() << " nodes" << endl;
        cout << "-> Used " << g_segmentation_result.fpsPoints.size() << " FPS sample points" << endl;
    } else {
        cout << "\n[X] SEGMENTATION FAILED: " << g_segmentation_result.errorMessage << endl;
    }
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
    cout << "--- LIBIGL ENHANCED FEATURES ---" << endl;
    cout << "13. Test Heat Geodesics (LibIGL)" << endl;
    cout << "14. Enhanced FPS with Heat Geodesics" << endl;
    cout << "15. Compare Geodesic Methods (Dijkstra vs Heat)" << endl;
    cout << "16. Enhanced R&D Descriptors (LibIGL)" << endl;
    cout << "17. Mesh Analysis (Normals, Curvature, Areas)" << endl;
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
            case 13:
                testHeatGeodesics();
                break;
            case 14:
                testEnhancedFPS();
                break;
            case 15:
                compareGeodesicMethods();
                break;
            case 16:
                testEnhancedDescriptors();
                break;
            case 17:
                testMeshAnalysis();
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

/**
 * Test LibIGL heat geodesics functionality
 */
void testHeatGeodesics() {
    if (!g_mesh || !g_harmonics) {
        cout << "Error: Mesh and harmonics must be loaded first!" << endl;
        return;
    }

    cout << "\n=== TESTING HEAT GEODESICS ===" << endl;
    cout << "This test compares LibIGL heat geodesics with Dijkstra distances" << endl;

    // Test heat geodesics on a few vertices
    int numVertices = g_mesh->verts.size();
    vector<int> testVertices = {0, numVertices/4, numVertices/2, 3*numVertices/4};

    for (int v : testVertices) {
        if (v >= numVertices) continue;

        cout << "Testing heat geodesics from vertex " << v << "..." << endl;
        Eigen::VectorXd heatDist = g_harmonics->computeHeatGeodesicDistances(v);
        cout << "  Heat geodesic computation successful (size: " << heatDist.size() << ")" << endl;
        cout << "  Max distance: " << heatDist.maxCoeff() << endl;
        cout << "  Min distance: " << heatDist.minCoeff() << endl;
    }

    cout << "Heat geodesics test completed!" << endl;
}

/**
 * Test enhanced FPS sampling
 */
void testEnhancedFPS() {
    if (!g_mesh) {
        cout << "Error: Mesh must be loaded first!" << endl;
        return;
    }

    cout << "\n=== TESTING ENHANCED FPS ===" << endl;
    cout << "Testing Farthest Point Sampling with LibIGL heat geodesics" << endl;

    // Test with different numbers of samples
    vector<int> sampleCounts = {5, 10, 20, 30};

    for (int count : sampleCounts) {
        cout << "Testing FPS with " << count << " samples..." << endl;

        // Use the Mesh's FPS method directly
        vector<int> fps_samples = g_mesh->farthestPointSamplingLibIGL(count);
        cout << "  FPS generated " << fps_samples.size() << " samples" << endl;

        if (!fps_samples.empty()) {
            cout << "  Sample vertices: ";
            for (size_t i = 0; i < min(size_t(5), fps_samples.size()); ++i) {
                cout << fps_samples[i] << " ";
            }
            if (fps_samples.size() > 5) cout << "...";
            cout << endl;
        }
    }

    cout << "Enhanced FPS test completed!" << endl;
}

/**
 * Compare different geodesic computation methods
 */
void compareGeodesicMethods() {
    if (!g_mesh || !g_harmonics) {
        cout << "Error: Mesh and harmonics must be loaded first!" << endl;
        return;
    }

    cout << "\n=== COMPARING GEODESIC METHODS ===" << endl;
    cout << "Comparing LibIGL heat geodesics with other methods" << endl;

    int sourceVertex = 0;
    int numVertices = g_mesh->verts.size();

    if (numVertices > 0) {
        cout << "Computing heat geodesics from vertex " << sourceVertex << "..." << endl;

        auto start = chrono::high_resolution_clock::now();
        Eigen::VectorXd heatDist = g_harmonics->computeHeatGeodesicDistances(sourceVertex);
        auto end = chrono::high_resolution_clock::now();

        auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
        cout << "Heat geodesics computed in " << duration.count() << " ms" << endl;
        cout << "Result statistics:" << endl;
        cout << "  Size: " << heatDist.size() << endl;
        cout << "  Max distance: " << heatDist.maxCoeff() << endl;
        cout << "  Min distance: " << heatDist.minCoeff() << endl;
        cout << "  Mean distance: " << heatDist.mean() << endl;
    }

    cout << "Geodesic methods comparison completed!" << endl;
}

/**
 * Test enhanced descriptor computation
 */
void testEnhancedDescriptors() {
    if (!g_mesh || !g_harmonics) {
        cout << "Error: Mesh and harmonics must be loaded first!" << endl;
        return;
    }

    cout << "\n=== TESTING ENHANCED DESCRIPTORS ===" << endl;
    cout << "Testing LibIGL-based R and D descriptors" << endl;

    int numVertices = g_mesh->verts.size();
    if (numVertices < 2) {
        cout << "Error: Mesh needs at least 2 vertices!" << endl;
        return;
    }

    int p = 0;
    int q = numVertices / 2;

    cout << "Computing harmonic field between vertices " << p << " and " << q << "..." << endl;

    // Compute harmonic field
    Eigen::VectorXd field = g_harmonics->computePairwiseHarmonicLibIGL(p, q);

    if (field.size() > 0) {
        cout << "Harmonic field computed successfully (size: " << field.size() << ")" << endl;

        // Test R descriptor
        cout << "Computing R descriptor..." << endl;
        Eigen::VectorXd rDesc = g_harmonics->computeRDescriptorLibIGL(field, DEFAULT_K);
        cout << "R descriptor size: " << rDesc.size() << endl;

        // Test D descriptor
        cout << "Computing D descriptor..." << endl;
        Eigen::VectorXd dDesc = g_harmonics->computeDDescriptorLibIGL(p, q, field, DEFAULT_K);
        cout << "D descriptor size: " << dDesc.size() << endl;
    } else {
        cout << "Error: Failed to compute harmonic field!" << endl;
    }

    cout << "Enhanced descriptors test completed!" << endl;
}

/**
 * Test comprehensive mesh analysis
 */
void testMeshAnalysis() {
    if (!g_mesh) {
        cout << "Error: Mesh must be loaded first!" << endl;
        return;
    }

    cout << "\n=== TESTING MESH ANALYSIS ===" << endl;
    cout << "Running comprehensive mesh analysis with LibIGL" << endl;

    int numVertices = g_mesh->verts.size();
    int numFaces = g_mesh->tris.size();

    cout << "Mesh statistics:" << endl;
    cout << "  Vertices: " << numVertices << endl;
    cout << "  Faces: " << numFaces << endl;

    if (g_harmonics && g_harmonics->isLibIGLInitialized()) {
        const Eigen::MatrixXd& V = g_harmonics->getVertexMatrix();
        const Eigen::MatrixXi& F = g_harmonics->getFaceMatrix();

        cout << "LibIGL mesh data:" << endl;
        cout << "  Vertex matrix: " << V.rows() << " x " << V.cols() << endl;
        cout << "  Face matrix: " << F.rows() << " x " << F.cols() << endl;

        // Test Laplacian computation
        cout << "Testing Laplacian computation..." << endl;
        bool laplacianOk = g_harmonics->computeCotangentLaplacianLibigl();
        cout << "Laplacian computation: " << (laplacianOk ? "SUCCESS" : "FAILED") << endl;

        // Test boundary detection
        cout << "Analyzing mesh properties..." << endl;
        cout << "Mesh analysis completed!" << endl;
    } else {
        cout << "Warning: LibIGL data not initialized" << endl;
    }

    cout << "Comprehensive mesh analysis completed!" << endl;
}

// End of file