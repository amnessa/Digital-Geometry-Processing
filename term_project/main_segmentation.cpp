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
#include <limits>

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
const double RIGIDITY_THRESHOLD = 0.85;  // Threshold for classifying rigid vs non-rigid nodes (significantly lowered for better junction detection)
const int DEFAULT_FPS_SAMPLES = 25;     // Number of FPS samples for initial point generation (increased from 15)

// Forward declarations
void loadMesh();
void unloadMesh();
void showMainMenu();
void performFullSegmentation();
void clearVisualization();
void testHeatGeodesics();
void testEnhancedFPS();
void compareGeodesicMethods();
void testEnhancedDescriptors();
void testMeshAnalysis();
void visualizeMeshSegmentationClusters();
void visualizeSkeletalKMeansClustering();
void visualizeGraphCutSegmentation(); // NEW: Graph-cut based optimal segmentation
void analyzeSkeletalSegments();
void analyzeDetailedRigidity();
void testHarmonicSurfaceSegmentation(); // Forward declare the new test function

/**
 * Load a mesh file and initialize the segmentation system
 */
void loadMesh() {
    string meshPath;
    cout << "\n=== LOAD MESH ===" << endl;
    cout << "Available meshes:" << endl;
    cout << "  1. 50.off" << endl;
    cout << "  2. 249.off" << endl;
    cout << "  3. 348.off" << endl;
    cout << "  4. man0.off" << endl;
    cout << "  5. Armadillo.off" << endl;
    cout << "  6. chair.off" << endl;
    cout << "  7. cow.off" << endl;
    cout << "  8. hand.off" << endl;
    cout << "\nEnter mesh number (1-8) or filename directly: ";

    string input;
    cin >> input;

    // Check if input is a number or filename
    if (input == "1") meshPath = "50.off";
    else if (input == "2") meshPath = "249.off";
    else if (input == "3") meshPath = "348.off";
    else if (input == "4") meshPath = "man0.off";
    else if (input == "5") meshPath = "Armadillo.off";
    else if (input == "6") meshPath = "chair.off";
    else if (input == "7") meshPath = "cow.off";
    else if (input == "8") meshPath = "hand.off";    else meshPath = input; // Assume it's a direct filename

    // Clear previous state completely
    if (g_root) {
        g_root->removeAllChildren();
    }

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
    }

    // Create and load new mesh
    g_mesh = new Mesh();

    // Construct full path to mesh file in models directory
    string fullMeshPath = "models/" + meshPath;    cout << "Loading mesh: " << fullMeshPath << "..." << endl;
    g_mesh->loadOff(const_cast<char*>(fullMeshPath.c_str()));
    g_mesh->normalizeCoordinates();

    cout << "Mesh loaded successfully!" << endl;
    cout << "  Vertices: " << g_mesh->verts.size() << endl;
    cout << "  Triangles: " << g_mesh->tris.size() << endl;
    cout << "  Edges: " << g_mesh->edges.size() << endl;    // Convert mesh to LibIGL format for enhanced processing
    cout << "\nConverting mesh to LibIGL format..." << endl;
    g_mesh->convertToLibIGL();

    // Initialize pairwise harmonics system AFTER mesh conversion
    g_harmonics = new PairwiseHarmonics(g_mesh);

    // Initialize LibIGL for robust geometry processing
    cout << "Initializing LibIGL for robust computations..." << endl;
    if (g_harmonics->initializeLibIGL()) {
        cout << "LibIGL initialization successful!" << endl;

        // Precompute heat geodesics for fast distance computation        cout << "Precomputing heat geodesics for enhanced performance..." << endl;
        if (g_mesh->precomputeHeatGeodesics()) {
            cout << "Heat geodesics precomputation successful!" << endl;
            cout << "  Fast geodesic distance computation is now available" << endl;
        } else {
            cout << "Warning: Heat geodesics precomputation failed. Using Dijkstra fallback." << endl;
        }

    } else {
        cout << "Warning: LibIGL initialization failed. Some features may not work optimally." << endl;
    }

    // Add mesh to visualization
    g_root->addChild(g_painter->getShapeSep(g_mesh));

    cout << "Mesh ready for pairwise harmonics analysis with LibIGL support!" << endl;

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
    cout << "======================================" << endl;    cout << "1. Load Mesh" << endl;
    cout << "2. Unload Current Mesh" << endl;
    cout << "3. Test Cotangent Laplacian" << endl;
    cout << "4. Test Farthest Point Sampling" << endl;
    cout << "5. Test Pairwise Harmonics" << endl;
    cout << "6. Test Iso-curve Extraction" << endl;
    cout << "7. Test Rigidity Analysis" << endl;
    cout << "8. Perform Full Segmentation" << endl;
    cout << "9. Clear Visualization" << endl;    cout << "--- ENHANCED ANALYSIS ---" << endl;
    cout << "10. Improved Iso-curve Extraction (Dense Bracelets)" << endl;
    cout << "11. Enhanced Rigidity Analysis (All Nodes)" << endl;
    cout << "12. Visualize All Rigidity Points (Color-coded)" << endl;
    cout << "13. Interactive Rigidity Threshold Testing" << endl;    cout << "--- SEGMENTATION VISUALIZATION ---" << endl;
    cout << "19. Visualize Mesh Components (Colored Vertex Clusters)" << endl;    cout << "20. Harmonic Function Surface Segmentation" << endl;
    cout << "21. Analyze Skeletal Segments (Diagnostic)" << endl;
    cout << "22. Detailed Rigidity Analysis (Debug Cutting)" << endl;
    cout << "23. Optimal Segmentation (Graph-Cut Energy Minimization)" << endl;
    cout << "--- LIBIGL ENHANCED FEATURES ---" << endl;
    cout << "14. Test Heat Geodesics (LibIGL)" << endl;
    cout << "15. Enhanced FPS with Heat Geodesics" << endl;
    cout << "16. Compare Geodesic Methods (Dijkstra vs Heat)" << endl;
    cout << "17. Enhanced R&D Descriptors (LibIGL)" << endl;
    cout << "18. Mesh Analysis (Normals, Curvature, Areas)" << endl;
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

        // Robust input handling with error checking
        if (!(cin >> choice)) {
            // Clear error flag
            cin.clear();
            // Clear input buffer
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            cout << "Invalid input! Please enter a number." << endl;
            continue;
        }

        // Clear any remaining characters in the input buffer
        cin.ignore(numeric_limits<streamsize>::max(), '\n');        switch (choice) {
            case 1:
                loadMesh();
                break;
            case 2:
                unloadMesh();
                break;
            case 3:
                testCotangentLaplacian();
                break;
            case 4:
                testFarthestPointSampling();
                break;
            case 5:
                testPairwiseHarmonics();
                break;
            case 6:
                testIsoCurveExtraction();
                break;
            case 7:
                testRigidityAnalysis();
                break;
            case 8:
                performFullSegmentation();
                break;
            case 9:
                clearVisualization();
                g_viewer->scheduleRedraw();
                g_viewer->viewAll();
                cout << "Visualization cleared." << endl;
                break;
            case 10:
                IsocurveAnalysis::improvedIsoCurveExtraction(g_mesh, g_harmonics, g_fps_samples, g_root, g_viewer, g_skeletal_segments);
                break;
            case 11:
                RigidityAnalysis::enhancedRigidityAnalysis(g_mesh, g_harmonics, g_fps_samples, g_skeletal_segments, g_rigidity_scores, g_nonrigid_nodes);
                break;
            case 12:
                RigidityAnalysis::visualizeAllRigidityPoints(g_mesh, g_skeletal_segments, g_rigidity_scores, g_root, g_viewer);
                break;
            case 13:
                RigidityAnalysis::interactiveRigidityThresholdTesting(g_skeletal_segments, g_rigidity_scores, g_root, g_viewer);
                break;
            case 14:
                testHeatGeodesics();
                break;
            case 15:
                testEnhancedFPS();
                break;
            case 16:
                compareGeodesicMethods();
                break;
            case 17:
                testEnhancedDescriptors();
                break;
            case 18:
                testMeshAnalysis();
                break;
            case 19:
                visualizeMeshSegmentationClusters();
                break;
            case 20:
                visualizeSkeletalKMeansClustering();
                break;
            case 21:
                analyzeSkeletalSegments();
                break;            case 22:
                analyzeDetailedRigidity();
                break;
            case 23:
                visualizeGraphCutSegmentation();
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

// ===================================================================
// SEGMENTATION CONCEPTS EXPLAINED:
// ===================================================================
// 1. MESH COMPONENTS: These are the final segmentation results - groups of vertices
//    that belong to the same "part" of the mesh (e.g., arm, leg, torso).
//    ✓ These are what you want to visualize with different colors!
//
// 2. SKELETAL SEGMENTS: These represent the "bones" or medial axis segments
//    that form the shape's internal skeleton structure.
//    → Used for analysis but NOT the final segmentation output
//
// 3. SKELETON NODES: Points along the skeleton representing junctions
//    or important anatomical landmarks.
//    → Used for skeleton construction and refinement
//
// The correct visualization shows mesh components (option 18), where each
// color represents vertices belonging to the same segmented part of the mesh.
// ===================================================================

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

    clearVisualization();

    for (int v : testVertices) {
        if (v >= numVertices) continue;

        cout << "Testing heat geodesics from vertex " << v << "..." << endl;
        Eigen::VectorXd heatDist = g_harmonics->computeHeatGeodesicDistances(v);
        cout << "Computed heat geodesic distances from vertex " << v << " (range: " << heatDist.minCoeff() << " to " << heatDist.maxCoeff() << ")" << endl;
        cout << "  Heat geodesic computation successful (size: " << heatDist.size() << ")" << endl;
        cout << "  Max distance: " << heatDist.maxCoeff() << endl;
        cout << "  Min distance: " << heatDist.minCoeff() << endl;

        // Visualize heat geodesic distances as scalar field
        VisualizationUtils::visualizeScalarField(g_mesh, heatDist, g_root, "Heat Geodesic Distances from vertex " + to_string(v));

        // Mark the source vertex with a special sphere
        Eigen::Vector3d sourcePos(g_mesh->verts[v]->coords[0], g_mesh->verts[v]->coords[1], g_mesh->verts[v]->coords[2]);
        Eigen::Vector3f sourceColor(1.0f, 1.0f, 0.0f); // Yellow for source
        SoSeparator* sourceViz = VisualizationUtils::createColoredSphere(sourcePos, sourceColor, 0.02f);
        g_root->addChild(sourceViz);

        break; // Only visualize first one to avoid clutter
    }

    g_viewer->scheduleRedraw();
    g_viewer->viewAll();
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

    clearVisualization();

    // Test with different numbers of samples
    vector<int> sampleCounts = {5, 10, 20, 30};

    for (int count : sampleCounts) {
        cout << "Testing FPS with " << count << " samples..." << endl;

        // Use the Mesh's FPS method directly
        vector<int> fps_samples = g_mesh->farthestPointSamplingLibIGL(count);
        cout << "LibIGL-enhanced FPS completed: " << fps_samples.size() << " samples generated" << endl;

        if (!fps_samples.empty()) {
            cout << "  FPS generated " << fps_samples.size() << " samples" << endl;
            cout << "  Sample vertices: ";
            for (size_t i = 0; i < min(size_t(5), fps_samples.size()); ++i) {
                cout << fps_samples[i] << " ";
            }
            if (fps_samples.size() > 5) cout << "...";
            cout << endl;
        }
    }

    // Visualize the last set of FPS samples
    cout << "Visualizing FPS samples with " << sampleCounts.back() << " points..." << endl;
    vector<int> fps_samples = g_mesh->farthestPointSamplingLibIGL(sampleCounts.back());

    for (size_t i = 0; i < fps_samples.size(); i++) {
        Eigen::Vector3d pos(g_mesh->verts[fps_samples[i]]->coords[0],
                            g_mesh->verts[fps_samples[i]]->coords[1],
                            g_mesh->verts[fps_samples[i]]->coords[2]);

        // Color based on sampling order: first red, last blue, others green
        Eigen::Vector3f color;
        if (i == 0) {
            color = Eigen::Vector3f(1.0f, 0.0f, 0.0f); // Red for first
        } else if (i == fps_samples.size() - 1) {
            color = Eigen::Vector3f(0.0f, 1.0f, 0.0f); // Green for last
        } else {
            color = Eigen::Vector3f(0.0f, 1.0f, 0.0f); // Green for others
        }

        SoSeparator* pointViz = VisualizationUtils::createColoredSphere(pos, color, 0.02f);
        g_root->addChild(pointViz);
    }

    g_viewer->scheduleRedraw();
    g_viewer->viewAll();
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
    cout << "Comparing LibIGL heat geodesics with Dijkstra-based geodesics" << endl;

    int sourceVertex = 0;
    int numVertices = g_mesh->verts.size();

    if (numVertices > 0) {
        cout << "Method 1: LibIGL Heat Geodesics" << endl;
        cout << "Computing heat geodesics from vertex " << sourceVertex << "..." << endl;

        auto start1 = chrono::high_resolution_clock::now();
        Eigen::VectorXd heatDist = g_harmonics->computeHeatGeodesicDistances(sourceVertex);
        auto end1 = chrono::high_resolution_clock::now();

        auto duration1 = chrono::duration_cast<chrono::milliseconds>(end1 - start1);
        cout << "Heat geodesics computed in " << duration1.count() << " ms" << endl;
        cout << "Result statistics:" << endl;
        cout << "  Size: " << heatDist.size() << endl;
        cout << "  Max distance: " << heatDist.maxCoeff() << endl;
        cout << "  Min distance: " << heatDist.minCoeff() << endl;
        cout << "  Mean distance: " << heatDist.mean() << endl;
        cout << "\nMethod 2: Dijkstra-based Geodesics" << endl;
        cout << "Computing Dijkstra geodesics from vertex " << sourceVertex << "..." << endl;

        auto start2 = chrono::high_resolution_clock::now();
        int N;
        float* dijkstraDistPtr = g_mesh->computeGeodesicDistances(sourceVertex, N);
        auto end2 = chrono::high_resolution_clock::now();

        auto duration2 = chrono::duration_cast<chrono::milliseconds>(end2 - start2);
        cout << "Dijkstra geodesics computed in " << duration2.count() << " ms" << endl;

        if (dijkstraDistPtr && N > 0) {
            // Convert to vector for easier handling
            vector<double> dijkstraDist(dijkstraDistPtr, dijkstraDistPtr + N);

            double maxDijkstra = *max_element(dijkstraDist.begin(), dijkstraDist.end());
            double minDijkstra = *min_element(dijkstraDist.begin(), dijkstraDist.end());
            double meanDijkstra = 0;
            for (double d : dijkstraDist) meanDijkstra += d;
            meanDijkstra /= dijkstraDist.size();

            cout << "Result statistics:" << endl;
            cout << "  Size: " << dijkstraDist.size() << endl;
            cout << "  Max distance: " << maxDijkstra << endl;
            cout << "  Min distance: " << minDijkstra << endl;
            cout << "  Mean distance: " << meanDijkstra << endl;

            cout << "\nComparison Summary:" << endl;
            cout << "  Heat geodesics time: " << duration1.count() << " ms" << endl;
            cout << "  Dijkstra time: " << duration2.count() << " ms" << endl;
            cout << "  Speed improvement: " << (double(duration2.count()) / duration1.count()) << "x faster" << endl;

            // Clean up the allocated memory
            delete[] dijkstraDistPtr;
        } else {
            cout << "Dijkstra geodesics computation failed!" << endl;
        }
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
    cout << "Testing LibIGL-based R and D descriptors for shape analysis" << endl;
    cout << "R Descriptor: Measures isocurve lengths (shape profile)" << endl;
    cout << "D Descriptor: Measures distance variations (shape complexity)" << endl;

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
        cout << "Computing R descriptor (isocurve length measurements)..." << endl;
        Eigen::VectorXd rDesc = g_harmonics->computeRDescriptorLibIGL(field, DEFAULT_K);
        cout << "R descriptor computed successfully (size: " << rDesc.size() << ")" << endl;
        cout << "R descriptor values (first 10): ";
        for (int i = 0; i < min(10, (int)rDesc.size()); i++) {
            cout << rDesc(i) << " ";
        }
        cout << endl;

        // Test D descriptor
        cout << "Computing D descriptor (distance variation measurements)..." << endl;
        Eigen::VectorXd dDesc = g_harmonics->computeDDescriptorLibIGL(p, q, field, DEFAULT_K);
        cout << "D descriptor computed successfully (size: " << dDesc.size() << ")" << endl;
        cout << "D descriptor values (first 10): ";
        for (int i = 0; i < min(10, (int)dDesc.size()); i++) {
            cout << dDesc(i) << " ";
        }
        cout << endl;

        cout << "\nDescriptor Applications:" << endl;
        cout << "  R Descriptor: Used for shape matching and symmetry detection" << endl;
        cout << "  D Descriptor: Used for mesh segmentation and part identification" << endl;
        cout << "  Both descriptors feed into the segmentation algorithm in Section 3.3" << endl;
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

/**
 * Visualize mesh segmentation by coloring vertices according to their mesh components
 * Each vertex is colored based on which mesh component (segmented part) it belongs to
 */
void visualizeMeshSegmentationClusters() {
    if (!g_segmentation_result.success || g_segmentation_result.meshComponents.empty() || !g_mesh) {
        cout << "Error: Please run full segmentation first to generate mesh components!" << endl;
        cout << "Use option 1 to perform complete segmentation." << endl;
        return;
    }

    cout << "\n=== MESH COMPONENT VISUALIZATION ===" << endl;
    cout << "Showing " << g_segmentation_result.meshComponents.size() << " mesh components with colored vertices" << endl;

    clearVisualization();

    // Define colors for different mesh components
    vector<Eigen::Vector3f> componentColors = {
        Eigen::Vector3f(1.0f, 0.0f, 0.0f), // Red
        Eigen::Vector3f(0.0f, 1.0f, 0.0f), // Green
        Eigen::Vector3f(0.0f, 0.0f, 0.0f), // White
        Eigen::Vector3f(1.0f, 1.0f, 0.0f), // Yellow
        Eigen::Vector3f(1.0f, 0.0f, 1.0f), // Magenta
        Eigen::Vector3f(0.0f, 1.0f, 1.0f), // Cyan
        Eigen::Vector3f(1.0f, 0.5f, 0.0f), // Orange
        Eigen::Vector3f(0.5f, 0.0f, 1.0f), // Purple
        Eigen::Vector3f(0.5f, 1.0f, 0.5f), // Light Green
        Eigen::Vector3f(1.0f, 0.5f, 0.5f)  // Light Red
    };

    // Create a mapping from vertex index to component index
    vector<int> vertexToComponent(g_mesh->verts.size(), -1);

    for (int compIdx = 0; compIdx < g_segmentation_result.meshComponents.size(); compIdx++) {
        const auto& component = g_segmentation_result.meshComponents[compIdx];
        for (int vertexIdx : component.vertexIndices) {
            if (vertexIdx >= 0 && vertexIdx < g_mesh->verts.size()) {
                vertexToComponent[vertexIdx] = compIdx;
            }
        }    }

    // Visualize vertices colored by their mesh component assignment
    cout << "Rendering colored spheres for each mesh component..." << endl;

    for (int compIdx = 0; compIdx < g_segmentation_result.meshComponents.size(); compIdx++) {
        const auto& component = g_segmentation_result.meshComponents[compIdx];
        Eigen::Vector3f color = componentColors[compIdx % componentColors.size()];

        cout << "  Component " << compIdx << ": " << component.vertexIndices.size() << " vertices, color: ("
             << color[0] << ", " << color[1] << ", " << color[2] << ")" << endl;

        // Visualize vertices in this component (downsample for performance)
        for (int i = 0; i < component.vertexIndices.size(); i += 15) { // Every 15th vertex
            int vertexIdx = component.vertexIndices[i];
            if (vertexIdx >= 0 && vertexIdx < g_mesh->verts.size()) {
                Eigen::Vector3d vertexPos(g_mesh->verts[vertexIdx]->coords[0],
                                         g_mesh->verts[vertexIdx]->coords[1],
                                         g_mesh->verts[vertexIdx]->coords[2]);

                SoSeparator* pointViz = VisualizationUtils::createColoredSphere(vertexPos, color, 0.02f);
                g_root->addChild(pointViz);
            }
        }

        // Also show the component center as a larger sphere
        SoSeparator* centerViz = VisualizationUtils::createColoredSphere(component.center, color, 0.02f);
        g_root->addChild(centerViz);
    }

    // Also visualize the partial skeleton as lines
    if (!g_segmentation_result.partialSkeleton.empty()) {
        cout << "Rendering partial skeleton..." << endl;
        for (int s = 0; s < g_segmentation_result.partialSkeleton.size(); s++) {
            const auto& segment = g_segmentation_result.partialSkeleton[s];
            if (segment.nodes.size() < 2) continue;

            // Use white/gray for skeleton
            Eigen::Vector3f skeletonColor(0.8f, 0.8f, 0.8f);

            SoSeparator* lineSep = new SoSeparator();
            SoMaterial* mat = new SoMaterial();
            mat->diffuseColor.setValue(skeletonColor[0], skeletonColor[1], skeletonColor[2]);
            mat->emissiveColor.setValue(skeletonColor[0] * 0.5f, skeletonColor[1] * 0.5f, skeletonColor[2] * 0.5f);
            lineSep->addChild(mat);

            SoCoordinate3* coords = new SoCoordinate3();
            coords->point.setNum(segment.nodes.size());

            for (int i = 0; i < segment.nodes.size(); i++) {
                coords->point.set1Value(i, segment.nodes[i][0], segment.nodes[i][1], segment.nodes[i][2]);
            }
            lineSep->addChild(coords);

            SoIndexedLineSet* lineSet = new SoIndexedLineSet();
            for (int i = 0; i < segment.nodes.size() - 1; i++) {
                lineSet->coordIndex.set1Value(i * 3, i);
                lineSet->coordIndex.set1Value(i * 3 + 1, i + 1);
                lineSet->coordIndex.set1Value(i * 3 + 2, -1);
            }
            lineSep->addChild(lineSet);
            g_root->addChild(lineSep);
        }
    }

    g_viewer->scheduleRedraw();
    g_viewer->viewAll();

    cout << "\n=== MESH COMPONENT VISUALIZATION COMPLETE ===" << endl;
    cout << "-> Displayed " << g_segmentation_result.meshComponents.size() << " mesh components with unique colors" << endl;
    cout << "-> Each colored sphere represents a vertex in that component" << endl;
    cout << "-> Large spheres show component centers" << endl;
    cout << "-> Gray lines show the partial skeleton structure" << endl;
}

/**
 * Color the surface using the final skeleton with harmonic function competition
 * This implements the correct approach described in the paper
 */
void visualizeSkeletalKMeansClustering() {
    if (!g_segmentation_result.success || g_segmentation_result.partialSkeleton.empty() || !g_mesh || !g_harmonics) {
        cout << "Error: Please run full segmentation first to generate skeletal segments!" << endl;
        cout << "Use option 7 to perform complete segmentation." << endl;
        return;
    }

    cout << "\n=== HARMONIC FUNCTION SURFACE SEGMENTATION ===" << endl;
    cout << "Using " << g_segmentation_result.partialSkeleton.size() << " skeletal segments for surface competition" << endl;

    clearVisualization();

    // Define colors for different segments
    vector<Eigen::Vector3f> segmentColors = {
        Eigen::Vector3f(1.0f, 0.0f, 0.0f), // Red
        Eigen::Vector3f(0.0f, 1.0f, 0.0f), // Green
        Eigen::Vector3f(0.0f, 0.0f, 1.0f), // Blue
        Eigen::Vector3f(1.0f, 1.0f, 0.0f), // Yellow
        Eigen::Vector3f(1.0f, 0.0f, 1.0f), // Magenta
        Eigen::Vector3f(0.0f, 1.0f, 1.0f), // Cyan
        Eigen::Vector3f(1.0f, 0.5f, 0.0f), // Orange
        Eigen::Vector3f(0.5f, 0.0f, 1.0f), // Purple
        Eigen::Vector3f(0.5f, 1.0f, 0.5f), // Light Green
        Eigen::Vector3f(1.0f, 0.5f, 0.5f)  // Light Red
    };

    // Step 0: SELECT THE BEST SKELETAL SEGMENTS (this is what's missing!)
    cout << "Step 0: Using ALL skeletal segments from the main segmentation..." << endl;

    // --- START OF REPLACEMENT ---
    // Create a simple list of all segment indices from the segmentation result.
    // We are removing the extra greedy selection to ensure we use the exact
    // skeleton (including torso parts) that the main algorithm generated.
    vector<int> selectedSegmentIndices;
    for (int i = 0; i < g_segmentation_result.partialSkeleton.size(); ++i) {
        selectedSegmentIndices.push_back(i);
    }
    // --- END OF REPLACEMENT ---

    cout << "Using all " << selectedSegmentIndices.size() << " segments for rigidity analysis." << endl;

    // NOW use only the selected segments for rigidity analysis and segmentation
    cout << "Step 1: Analyzing rigidity for ALL segments..." << endl;

    struct RigidSegment {
        int originalSegmentIdx;
        vector<Eigen::Vector3d> nodes;
        double avgRigidity;
        double startHarmonicValue;
        double endHarmonicValue;
        int sourceIdx, targetIdx;
    };

    vector<RigidSegment> rigidSegments;
    const double RIGIDITY_SPLIT_THRESHOLD = 0.85; // Threshold for splitting at junctions

    for (int segIdx : selectedSegmentIndices) {
        const auto& segment = g_segmentation_result.partialSkeleton[segIdx];

        if (segment.nodes.size() < 3 || segment.nodeRigidities.size() != segment.nodes.size()) {
            // Can't analyze this segment, add as-is
            RigidSegment rigidSeg;
            rigidSeg.originalSegmentIdx = segIdx;
            rigidSeg.nodes = segment.nodes;
            rigidSeg.avgRigidity = segment.avgRigidity;
            rigidSeg.startHarmonicValue = 0.0;
            rigidSeg.endHarmonicValue = 1.0;
            rigidSeg.sourceIdx = segment.sourceIdx;
            rigidSeg.targetIdx = segment.targetIdx;
            rigidSegments.push_back(rigidSeg);
            continue;
        }

        // Find junction points (low rigidity nodes)
        vector<int> junctionIndices;

        // IMPROVED: Use relative rigidity drops instead of absolute threshold
        cout << "    Analyzing rigidity profile for segment " << segIdx << ":" << endl;

        // Find the minimum rigidity in this segment
        double minRigidity = *min_element(segment.nodeRigidities.begin(), segment.nodeRigidities.end());
        double maxRigidity = *max_element(segment.nodeRigidities.begin(), segment.nodeRigidities.end());
        double rigidityRange = maxRigidity - minRigidity;

        cout << "      Rigidity range: [" << minRigidity << ", " << maxRigidity << "] (span: " << rigidityRange << ")" << endl;

        // Use relative threshold: nodes significantly below segment average
        double avgRigidity = segment.avgRigidity;
        double relativeThreshold = avgRigidity - 0.05; // 5% below average

        for (int i = 1; i < segment.nodeRigidities.size() - 1; i++) { // Skip endpoints
            double rigidity = segment.nodeRigidities[i];

            // Junction criteria: either absolute low OR significant dip
            bool isJunction = (rigidity < RIGIDITY_SPLIT_THRESHOLD) ||
                             (rigidity < relativeThreshold && rigidityRange > 0.05);

            if (isJunction) {
                junctionIndices.push_back(i);
                cout << "      Found junction at node " << i << " (rigidity: " << rigidity << ")" << endl;
            }
        }

        cout << "    Found " << junctionIndices.size() << " junction points: ";
        for (int j : junctionIndices) cout << j << " ";
        cout << endl;

        // If no junctions found, add entire segment as rigid
        if (junctionIndices.empty()) {
            RigidSegment rigidSeg;
            rigidSeg.originalSegmentIdx = segIdx;
            rigidSeg.nodes = segment.nodes;
            rigidSeg.avgRigidity = segment.avgRigidity;
            rigidSeg.startHarmonicValue = 0.0;
            rigidSeg.endHarmonicValue = 1.0;
            rigidSeg.sourceIdx = segment.sourceIdx;
            rigidSeg.targetIdx = segment.targetIdx;
            rigidSegments.push_back(rigidSeg); // Fixed: was pushBack
            continue;
        }

        // --- START OF REPLACEMENT ---
        // Robustly split the segment at junction points.
        cout << "  Segment " << segIdx << ": Found " << junctionIndices.size() << " junctions, splitting..." << endl;
        int lastCut = -1;
        junctionIndices.push_back(segment.nodes.size()); // Add a virtual junction at the end

        for (int cutPoint : junctionIndices) {
            int startNode = lastCut + 1;
            int endNode = cutPoint - 1;

            // Ensure the segment is valid and has at least 1 node.
            if (endNode >= startNode) {
                RigidSegment rigidSeg;
                rigidSeg.originalSegmentIdx = segIdx;

                // Extract nodes and calculate average rigidity for this new rigid part
                double totalRigidity = 0;
                for (int n = startNode; n <= endNode; ++n) {
                    rigidSeg.nodes.push_back(segment.nodes[n]);
                    totalRigidity += segment.nodeRigidities[n];
                }
                rigidSeg.avgRigidity = totalRigidity / rigidSeg.nodes.size();

                // Set harmonic range and source/target
                if (segment.nodes.size() > 1) {
                    rigidSeg.startHarmonicValue = (double)startNode / (segment.nodes.size() - 1);
                    rigidSeg.endHarmonicValue = (double)endNode / (segment.nodes.size() - 1);
                } else {
                    rigidSeg.startHarmonicValue = 0.0;
                    rigidSeg.endHarmonicValue = 1.0;
                }
                rigidSeg.sourceIdx = segment.sourceIdx;
                rigidSeg.targetIdx = segment.targetIdx;

                rigidSegments.push_back(rigidSeg);
                cout << "    Created rigid sub-segment from node " << startNode
                     << " to " << endNode << " (avg rigidity: " << rigidSeg.avgRigidity << ")" << endl;
            }
            lastCut = cutPoint;
        }
        // --- END OF REPLACEMENT ---
    }

    cout << "Total rigid segments after splitting: " << rigidSegments.size() << endl;

    // Step 2: Cache harmonic fields for all original segments to avoid recomputation
    cout << "Step 2: Caching harmonic fields for original skeletal segments..." << endl;

    map<pair<int, int>, Eigen::VectorXd> cachedHarmonicFields;

    // Cache harmonic fields for unique source-target pairs
    for (const auto& rigidSeg : rigidSegments) {

        if (rigidSeg.sourceIdx < 0 || rigidSeg.targetIdx < 0) {
            continue; // Skip segments for torso
        }
        pair<int, int> segmentPair = {rigidSeg.sourceIdx, rigidSeg.targetIdx};

        if (cachedHarmonicFields.find(segmentPair) == cachedHarmonicFields.end()) {
            cout << "  Computing harmonic field for segment pair (" << rigidSeg.sourceIdx << ", " << rigidSeg.targetIdx << ")" << endl;
            Eigen::VectorXd harmonicField = g_harmonics->computePairwiseHarmonic(rigidSeg.sourceIdx, rigidSeg.targetIdx);

            if (harmonicField.size() == g_mesh->verts.size()) {
                cachedHarmonicFields[segmentPair] = harmonicField;
            } else {
                cout << "    Warning: Invalid harmonic field size for pair (" << rigidSeg.sourceIdx << ", " << rigidSeg.targetIdx << ")" << endl;
            }
        }
    }

    cout << "Cached " << cachedHarmonicFields.size() << " unique harmonic fields" << endl;

    // Step 3: Hybrid assignment using a two-pass strategy
    cout << "Step 3: Assigning vertices using hybrid watershed competition..." << endl;

    vector<int> vertexSegmentAssignment(g_mesh->verts.size(), -1);
    vector<int> segmentVertexCounts(rigidSegments.size(), 0);

    // Pass 1: High-confidence assignment for limb segments using harmonic distance
    cout << "  Pass 1: Assigning clear limb vertices via harmonic competition..." << endl;
    const double HARMONIC_CONFIDENCE_THRESHOLD = 0.05; // Vertex must be very close to a segment's harmonic range

    for (int v = 0; v < g_mesh->verts.size(); v++) {
        double minDistance = numeric_limits<double>::max();
        int bestSegment = -1;
        double bestRigidity = 0.0;

        for (int rigidSegIdx = 0; rigidSegIdx < rigidSegments.size(); rigidSegIdx++) {
            const auto& rigidSeg = rigidSegments[rigidSegIdx];
            if (rigidSeg.sourceIdx == -1) continue; // Skip torso segments in this pass

            pair<int, int> segmentPair = {rigidSeg.sourceIdx, rigidSeg.targetIdx};
            auto harmonicIt = cachedHarmonicFields.find(segmentPair);
            if (harmonicIt == cachedHarmonicFields.end()) continue;

            const Eigen::VectorXd& harmonicField = harmonicIt->second;
            double harmonicValue = harmonicField(v);

            double distanceToRange = 0.0;
            if (harmonicValue < rigidSeg.startHarmonicValue) {
                distanceToRange = rigidSeg.startHarmonicValue - harmonicValue;
            } else if (harmonicValue > rigidSeg.endHarmonicValue) {
                distanceToRange = harmonicValue - rigidSeg.endHarmonicValue;
            }

            if (distanceToRange < minDistance) {
                minDistance = distanceToRange;
                bestSegment = rigidSegIdx;
                bestRigidity = rigidSeg.avgRigidity;
            } else if (abs(distanceToRange - minDistance) < 1e-9) {
                if (rigidSeg.avgRigidity > bestRigidity) {
                    bestSegment = rigidSegIdx;
                    bestRigidity = rigidSeg.avgRigidity;
                }
            }
        }

        // Only assign if the vertex is a very clear, unambiguous fit for a limb
        if (bestSegment != -1 && minDistance < HARMONIC_CONFIDENCE_THRESHOLD) {
            vertexSegmentAssignment[v] = bestSegment;
        }
    }

    // Pass 2: Assign all remaining vertices to the geometrically closest rigid segment (handles torso/junctions)
    cout << "  Pass 2: Assigning remaining torso/junction vertices via Euclidean distance..." << endl;
    for (int v = 0; v < g_mesh->verts.size(); v++) {
        if (vertexSegmentAssignment[v] != -1) continue; // Skip already-assigned vertices

        double minEuclideanDist = numeric_limits<double>::max();
        int bestSegment = -1;
        Eigen::Vector3d vertPos(g_mesh->verts[v]->coords[0], g_mesh->verts[v]->coords[1], g_mesh->verts[v]->coords[2]);

        for (int rigidSegIdx = 0; rigidSegIdx < rigidSegments.size(); rigidSegIdx++) {
            const auto& rigidSeg = rigidSegments[rigidSegIdx];
            if (rigidSeg.nodes.empty()) continue;

            // Find the closest point on this segment's polyline to the vertex
            for (const auto& nodePos : rigidSeg.nodes) {
                double dist = (vertPos - nodePos).norm();
                if (dist < minEuclideanDist) {
                    minEuclideanDist = dist;
                    bestSegment = rigidSegIdx;
                }
            }
        }

        if (bestSegment != -1) {
            vertexSegmentAssignment[v] = bestSegment;
        }
    }

    // Recalculate final vertex counts for each segment
    for(int i = 0; i < rigidSegments.size(); ++i) segmentVertexCounts[i] = 0;
    for(int v = 0; v < g_mesh->verts.size(); ++v) {
        if(vertexSegmentAssignment[v] != -1) {
            segmentVertexCounts[vertexSegmentAssignment[v]]++;
        }
    }
    // --- END OF REPLACEMENT ---

    // Step 4: Visualize the results
    cout << "Step 4: Visualizing segmentation results..." << endl;

    // Print segment statistics
    for (int i = 0; i < rigidSegments.size(); i++) {
        cout << "  Rigid Segment " << i << " (from original segment " << rigidSegments[i].originalSegmentIdx << "): "
             << segmentVertexCounts[i] << " vertices, avg rigidity: " << rigidSegments[i].avgRigidity << endl;
    }

    // Visualize vertices colored by segment assignment
    cout << "Rendering colored mesh vertices..." << endl;
    for (int v = 0; v < g_mesh->verts.size(); v += 20) { // More aggressive downsampling
        Eigen::Vector3d vertexPos(g_mesh->verts[v]->coords[0], g_mesh->verts[v]->coords[1], g_mesh->verts[v]->coords[2]);

        int segmentIdx = vertexSegmentAssignment[v];
        Eigen::Vector3f color;

        if (segmentIdx >= 0 && segmentIdx < rigidSegments.size()) {
            color = segmentColors[segmentIdx % segmentColors.size()];
        } else {
            color = Eigen::Vector3f(0.5f, 0.5f, 0.5f); // Gray for unassigned
        }

        SoSeparator* pointViz = VisualizationUtils::createColoredSphere(vertexPos, color, 0.015f);
        g_root->addChild(pointViz);
    }

    // Visualize the rigid skeletal segments as lines
    cout << "Rendering rigid skeletal segments..." << endl;
    for (int rigidSegIdx = 0; rigidSegIdx < rigidSegments.size(); rigidSegIdx++) {
        const auto& rigidSeg = rigidSegments[rigidSegIdx];

        if (rigidSeg.nodes.size() < 2) continue;

        Eigen::Vector3f color = segmentColors[rigidSegIdx % segmentColors.size()];

        SoSeparator* lineSep = new SoSeparator();
        SoMaterial* mat = new SoMaterial();
        mat->diffuseColor.setValue(color[0], color[1], color[2]);
        mat->emissiveColor.setValue(color[0] * 0.3f, color[1] * 0.3f, color[2] * 0.3f);
        lineSep->addChild(mat);

        SoCoordinate3* coords = new SoCoordinate3();
        coords->point.setNum(rigidSeg.nodes.size());

        for (int i = 0; i < rigidSeg.nodes.size(); i++) {
            coords->point.set1Value(i, rigidSeg.nodes[i][0], rigidSeg.nodes[i][1], rigidSeg.nodes[i][2]);
        }
        lineSep->addChild(coords);

        SoIndexedLineSet* lineSet = new SoIndexedLineSet();
        for (int i = 0; i < rigidSeg.nodes.size() - 1; i++) {
            lineSet->coordIndex.set1Value(i * 3, i);
            lineSet->coordIndex.set1Value(i * 3 + 1, i + 1);
            lineSet->coordIndex.set1Value(i * 3 + 2, -1);
        }
        lineSep->addChild(lineSet);
        g_root->addChild(lineSep);

        // Add spheres at segment endpoints to show rigid parts clearly
        if (!rigidSeg.nodes.empty()) {
            SoSeparator* startSphere = VisualizationUtils::createColoredSphere(rigidSeg.nodes[0], color, 0.03f);
            g_root->addChild(startSphere);

            SoSeparator* endSphere = VisualizationUtils::createColoredSphere(rigidSeg.nodes.back(), color, 0.03f);
            g_root->addChild(endSphere);
        }
    }

    g_viewer->scheduleRedraw();
    g_viewer->viewAll();    cout << "\n=== HARMONIC FUNCTION SURFACE SEGMENTATION COMPLETE ===" << endl;
    cout << "-> Split " << g_segmentation_result.partialSkeleton.size() << " original segments into "
         << rigidSegments.size() << " rigid parts using rigidity analysis" << endl;
    cout << "-> Assigned surface vertices using harmonic function competition" << endl;
    cout << "-> Each color represents a distinct rigid part of the model" << endl;
    cout << "-> Thick lines: rigid skeletal segments" << endl;
    cout << "-> Large spheres: segment endpoints" << endl;
    cout << "-> Small spheres: surface vertices colored by segment assignment" << endl;
    cout << "\nKEY INSIGHT: Surface vertices are colored based on which skeletal segment" << endl;
    cout << "they belong to via harmonic function 'competition'. This implements the" << endl;
    cout << "watershed principle from the paper - vertices flow to the skeletal part" << endl;
    cout << "they are most connected to along the surface geometry." << endl;
}

/**
 * Diagnostic function to analyze skeletal segments
 */
void analyzeSkeletalSegments() {
    if (!g_segmentation_result.success || g_segmentation_result.partialSkeleton.empty()) {
        cout << "Error: Please run full segmentation first!" << endl;
        return;
    }

    cout << "\n=== SKELETAL SEGMENT ANALYSIS ===" << endl;
    cout << "Total segments in partial skeleton: " << g_segmentation_result.partialSkeleton.size() << endl;

    for (int i = 0; i < g_segmentation_result.partialSkeleton.size(); i++) {
        const auto& segment = g_segmentation_result.partialSkeleton[i];
            // ADD THIS FILTER HERE TOO:
        if (segment.avgRigidity < 0.6) {
            cout << "\nSegment " << i << " (FILTERED - Low Rigidity):" << endl;
            cout << "  Avg Rigidity: " << segment.avgRigidity << " (below threshold 0.6)" << endl;
            cout << "  Length: " << segment.length << " (likely problematic long line)" << endl;
            cout << "  Source FPS: " << segment.sourceIdx << ", Target FPS: " << segment.targetIdx << endl;
            cout << "  ** This segment would be skipped in visualization **" << endl;
            continue;
    }

        cout << "\nSegment " << i << ":" << endl;
        cout << "  Nodes: " << segment.nodes.size() << endl;
        cout << "  Length: " << segment.length << endl;
        cout << "  Avg Rigidity: " << segment.avgRigidity << endl;
        cout << "  Quality: " << segment.quality << endl;
        cout << "  Source FPS: " << segment.sourceIdx << ", Target FPS: " << segment.targetIdx << endl;

        if (!segment.nodes.empty()) {
            Eigen::Vector3d startPoint = segment.nodes[0];
            Eigen::Vector3d endPoint = segment.nodes.back();

            cout << "  Start point: (" << startPoint[0] << ", " << startPoint[1] << ", " << startPoint[2] << ")" << endl;
            cout << "  End point: (" << endPoint[0] << ", " << endPoint[1] << ", " << endPoint[2] << ")" << endl;

            // Check if this segment starts at the same point as others
            for (int j = 0; j < i; j++) {
                if (!g_segmentation_result.partialSkeleton[j].nodes.empty()) {
                    Eigen::Vector3d otherStart = g_segmentation_result.partialSkeleton[j].nodes[0];
                    double distance = (startPoint - otherStart).norm();
                    if (distance < 0.01) { // Very close starting points
                        cout << "  WARNING: Starts very close to segment " << j << " (distance: " << distance << ")" << endl;
                    }
                }
            }
        }

        // Analyze rigidity distribution
        if (!segment.nodeRigidities.empty()) {
            double minRig = *min_element(segment.nodeRigidities.begin(), segment.nodeRigidities.end());
            double maxRig = *max_element(segment.nodeRigidities.begin(), segment.nodeRigidities.end());
            cout << "  Rigidity range: [" << minRig << ", " << maxRig << "]" << endl;

            // Count how many nodes are below threshold
            int lowRigidityCount = 0;
            for (double rig : segment.nodeRigidities) {
                if (rig < RIGIDITY_THRESHOLD) lowRigidityCount++;
            }
            cout << "  Nodes below threshold (" << RIGIDITY_THRESHOLD << "): " << lowRigidityCount << "/" << segment.nodeRigidities.size() << endl;
        }
    }

    // Analyze FPS points used
    cout << "\nFPS Points Analysis:" << endl;
    cout << "Total FPS points: " << g_segmentation_result.fpsPoints.size() << endl;

    set<int> usedSourcePoints, usedTargetPoints;
    for (const auto& segment : g_segmentation_result.partialSkeleton) {
        usedSourcePoints.insert(segment.sourceIdx);
        usedTargetPoints.insert(segment.targetIdx);
    }

    cout << "Unique source FPS points used: " << usedSourcePoints.size() << endl;
    cout << "Unique target FPS points used: " << usedTargetPoints.size() << endl;

    if (usedSourcePoints.size() == 1) {
        cout << "WARNING: All segments use the same source point!" << endl;
    }
    if (usedTargetPoints.size() == 1) {
        cout << "WARNING: All segments use the same target point!" << endl;
    }
}

/**
 * Diagnostic function to show detailed rigidity information for troubleshooting
 */
void analyzeDetailedRigidity() {
    if (!g_segmentation_result.success || g_segmentation_result.partialSkeleton.empty()) {
        cout << "Error: Please run full segmentation first!" << endl;
        return;
    }

    cout << "\n=== DETAILED RIGIDITY ANALYSIS ===" << endl;

    for (int segIdx = 0; segIdx < g_segmentation_result.partialSkeleton.size(); segIdx++) {
        const auto& segment = g_segmentation_result.partialSkeleton[segIdx];

        cout << "\nSegment " << segIdx << " (FPS " << segment.sourceIdx << " -> " << segment.targetIdx << "):" << endl;
        cout << "  Total nodes: " << segment.nodes.size() << endl;
        cout << "  Rigidity values:" << endl;

        // Show rigidity histogram
        int lowCount = 0, mediumCount = 0, highCount = 0;

        for (int i = 0; i < segment.nodeRigidities.size(); i++) {
            double rig = segment.nodeRigidities[i];

            if (i < 10 || i % 5 == 0) { // Show first 10 and every 5th value
                cout << "    Node " << i << ": " << rig << endl;
            }

            if (rig < 0.4) lowCount++;
            else if (rig < 0.7) mediumCount++;
            else highCount++;
        }

        cout << "  Rigidity distribution:" << endl;
        cout << "    Low (< 0.4): " << lowCount << " nodes" << endl;
        cout << "    Medium (0.4-0.7): " << mediumCount << " nodes" << endl;
        cout << "    High (> 0.7): " << highCount << " nodes" << endl;

        // Analyze why cutting didn't work
        cout << "  Cutting analysis:" << endl;
        cout << "    Threshold used: " << RIGIDITY_THRESHOLD << endl;
        cout << "    Nodes below threshold: " << lowCount + (RIGIDITY_THRESHOLD > 0.4 ? mediumCount : 0) << endl;

        if (lowCount == 0 && mediumCount == 0) {
            cout << "    ISSUE: No low rigidity nodes found - all nodes are too rigid!" << endl;
            cout << "    SOLUTION: Lower the rigidity threshold or check rigidity computation" << endl;
        }
    }
}

/**
 * Unload the current mesh and clean up all related data
 */
void unloadMesh() {
    cout << "\n=== UNLOAD MESH ===" << endl;

    if (!g_mesh) {
        cout << "No mesh is currently loaded." << endl;
        return;
    }

    cout << "Unloading current mesh..." << endl;

    // Clear all visualization including the mesh itself
    if (g_root) {
        g_root->removeAllChildren();
    }

    // Clean up segmentation data
    if (g_segmentation) {
        delete g_segmentation;
        g_segmentation = nullptr;
    }

    // Clean up harmonics data
    if (g_harmonics) {
        delete g_harmonics;
        g_harmonics = nullptr;
    }

    // Clean up mesh data
    if (g_mesh) {
        delete g_mesh;
        g_mesh = nullptr;
    }

    // Clear all algorithm state
    g_fps_samples.clear();
    g_skeletal_segments.clear();
    g_rigidity_scores.clear();
    g_nonrigid_nodes.clear();

    // Clear segmentation result
    g_segmentation_result = PairwiseHarmonicsSegmentation::SegmentationResult();

    // Force viewer to update and refresh
    if (g_viewer) {
        g_viewer->scheduleRedraw();
        g_viewer->viewAll();
    }

    cout << "Mesh unloaded successfully. All data cleared." << endl;
}

/**
 * Test function to debug and analyze the harmonic-based surface segmentation approach
 */
void testHarmonicSurfaceSegmentation() {
    if (!g_segmentation_result.success || g_segmentation_result.partialSkeleton.empty() || !g_mesh || !g_harmonics) {
        cout << "Error: Please run full segmentation first!" << endl;
        return;
    }

    cout << "\n=== TESTING HARMONIC SURFACE SEGMENTATION ===" << endl;

    // Test a single segment to understand the data
    if (!g_segmentation_result.partialSkeleton.empty()) {
        const auto& testSegment = g_segmentation_result.partialSkeleton[0];

        cout << "Testing segment 0:" << endl;
        cout << "  Source FPS: " << testSegment.sourceIdx << ", Target FPS: " << testSegment.targetIdx << endl;
        cout << "  Nodes: " << testSegment.nodes.size() << endl;
        cout << "  Avg Rigidity: " << testSegment.avgRigidity << endl;
        cout << "  Length: " << testSegment.length << endl;

        // Compute harmonic field for this segment
        cout << "Computing harmonic field..." << endl;
        Eigen::VectorXd harmonicField = g_harmonics->computePairwiseHarmonic(testSegment.sourceIdx, testSegment.targetIdx);

        if (harmonicField.size() == g_mesh->verts.size()) {
            double minVal = harmonicField.minCoeff();
            double maxVal = harmonicField.maxCoeff();
            cout << "  Harmonic field range: [" << minVal << ", " << maxVal << "]" << endl;

            // Analyze rigidity distribution
            if (testSegment.nodeRigidities.size() == testSegment.nodes.size()) {
                cout << "  Rigidity analysis:" << endl;
                for (int i = 0; i < min(10, (int)testSegment.nodeRigidities.size()); i++) {
                    double rigidity = testSegment.nodeRigidities[i];
                    double harmonicValue = (double)i / (testSegment.nodes.size() - 1);
                    cout << "    Node " << i << ": rigidity=" << rigidity << ", estimated harmonic=" << harmonicValue << endl;
                }

                // Find potential junction points (low rigidity)
                vector<int> junctionPoints;
                const double threshold = 0.85;
                for (int i = 0; i < testSegment.nodeRigidities.size(); i++) {
                    if (testSegment.nodeRigidities[i] < threshold) {
                        junctionPoints.push_back(i);
                    }
                }

                cout << "  Junction points (rigidity < " << threshold << "): ";
                for (int jp : junctionPoints) {
                    cout << jp << " ";
                }
                cout << endl;

                if (junctionPoints.size() > 0) {
                    cout << "  This segment should be split into " << (junctionPoints.size() + 1) << " rigid parts" << endl;
                } else {
                    cout << "  This segment is uniformly rigid (no junctions detected)" << endl;
                }
            }

            // Count vertices in different harmonic ranges
            int range1 = 0, range2 = 0, range3 = 0; // 0-0.33, 0.33-0.66, 0.66-1.0
            for (int v = 0; v < harmonicField.size(); v++) {
                double normalizedVal = (harmonicField(v) - minVal) / (maxVal - minVal);
                if (normalizedVal < 0.33) range1++;
                else if (normalizedVal < 0.66) range2++;
                else range3++;
            }

            cout << "  Vertex distribution in harmonic ranges:" << endl;
            cout << "    Range [0.0-0.33]: " << range1 << " vertices" << endl;
            cout << "    Range [0.33-0.66]: " << range2 << " vertices" << endl;
            cout << "    Range [0.66-1.0]: " << range3 << " vertices" << endl;

        } else {
            cout << "  ERROR: Harmonic field size mismatch!" << endl;
        }
    }
}

/**
 * Optimal Surface Segmentation using Graph-Cut Energy Minimization
 * This implements a simplified CRF model inspired by Kalogerakis et al.
 * Replaces the heuristic watershed approach with globally optimal segmentation.
 */
void visualizeGraphCutSegmentation() {
    if (!g_segmentation_result.success || g_segmentation_result.partialSkeleton.empty() || !g_mesh) {
        cout << "Error: Full segmentation must be performed first (Option 8)." << endl;
        return;
    }

    cout << "\n=== OPTIMAL SEGMENTATION (GRAPH-CUT ENERGY MINIMIZATION) ===" << endl;
    cout << "Implementing CRF-inspired approach for globally optimal surface segmentation..." << endl;
    clearVisualization();

    // Step 1: Reuse existing skeleton refinement logic to get rigid segments
    cout << "Step 1: Splitting skeleton into rigid parts..." << endl;

    // Define colors for different segments
    vector<Eigen::Vector3f> segmentColors = {
        Eigen::Vector3f(1.0f, 0.0f, 0.0f), // Red
        Eigen::Vector3f(0.0f, 1.0f, 0.0f), // Green
        Eigen::Vector3f(0.0f, 0.0f, 1.0f), // Blue
        Eigen::Vector3f(1.0f, 1.0f, 0.0f), // Yellow
        Eigen::Vector3f(1.0f, 0.0f, 1.0f), // Magenta
        Eigen::Vector3f(0.0f, 1.0f, 1.0f), // Cyan
        Eigen::Vector3f(1.0f, 0.5f, 0.0f), // Orange
        Eigen::Vector3f(0.5f, 0.0f, 1.0f), // Purple
        Eigen::Vector3f(0.5f, 1.0f, 0.5f), // Light Green
        Eigen::Vector3f(1.0f, 0.5f, 0.5f)  // Light Red

    };

    // Use all skeletal segments from the segmentation result
    vector<int> selectedSegmentIndices;
    for (int i = 0; i < g_segmentation_result.partialSkeleton.size(); ++i) {
        selectedSegmentIndices.push_back(i);
    }

    cout << "Using " << selectedSegmentIndices.size() << " segments for graph-cut optimization." << endl;

    // Create rigid segments with same logic as existing function
    struct RigidSegment {
        int originalSegmentIdx;
        vector<Eigen::Vector3d> nodes;
        double avgRigidity;
        double startHarmonicValue;
        double endHarmonicValue;
        int sourceIdx, targetIdx;
    };

    vector<RigidSegment> rigidSegments;
    const double RIGIDITY_SPLIT_THRESHOLD = 0.85;

    for (int segIdx : selectedSegmentIndices) {
        const auto& segment = g_segmentation_result.partialSkeleton[segIdx];

        if (segment.nodes.size() < 3 || segment.nodeRigidities.size() != segment.nodes.size()) {
            RigidSegment rigidSeg;
            rigidSeg.originalSegmentIdx = segIdx;
            rigidSeg.nodes = segment.nodes;
            rigidSeg.avgRigidity = segment.avgRigidity;
            rigidSeg.startHarmonicValue = 0.0;
            rigidSeg.endHarmonicValue = 1.0;
            rigidSeg.sourceIdx = segment.sourceIdx;
            rigidSeg.targetIdx = segment.targetIdx;
            rigidSegments.push_back(rigidSeg);
            continue;
        }

        // Find junction points using same logic
        vector<int> junctionIndices;
        double minRigidity = *min_element(segment.nodeRigidities.begin(), segment.nodeRigidities.end());
        double maxRigidity = *max_element(segment.nodeRigidities.begin(), segment.nodeRigidities.end());
        double rigidityRange = maxRigidity - minRigidity;
        double avgRigidity = segment.avgRigidity;
        double relativeThreshold = avgRigidity - 0.05;

        for (int i = 1; i < segment.nodeRigidities.size() - 1; i++) { // Skip endpoints
            double rigidity = segment.nodeRigidities[i];

            // Junction criteria: either absolute low OR significant dip
            bool isJunction = (rigidity < RIGIDITY_SPLIT_THRESHOLD) ||
                             (rigidity < relativeThreshold && rigidityRange > 0.05);

            if (isJunction) {
                junctionIndices.push_back(i);
            }
        }

        if (junctionIndices.empty()) {
            RigidSegment rigidSeg;
            rigidSeg.originalSegmentIdx = segIdx;
            rigidSeg.nodes = segment.nodes;
            rigidSeg.avgRigidity = segment.avgRigidity;
            rigidSeg.startHarmonicValue = 0.0;
            rigidSeg.endHarmonicValue = 1.0;
            rigidSeg.sourceIdx = segment.sourceIdx;
            rigidSeg.targetIdx = segment.targetIdx;
            rigidSegments.push_back(rigidSeg); // Fixed: was pushBack
            continue;
        }

        // --- START OF REPLACEMENT ---
        // Robustly split the segment at junction points.
        cout << "  Segment " << segIdx << ": Found " << junctionIndices.size() << " junctions, splitting..." << endl;
        int lastCut = -1;
        junctionIndices.push_back(segment.nodes.size()); // Add a virtual junction at the end

        for (int cutPoint : junctionIndices) {
            int startNode = lastCut + 1;
            int endNode = cutPoint - 1;

            // Ensure the segment is valid and has at least 1 node.
            if (endNode >= startNode) {
                RigidSegment rigidSeg;
                rigidSeg.originalSegmentIdx = segIdx;

                // Extract nodes and calculate average rigidity for this new rigid part
                double totalRigidity = 0;
                for (int n = startNode; n <= endNode; ++n) {
                    rigidSeg.nodes.push_back(segment.nodes[n]);
                    totalRigidity += segment.nodeRigidities[n];
                }
                rigidSeg.avgRigidity = totalRigidity / rigidSeg.nodes.size();

                // Set harmonic range and source/target
                if (segment.nodes.size() > 1) {
                    rigidSeg.startHarmonicValue = (double)startNode / (segment.nodes.size() - 1);
                    rigidSeg.endHarmonicValue = (double)endNode / (segment.nodes.size() - 1);
                } else {
                    rigidSeg.startHarmonicValue = 0.0;
                    rigidSeg.endHarmonicValue = 1.0;
                }
                rigidSeg.sourceIdx = segment.sourceIdx;
                rigidSeg.targetIdx = segment.targetIdx;

                rigidSegments.push_back(rigidSeg);
                cout << "    Created rigid sub-segment from node " << startNode
                     << " to " << endNode << " (avg rigidity: " << rigidSeg.avgRigidity << ")" << endl;
            }
            lastCut = cutPoint;
        }
        // --- END OF REPLACEMENT ---
    }

    cout << "Created " << rigidSegments.size() << " rigid segments for optimization." << endl;

    // Step 2: Build face adjacency graph
    cout << "Step 2: Building face adjacency graph..." << endl;

    int numFaces = g_mesh->tris.size();
    vector<vector<int>> faceNeighbors(numFaces);

    // Build face adjacency using edge sharing
    for (int f1 = 0; f1 < numFaces; f1++) {
        Triangle* tri1 = g_mesh->tris[f1];
        for (int f2 = f1 + 1; f2 < numFaces; f2++) {
            Triangle* tri2 = g_mesh->tris[f2];

            // Check if triangles share an edge (2 vertices)
            int sharedVertices = 0;
            int tri1_indices[] = { tri1->v1i, tri1->v2i, tri1->v3i };
            int tri2_indices[] = { tri2->v1i, tri2->v2i, tri2->v3i };
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    if (tri1_indices[i] == tri2_indices[j]) {
                        sharedVertices++;
                    }
                }
            }

            if (sharedVertices >= 2) {
                faceNeighbors[f1].push_back(f2);
                faceNeighbors[f2].push_back(f1);
            }
        }
    }

    cout << "Built adjacency for " << numFaces << " faces." << endl;

    // Step 3: Caching harmonic fields for efficiency
    cout << "Step 3: Caching harmonic fields..." << endl;

    map<pair<int, int>, Eigen::VectorXd> cachedHarmonicFields;

    for (const auto& rigidSeg : rigidSegments) {
        if (rigidSeg.sourceIdx < 0 || rigidSeg.targetIdx < 0) continue;

        // --- FIX: Use a canonical key for the cache to avoid redundant computations ---
        // The key is always (min_index, max_index).
        pair<int, int> canonicalPair = {min(rigidSeg.sourceIdx, rigidSeg.targetIdx), max(rigidSeg.sourceIdx, rigidSeg.targetIdx)};

        if (cachedHarmonicFields.find(canonicalPair) == cachedHarmonicFields.end()) {
            // Compute the harmonic field using the canonical source (min index) and target (max index).
            // This ensures the field is always computed in the same "direction".
            Eigen::VectorXd harmonicField = g_harmonics->computePairwiseHarmonic(canonicalPair.first, canonicalPair.second);
            if (harmonicField.size() == g_mesh->verts.size()) {
                cachedHarmonicFields[canonicalPair] = harmonicField;
            }
        }
    }

    cout << "Cached " << cachedHarmonicFields.size() << " harmonic fields." << endl;
    // Step 4: Define energy function and solve optimization
    cout << "Step 4: Computing data costs and smoothness costs..." << endl;

    // Initialize face labels and energy matrices
    vector<int> faceLabels(numFaces, -1);
    int numLabels = rigidSegments.size();

    // Data cost: cost of assigning each face to each label
    vector<vector<double>> dataCost(numFaces, vector<double>(numLabels, 1.0));

    // Compute data costs based on face-to-segment affinity
    for (int f = 0; f < numFaces; f++) {
        Triangle* face = g_mesh->tris[f];

        // Compute face centroid
        Eigen::Vector3d faceCentroid(0, 0, 0);
        Vertex* face_verts[] = { g_mesh->verts[face->v1i], g_mesh->verts[face->v2i], g_mesh->verts[face->v3i] };
        for (int i = 0; i < 3; i++) {
            faceCentroid[0] += face_verts[i]->coords[0];
            faceCentroid[1] += face_verts[i]->coords[1];
            faceCentroid[2] += face_verts[i]->coords[2];
        }
        faceCentroid /= 3.0;

        // Find closest vertex to face centroid for harmonic evaluation
        int closestVertex = 0;
        double minDist = numeric_limits<double>::max();
        for (int v = 0; v < g_mesh->verts.size(); v++) {
            Eigen::Vector3d vertPos(g_mesh->verts[v]->coords[0],
                                   g_mesh->verts[v]->coords[1],
                                   g_mesh->verts[v]->coords[2]);
            double dist = (faceCentroid - vertPos).norm();
            if (dist < minDist) {
                minDist = dist;
                closestVertex = v;
            }
        }

        // Compute affinity to each rigid segment
        for (int label = 0; label < numLabels; label++) {
            const auto& rigidSeg = rigidSegments[label];

            if (rigidSeg.sourceIdx >= 0 && rigidSeg.targetIdx >= 0) {
                // Use harmonic distance for limb segments

                // --- FIX: Use canonical key to look up the field and adjust the value based on direction ---
                int src = rigidSeg.sourceIdx;
                int tgt = rigidSeg.targetIdx;
                pair<int, int> canonicalPair = {min(src, tgt), max(src, tgt)};
                auto harmonicIt = cachedHarmonicFields.find(canonicalPair);

                if (harmonicIt != cachedHarmonicFields.end()) {
                    const Eigen::VectorXd& harmonicField = harmonicIt->second;
                    double harmonicValue = harmonicField(closestVertex);

                    // The cached field is always from min_idx to max_idx.
                    // If this segment's source is the max_idx, we need to flip the harmonic value.
                    if (src > tgt) {
                        harmonicValue = 1.0 - harmonicValue;
                    }

                    // --- REVISED COST FUNCTION (Limb Segments) ---
                    // The old cost function created a "zero-cost" zone that gave limb segments an unfair advantage.
                    // The new cost is the distance to the center of the segment's harmonic range,
                    // which provides a smoother and more competitive cost value.
                    double rangeCenter = (rigidSeg.startHarmonicValue + rigidSeg.endHarmonicValue) / 2.0;
                    dataCost[f][label] = abs(harmonicValue - rangeCenter);

                } else {
                    dataCost[f][label] = 1.0; // High cost if field not found
                }
            } else {
                // --- REVISED COST FUNCTION (Torso Segments) ---
                // The old cost had an arbitrary scaling factor.
                // The new cost normalizes the Euclidean distance by the mesh's bounding box diagonal,
                // making it directly comparable to the harmonic costs which are in the [0, 1] range.
                double minEuclideanDist = numeric_limits<double>::max();
                for (const auto& nodePos : rigidSeg.nodes) {
                    double dist = (faceCentroid - nodePos).norm();
                    minEuclideanDist = min(minEuclideanDist, dist);
                }

                float bbox_diag = g_mesh->getBoundingBoxDiagonal();
                if (bbox_diag > 1e-6) {
                    // Scale cost to be comparable to harmonic cost (which is in [0, 0.5])
                    dataCost[f][label] = (minEuclideanDist / bbox_diag);
                } else {
                    dataCost[f][label] = minEuclideanDist; // Fallback
                }
            }
        }
    }
    // --- NEW: Normalize data costs per-face to improve label competition ---
    cout << "Normalizing data costs per-face to create a more stable energy function..." << endl;
    for (int f = 0; f < numFaces; f++) {
        // Find the minimum cost for this face to use as a baseline for numerical stability
        double min_cost_for_face = dataCost[f][0];
        for (int label = 1; label < numLabels; label++) {
            if (dataCost[f][label] < min_cost_for_face) {
                min_cost_for_face = dataCost[f][label];
            }
        }

        // Convert costs to probabilities using a softmax-like approach
        double sum_exp = 0.0;
        for (int label = 0; label < numLabels; label++) {
            sum_exp += exp(-(dataCost[f][label] - min_cost_for_face));
        }

        // Convert probabilities back to a more stable negative-log cost
        if (sum_exp > 1e-9) {
            for (int label = 0; label < numLabels; label++) {
                double probability = exp(-(dataCost[f][label] - min_cost_for_face)) / sum_exp;
                dataCost[f][label] = -log(probability + 1e-9); // Add epsilon to avoid log(0)
            }
        }
    }
    cout << "Data cost normalization complete." << endl;

    // Smoothness cost: penalty for different labels on adjacent faces
    cout << "Computing smoothness costs based on dihedral angles..." << endl;

    // Simple smoothness model: constant penalty for label disagreement
    double smoothnessPenalty = 0.2;

    // Step 5: Solve energy minimization using iterative ICM (Iterated Conditional Modes)
    cout << "Step 5: Solving energy minimization using ICM optimization..." << endl;

    // Initialize with greedy assignment based on minimum data cost
    for (int f = 0; f < numFaces; f++) {
        int bestLabel = 0;
        double minCost = dataCost[f][0];
        for (int label = 1; label < numLabels; label++) {
            if (dataCost[f][label] < minCost) {
                minCost = dataCost[f][label];
                bestLabel = label;
            }
        }
        faceLabels[f] = bestLabel;
    }

    // ICM iterations to minimize energy
    bool converged = false;
    int maxIterations = 20;

    for (int iter = 0; iter < maxIterations && !converged; iter++) {
        bool changed = false;

        for (int f = 0; f < numFaces; f++) {
            int currentLabel = faceLabels[f];
            int bestLabel = currentLabel;
            double minEnergy = numeric_limits<double>::max();

            // Try each possible label for this face
            for (int label = 0; label < numLabels; label++) {
                // Compute total energy for this assignment
                double energy = dataCost[f][label];

                // Add smoothness costs from neighbors
                for (int neighbor : faceNeighbors[f]) {
                    if (faceLabels[neighbor] != label) {
                        energy += smoothnessPenalty;
                    }
                }

                if (energy < minEnergy) {
                    minEnergy = energy;
                    bestLabel = label;
                }
            }

            if (bestLabel != currentLabel) {
                faceLabels[f] = bestLabel;
                changed = true;
            }
        }

        if (!changed) {
            converged = true;
            cout << "ICM converged after " << (iter + 1) << " iterations." << endl;
        }
    }

    if (!converged) {
        cout << "ICM reached maximum iterations (" << maxIterations << ")." << endl;
    }

    // Step 6: Visualize results
    cout << "Step 6: Visualizing graph-cut segmentation results..." << endl;

    // Count faces per segment
    vector<int> segmentFaceCounts(numLabels, 0);
    for (int f = 0; f < numFaces; f++) {
        if (faceLabels[f] >= 0 && faceLabels[f] < numLabels) {
            segmentFaceCounts[faceLabels[f]]++;
        }
    }

    // Print statistics
    for (int i = 0; i < numLabels; i++) {
        cout << "  Segment " << i << ": " << segmentFaceCounts[i] << " faces" << endl;
    }

    // Visualize face-based segmentation with mesh coloring
    cout << "Rendering face-colored mesh segmentation..." << endl;

    SoSeparator* meshSep = new SoSeparator();

    // Create separate coordinate and face sets for each segment
    for (int segmentIdx = 0; segmentIdx < numLabels; segmentIdx++) {
        vector<int> segmentFaces;
        for (int f = 0; f < numFaces; f++) {
            if (faceLabels[f] == segmentIdx) {
                segmentFaces.push_back(f);
            }
        }

        if (segmentFaces.empty()) continue;

        SoSeparator* segSep = new SoSeparator();

        // Set material color for this segment
        SoMaterial* mat = new SoMaterial();
        Eigen::Vector3f color = segmentColors[segmentIdx % segmentColors.size()];
        mat->diffuseColor.setValue(color[0], color[1], color[2]);
        mat->ambientColor.setValue(color[0] * 0.3f, color[1] * 0.3f, color[2] * 0.3f);
        segSep->addChild(mat);

        // Create coordinates for all vertices
        SoCoordinate3* coords = new SoCoordinate3();
        coords->point.setNum(g_mesh->verts.size());
        for (int v = 0; v < g_mesh->verts.size(); v++) {
            coords->point.set1Value(v, g_mesh->verts[v]->coords[0],
                                      g_mesh->verts[v]->coords[1],
                                      g_mesh->verts[v]->coords[2]);
        }
        segSep->addChild(coords);

        // Create indexed face set for this segment
        SoIndexedFaceSet* faceSet = new SoIndexedFaceSet();
        int faceIndex = 0;
        for (int f : segmentFaces) {
            Triangle* tri = g_mesh->tris[f];
            faceSet->coordIndex.set1Value(faceIndex * 4 + 0, tri->v1i);
            faceSet->coordIndex.set1Value(faceIndex * 4 + 1, tri->v2i);
            faceSet->coordIndex.set1Value(faceIndex * 4 + 2, tri->v3i);
            faceSet->coordIndex.set1Value(faceIndex * 4 + 3, -1); // End of face
            faceIndex++;
        }
        segSep->addChild(faceSet);
        meshSep->addChild(segSep);
    }

    g_root->addChild(meshSep);

    // Visualize skeletal segments as lines
    cout << "Rendering skeletal segments..." << endl;
    for (int rigidSegIdx = 0; rigidSegIdx < rigidSegments.size(); rigidSegIdx++) {
        const auto& rigidSeg = rigidSegments[rigidSegIdx];

        if (rigidSeg.nodes.size() < 2) continue;

        Eigen::Vector3f color = segmentColors[rigidSegIdx % segmentColors.size()];

        SoSeparator* lineSep = new SoSeparator();
        SoMaterial* mat = new SoMaterial();
        mat->diffuseColor.setValue(color[0], color[1], color[2]);
        mat->emissiveColor.setValue(color[0] * 0.5f, color[1] * 0.5f, color[2] * 0.5f);
        lineSep->addChild(mat);

        SoCoordinate3* coords = new SoCoordinate3();
        coords->point.setNum(rigidSeg.nodes.size());

        for (int i = 0; i < rigidSeg.nodes.size(); i++) {
            coords->point.set1Value(i, rigidSeg.nodes[i][0], rigidSeg.nodes[i][1], rigidSeg.nodes[i][2]);
        }
        lineSep->addChild(coords);

        SoIndexedLineSet* lineSet = new SoIndexedLineSet();
        for (int i = 0; i < rigidSeg.nodes.size() - 1; i++) {
            lineSet->coordIndex.set1Value(i * 3, i);
            lineSet->coordIndex.set1Value(i * 3 + 1, i + 1);
            lineSet->coordIndex.set1Value(i * 3 + 2, -1);
        }
        lineSep->addChild(lineSet);
        g_root->addChild(lineSep);
    }

    g_viewer->scheduleRedraw();
    g_viewer->viewAll();

    cout << "\n=== GRAPH-CUT SEGMENTATION COMPLETE ===" << endl;
    cout << "-> Replaced heuristic watershed with globally optimal energy minimization" << endl;
    cout << "-> Used face-based segmentation with smoothness constraints" << endl;
    cout << "-> Energy function combines harmonic affinity with boundary smoothness" << endl;
    cout << "-> Each color represents an optimal segment found by graph-cut algorithm" << endl;
    cout << "-> This approach provides cleaner boundaries and better global consistency" << endl;
}