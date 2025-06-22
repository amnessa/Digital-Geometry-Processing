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
const double RIGIDITY_THRESHOLD = 0.9;  // Threshold for classifying rigid vs non-rigid nodes (significantly lowered for better junction detection)
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
void visualizeMeshSegmentationClusters();
void visualizeSkeletalKMeansClustering();
void analyzeSkeletalSegments();
void analyzeDetailedRigidity();
void analyzeDetailedRigidity();

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
    cout << "======================================" << endl;
    cout << "1. Load Mesh" << endl;
    cout << "2. Test Cotangent Laplacian" << endl;
    cout << "3. Test Farthest Point Sampling" << endl;
    cout << "4. Test Pairwise Harmonics" << endl;
    cout << "5. Test Iso-curve Extraction" << endl;    cout << "6. Test Rigidity Analysis" << endl;
    cout << "7. Perform Full Segmentation" << endl;
    cout << "8. Clear Visualization" << endl;    cout << "--- ENHANCED ANALYSIS ---" << endl;
    cout << "9. Improved Iso-curve Extraction (Dense Bracelets)" << endl;
    cout << "10. Enhanced Rigidity Analysis (All Nodes)" << endl;
    cout << "11. Visualize All Rigidity Points (Color-coded)" << endl;
    cout << "12. Interactive Rigidity Threshold Testing" << endl;    cout << "--- SEGMENTATION VISUALIZATION ---" << endl;
    cout << "18. Visualize Mesh Components (Colored Vertex Clusters)" << endl;
    cout << "19. Skeletal K-means Clustering (Euclidean Distance)" << endl;
    cout << "20. Analyze Skeletal Segments (Diagnostic)" << endl;
    cout << "21. Detailed Rigidity Analysis (Debug Cutting)" << endl;
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
        cin.ignore(numeric_limits<streamsize>::max(), '\n');switch (choice) {
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
                break;            case 16:
                testEnhancedDescriptors();
                break;
            case 17:
                testMeshAnalysis();
                break;
            case 18:
                visualizeMeshSegmentationClusters();
                break;
            case 19:
                visualizeSkeletalKMeansClustering();
                break;        case 20:
            analyzeSkeletalSegments();
            break;
        case 21:
            analyzeDetailedRigidity();
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
            color = Eigen::Vector3f(0.0f, 0.0f, 1.0f); // Blue for last
        } else {
            color = Eigen::Vector3f(0.0f, 1.0f, 0.0f); // Green for others
        }

        SoSeparator* pointViz = VisualizationUtils::createColoredSphere(pos, color, 0.015f);
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
        cout << "  Mean distance: " << heatDist.mean() << endl;        cout << "\nMethod 2: Dijkstra-based Geodesics" << endl;
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
        Eigen::Vector3f(0.0f, 0.0f, 1.0f), // Blue
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

                SoSeparator* pointViz = VisualizationUtils::createColoredSphere(vertexPos, color, 0.008f);
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
 * Visualize mesh segmentation using skeletal-based K-means clustering
 * Generate points along skeletal segments as centroids, then cluster vertices by Euclidean distance
 */
void visualizeSkeletalKMeansClustering() {
    if (!g_segmentation_result.success || g_segmentation_result.partialSkeleton.empty() || !g_mesh) {
        cout << "Error: Please run full segmentation first to generate skeletal segments!" << endl;
        cout << "Use option 7 to perform complete segmentation." << endl;
        return;
    }

    cout << "\n=== SKELETAL K-MEANS CLUSTERING ===" << endl;
    cout << "Using " << g_segmentation_result.partialSkeleton.size() << " skeletal segments as cluster centroids" << endl;

    clearVisualization();

    // Define colors for different clusters
    vector<Eigen::Vector3f> clusterColors = {
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

    // Generate centroids along each skeletal segment
    vector<vector<Eigen::Vector3d>> skeletalCentroids(g_segmentation_result.partialSkeleton.size());

    for (int segIdx = 0; segIdx < g_segmentation_result.partialSkeleton.size(); segIdx++) {
        const auto& segment = g_segmentation_result.partialSkeleton[segIdx];

        if (segment.nodes.size() < 2) {
            // If segment has only one node, use it as the single centroid
            if (!segment.nodes.empty()) {
                skeletalCentroids[segIdx].push_back(segment.nodes[0]);
            }
            continue;
        }

        // Generate points along the skeletal curve/line
        int numCentroidsPerSegment = max(3, (int)segment.nodes.size() / 2); // At least 3 centroids per segment

        for (int i = 0; i < numCentroidsPerSegment; i++) {
            double t = (double)i / (numCentroidsPerSegment - 1); // Parameter from 0 to 1

            // Find the position along the skeletal path at parameter t
            double totalLength = 0.0;
            for (int j = 0; j < segment.nodes.size() - 1; j++) {
                totalLength += (segment.nodes[j+1] - segment.nodes[j]).norm();
            }

            double targetLength = t * totalLength;
            double currentLength = 0.0;

            for (int j = 0; j < segment.nodes.size() - 1; j++) {
                double edgeLength = (segment.nodes[j+1] - segment.nodes[j]).norm();

                if (currentLength + edgeLength >= targetLength) {
                    // Interpolate between nodes j and j+1
                    double alpha = (targetLength - currentLength) / edgeLength;
                    Eigen::Vector3d interpolatedPoint = segment.nodes[j] + alpha * (segment.nodes[j+1] - segment.nodes[j]);
                    skeletalCentroids[segIdx].push_back(interpolatedPoint);
                    break;
                }
                currentLength += edgeLength;
            }
        }

        cout << "  Segment " << segIdx << ": Generated " << skeletalCentroids[segIdx].size() << " centroids" << endl;
    }

    // Cluster each vertex to the closest skeletal centroid
    vector<int> vertexClusters(g_mesh->verts.size());
    vector<int> clusterSizes(g_segmentation_result.partialSkeleton.size(), 0);

    for (int v = 0; v < g_mesh->verts.size(); v++) {
        Eigen::Vector3d vertexPos(g_mesh->verts[v]->coords[0],
                                 g_mesh->verts[v]->coords[1],
                                 g_mesh->verts[v]->coords[2]);

        double minDistance = 1e10;
        int closestCluster = 0;

        // Find the closest skeletal centroid across all segments
        for (int segIdx = 0; segIdx < skeletalCentroids.size(); segIdx++) {
            for (const auto& centroid : skeletalCentroids[segIdx]) {
                double distance = (vertexPos - centroid).norm();
                if (distance < minDistance) {
                    minDistance = distance;
                    closestCluster = segIdx;
                }
            }
        }

        vertexClusters[v] = closestCluster;
        clusterSizes[closestCluster]++;
    }

    // Print cluster statistics
    cout << "Cluster sizes:" << endl;
    for (int i = 0; i < clusterSizes.size(); i++) {
        cout << "  Cluster " << i << ": " << clusterSizes[i] << " vertices" << endl;
    }

    // Visualize vertices colored by cluster
    cout << "Rendering colored spheres for each cluster..." << endl;
    for (int v = 0; v < g_mesh->verts.size(); v += 12) { // Downsample for performance
        Eigen::Vector3d vertexPos(g_mesh->verts[v]->coords[0],
                                 g_mesh->verts[v]->coords[1],
                                 g_mesh->verts[v]->coords[2]);

        int cluster = vertexClusters[v];
        Eigen::Vector3f color = clusterColors[cluster % clusterColors.size()];

        SoSeparator* pointViz = VisualizationUtils::createColoredSphere(vertexPos, color, 0.006f);
        g_root->addChild(pointViz);
    }

    // Visualize the skeletal centroids as larger spheres
    cout << "Rendering skeletal centroids..." << endl;
    for (int segIdx = 0; segIdx < skeletalCentroids.size(); segIdx++) {
        Eigen::Vector3f color = clusterColors[segIdx % clusterColors.size()];

        for (const auto& centroid : skeletalCentroids[segIdx]) {
            SoSeparator* centroidViz = VisualizationUtils::createColoredSphere(centroid, color, 0.015f);
            g_root->addChild(centroidViz);
        }
    }

    // Also visualize the original skeletal segments as lines
    cout << "Rendering skeletal segments..." << endl;
    for (int segIdx = 0; segIdx < g_segmentation_result.partialSkeleton.size(); segIdx++) {
        const auto& segment = g_segmentation_result.partialSkeleton[segIdx];
        if (segment.nodes.size() < 2) continue;

        Eigen::Vector3f color = clusterColors[segIdx % clusterColors.size()];

        SoSeparator* lineSep = new SoSeparator();
        SoMaterial* mat = new SoMaterial();
        mat->diffuseColor.setValue(color[0], color[1], color[2]);
        mat->emissiveColor.setValue(color[0] * 0.3f, color[1] * 0.3f, color[2] * 0.3f);
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

    g_viewer->scheduleRedraw();
    g_viewer->viewAll();

    cout << "\n=== SKELETAL K-MEANS CLUSTERING COMPLETE ===" << endl;
    cout << "-> Used " << g_segmentation_result.partialSkeleton.size() << " skeletal segments as cluster bases" << endl;
    cout << "-> Clustered vertices using Euclidean distance to skeletal centroids" << endl;
    cout << "-> Small spheres: vertices colored by cluster assignment" << endl;
    cout << "-> Large spheres: skeletal centroids used for clustering" << endl;
    cout << "-> Colored lines: original skeletal segments" << endl;
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