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
#include "helpers.h"
#include "PairwiseHarmonicsSegmentation.h"
#include "IsocurveAnalysis.h"
#include "RigidityAnalysis.h"
#include "VisualizationUtils.h"
#include "AnimatedMeshPipeline.h"

using namespace std;

// Global variables for core components
Mesh* g_mesh = nullptr;
Painter* g_painter = nullptr;
PairwiseHarmonics* g_harmonics = nullptr;
PairwiseHarmonicsSegmentation* g_segmentation = nullptr;
AnimatedMeshPipeline* g_animated_pipeline = nullptr; // NEW: For animation

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

// NEW: Global variables for animated mesh pipeline
vector<Mesh*> g_animation_meshes;                 // List of animation frame meshes
int g_currentFrameIndex = 0;                      // Current frame being processed

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

// NEW: Animated mesh pipeline functions
void generateStaticAnalysis();           // Stage 1: Run once on reference mesh
void processAnimationSequence();        // Stage 2: Process all animation frames
void processAnimationFrame(int frameIndex);           // Process single frame interactively
void loadAnimationSequence();           // Load horse gallop animation

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
    cout << "  9. horse-gallop-reference.off (for animation)" << endl;
    cout << "\nEnter mesh number (1-9) or filename directly: ";

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
    else if (input == "8") meshPath = "hand.off";
    else if (input == "9") meshPath = "horse_gallop_off/horse-gallop-reference.off";
    else meshPath = input; // Assume it's a direct filename

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
 * Unload the current mesh and clear all associated data
 */
void unloadMesh() {
    cout << "\nUnloading mesh and clearing data..." << endl;

    // Clear visualization completely by removing all nodes from the scene graph
    if (g_root) {
        g_root->removeAllChildren();
    }
    if (g_viewer) {
        g_viewer->scheduleRedraw();
    }

    // Delete animation data
    if (g_animated_pipeline) {
        delete g_animated_pipeline;
        g_animated_pipeline = nullptr;
    }
    for (auto* mesh : g_animation_meshes) {
        delete mesh;
    }
    g_animation_meshes.clear();

    // Delete segmentation components
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

    // Clear state variables
    g_fps_samples.clear();
    g_skeletal_segments.clear();
    g_rigidity_scores.clear();
    g_nonrigid_nodes.clear();
    g_segmentation_result = {}; // Reset segmentation result

    // The visualization is already cleared above.
    cout << "Mesh unloaded." << endl;
}


/**
 * Clear all visualization except the base mesh
 */
void clearVisualization() {
    // Use the utility function which preserves the mesh (the first child)
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
 * Placeholder for testing heat geodesics
 */
void testHeatGeodesics() {
    VisualizationUtils::testHeatGeodesics(g_mesh, g_harmonics, g_root, g_viewer);
}

/**
 * Placeholder for testing enhanced FPS
 */
void testEnhancedFPS() {
    VisualizationUtils::testEnhancedFPS(g_mesh, g_root, g_viewer);
}

/**
 * Placeholder for comparing geodesic methods
 */
void compareGeodesicMethods() {
    VisualizationUtils::compareGeodesicMethods(g_mesh, g_harmonics);
}

/**
 * Placeholder for testing enhanced descriptors
 */
void testEnhancedDescriptors() {
    VisualizationUtils::testEnhancedDescriptors(g_mesh, g_harmonics, DEFAULT_K);
}

/**
 * Placeholder for testing mesh analysis
 */
void testMeshAnalysis() {
    VisualizationUtils::testMeshAnalysis(g_mesh, g_harmonics);
}

/**
 * Placeholder for visualizing mesh segmentation clusters
 */
void visualizeMeshSegmentationClusters() {
    VisualizationUtils::visualizeMeshSegmentationClusters(g_segmentation_result, g_mesh, g_root, g_viewer);
}

/**
 * Placeholder for visualizing skeletal k-means clustering
 */
void visualizeSkeletalKMeansClustering() {
    cout << "Function 'visualizeSkeletalKMeansClustering' is not yet implemented." << endl;
}

/**
 * Placeholder for visualizing graph-cut segmentation
 */
void visualizeGraphCutSegmentation() {
    VisualizationUtils::visualizeGraphCutSegmentation(g_segmentation_result, g_mesh, g_harmonics, g_root, g_viewer);
}


void analyzeSkeletalSegments() {
    if (!g_segmentation_result.success) {
        cout << "Error: A successful segmentation must be performed first (Option 8)." << endl;
        return;
    }

    cout << "\n--- ANALYZING SKELETAL SEGMENTS ---" << endl;

    const std::vector<Eigen::Vector3d>* nodes_ptr = &g_segmentation_result.skeletonNodes;
    bool isDeformed = false;
    Mesh* meshToDraw = g_mesh;

    // Check if we are in an animation context and have a deformed skeleton
    if (g_animated_pipeline && !g_animated_pipeline->getDeformedSkeleton().empty()) {
        nodes_ptr = &g_animated_pipeline->getDeformedSkeleton();
        isDeformed = true;
        if (!g_animation_meshes.empty()) {
            meshToDraw = g_animation_meshes[g_currentFrameIndex];
        }
        cout << "Visualizing DEFORMED skeleton for frame " << g_currentFrameIndex << "..." << endl;
    } else {
        cout << "Visualizing REFERENCE skeleton..." << endl;
    }

    const auto& nodes = *nodes_ptr;
    const auto& adjacency = g_segmentation_result.skeletonAdjacency;
    cout << "  Nodes: " << nodes.size() << endl;
    cout << "  Edges: " << adjacency.size() << endl;

    // Clear previous visualizations but keep the mesh
    clearVisualization();
    if (meshToDraw) {
        g_root->addChild(g_painter->getShapeSep(meshToDraw));
    }

    // Reconstruct skeleton segments from the nodes and adjacency list
    std::vector<std::vector<Eigen::Vector3d>> segments;
    for (const auto& edge : adjacency) {
        if (edge.first < nodes.size() && edge.second < nodes.size()) {
            segments.push_back({ nodes[edge.first], nodes[edge.second] });
        }
    }

    // Visualize the skeleton in a distinct color
    Eigen::Vector3f skeletonColor = isDeformed ? Eigen::Vector3f(0.0, 1.0, 0.0) : Eigen::Vector3f(1.0, 1.0, 0.0); // Green for deformed, Yellow for reference
    VisualizationUtils::visualizeSkeletalSegments(segments, skeletonColor, g_root);

    g_viewer->viewAll();
    cout << "Skeleton visualized." << endl;
}

/**
 * Placeholder for analyzing detailed rigidity
 */
void analyzeDetailedRigidity() {
    VisualizationUtils::analyzeDetailedRigidity(g_segmentation_result);
}

/**
 * Placeholder for testing harmonic surface segmentation
 */
void testHarmonicSurfaceSegmentation() {
    VisualizationUtils::visualizeHarmonicSurfaceSegmentation(g_segmentation_result, g_mesh, g_harmonics, g_root, g_viewer);
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
    cout << "2. Unload Current Mesh" << endl;
    cout << "3. Test Cotangent Laplacian" << endl;
    cout << "4. Test Farthest Point Sampling" << endl;
    cout << "5. Test Pairwise Harmonics" << endl;
    cout << "6. Test Iso-curve Extraction" << endl;
    cout << "7. Test Rigidity Analysis" << endl;
    cout << "8. Perform Full Segmentation" << endl;
    cout << "9. Clear Visualization" << endl;
    cout << "--- ENHANCED ANALYSIS ---" << endl;
    cout << "10. Improved Iso-curve Extraction (Dense Bracelets)" << endl;
    cout << "11. Enhanced Rigidity Analysis (All Nodes)" << endl;
    cout << "12. Visualize All Rigidity Points (Color-coded)" << endl;
    cout << "13. Interactive Rigidity Threshold Testing" << endl;
    cout << "--- LIBIGL ENHANCED FEATURES ---" << endl;
    cout << "14. Test Heat Geodesics (LibIGL)" << endl;
    cout << "15. Enhanced FPS with Heat Geodesics" << endl;
    cout << "16. Compare Geodesic Methods (Dijkstra vs Heat)" << endl;
    cout << "17. Enhanced R&D Descriptors (LibIGL)" << endl;
    cout << "18. Mesh Analysis (Normals, Curvature, Areas)" << endl;
    cout << "--- SEGMENTATION VISUALIZATION ---" << endl;
    cout << "19. Visualize Mesh Components (Colored Vertex Clusters)" << endl;
    cout << "20. Harmonic Function Surface Segmentation" << endl;
    cout << "21. Analyze Skeletal Segments (Diagnostic)" << endl;
    cout << "22. Detailed Rigidity Analysis (Debug Cutting)" << endl;
    cout << "23. Optimal Segmentation (Graph-Cut Energy Minimization)" << endl;
    cout << "--- ANIMATED MESH PIPELINE ---" << endl;
    cout << "24. Load Horse Gallop Animation" << endl;
    cout << "25. Generate Static Analysis (BIND SKELETON)" << endl;
    cout << "26. Process Animation Sequence (DEFORM SKELETON)" << endl;
    cout << "27. Process Single Animation Frame (Interactive)" << endl;
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
                VisualizationUtils::testHeatGeodesics(g_mesh, g_harmonics, g_root, g_viewer);
                break;
            case 15:
                VisualizationUtils::testEnhancedFPS(g_mesh, g_root, g_viewer);
                break;
            case 16:
                VisualizationUtils::compareGeodesicMethods(g_mesh, g_harmonics);
                break;
            case 17:
                VisualizationUtils::testEnhancedDescriptors(g_mesh, g_harmonics, DEFAULT_K);
                break;
            case 18:
                VisualizationUtils::testMeshAnalysis(g_mesh, g_harmonics);
                break;
            case 19:
                VisualizationUtils::visualizeMeshSegmentationClusters(g_segmentation_result, g_mesh, g_root, g_viewer);
                break;
            case 20:
                VisualizationUtils::visualizeHarmonicSurfaceSegmentation(g_segmentation_result, g_mesh, g_harmonics, g_root, g_viewer);
                break;
            case 21:
                analyzeSkeletalSegments();
                break;
            case 22:
                VisualizationUtils::analyzeDetailedRigidity(g_segmentation_result);
                break;
            case 23:
                // Check for animation context first
                if (g_animated_pipeline && !g_animation_meshes.empty() && !g_animated_pipeline->getDeformedSkeleton().empty()) {
                    cout << "Re-segmenting current animation frame using deformed skeleton..." << endl;
                    Mesh* currentFrameMesh = g_animation_meshes[g_currentFrameIndex];

                    // Create a temporary, updated segmentation result for the deformed frame.
                    // This forces the visualization function to use the new skeleton nodes.
                    PairwiseHarmonicsSegmentation::SegmentationResult deformed_result = g_segmentation_result; // Copy original connectivity
                    deformed_result.skeletonNodes = g_animated_pipeline->getDeformedSkeleton(); // Overwrite with deformed nodes

                    // A new harmonics object is needed for the new mesh geometry
                    PairwiseHarmonics harmonics_for_frame(currentFrameMesh);
                    harmonics_for_frame.initializeLibIGL(); // Re-initialize for the new mesh

                    // Call the visualization with the temporary result containing the deformed skeleton
                    VisualizationUtils::visualizeGraphCutSegmentation(deformed_result, currentFrameMesh, &harmonics_for_frame, g_root, g_viewer, nullptr);
                } else if (g_mesh && g_harmonics && g_segmentation_result.success) {
                    // Fallback to static mesh segmentation
                    VisualizationUtils::visualizeGraphCutSegmentation(g_segmentation_result, g_mesh, g_harmonics, g_root, g_viewer, nullptr);
                } else {
                    cout << "Please load a mesh and run segmentation (Option 8) first." << endl;
                }
                break;
            case 24:
                loadAnimationSequence();
                break;
            case 25:
                generateStaticAnalysis();
                break;
            case 26:
                processAnimationSequence();
                break;
            case 0:
                cout << "Exiting..." << endl;
                exit(0);
                break;
            case 27:
                if (g_animation_meshes.empty()) {
                    cout << "Please load an animation sequence first (Option 24)." << endl;
                } else {
                    cout << "Enter frame index to process (0-" << g_animation_meshes.size() - 1 << "): ";
                    int frame_idx;
                    cin >> frame_idx;
                    // Clear input buffer after reading number
                    cin.ignore(numeric_limits<streamsize>::max(), '\n');
                    processAnimationFrame(frame_idx);
                }
                break;
            default:
                cout << "Invalid option!" << endl;
                break;
        }
    }

    return 0;
}

/**
 * Generate the static analysis for the animated mesh pipeline
 * Stage 1: Bind the skeleton to the mesh
 */
void generateStaticAnalysis() {
    if (!g_mesh) {
        cout << "Error: No mesh loaded!" << endl;
        return;
    }
    if (!g_segmentation_result.success) {
        cout << "Error: Please perform a successful segmentation first (Option 8)." << endl;
        return;
    }

    // Clear previous static analysis data
    if (g_animated_pipeline) {
        delete g_animated_pipeline;
        g_animated_pipeline = nullptr;
    }

    // Create and bind the animated mesh pipeline using the segmentation result
    cout << "\n=== GENERATING STATIC ANALYSIS (BINDING SKELETON) ===" << endl;
    g_animated_pipeline = new AnimatedMeshPipeline(g_mesh, g_segmentation_result);
    g_animated_pipeline->runStaticAnalysis(); // This calls bindSkeleton() internally

    cout << "Static analysis generated successfully!" << endl;
    cout << "Skeleton is now bound to the reference mesh." << endl;
}

/**
 * Process the entire animation sequence
 * Stage 2: Deform the skeleton across all frames
 */
void processAnimationSequence() {
    if (!g_animated_pipeline) {
        cout << "Please generate the static analysis first." << endl;
        return;
    }
    if (g_animation_meshes.empty()) {
        cout << "Please load an animation sequence first." << endl;
        return;
    }

    cout << "\n=== PROCESSING ANIMATION SEQUENCE ===" << endl;
    for (size_t i = 0; i < g_animation_meshes.size(); ++i) {
        Mesh* frameMesh = g_animation_meshes[i];
        g_animated_pipeline->processAnimationFrame(frameMesh);
    }
    cout << "Animation sequence processed: " << g_animation_meshes.size() << " frames." << endl;
    cout << "Use interactive frame processing to visualize results." << endl;
}

/**
 * Process and visualize a single frame of the animation
 */
void processAnimationFrame(int frameIndex) {
    if (!g_animated_pipeline) {
        cout << "Please generate the static analysis first." << endl;
        return;
    }
    if (g_animation_meshes.empty()) {
        cout << "Please load an animation sequence first." << endl;
        return;
    }
    if (frameIndex < 0 || frameIndex >= g_animation_meshes.size()) {
        cout << "Invalid frame index. Please choose between 0 and " << g_animation_meshes.size() - 1 << "." << endl;
        return;
    }


    g_currentFrameIndex = frameIndex;
    cout << "\n--- PROCESSING FRAME " << g_currentFrameIndex << " ---" << endl;

    // Process the frame to get the deformed skeleton
    Mesh* currentFrameMesh = g_animation_meshes[g_currentFrameIndex];
    g_animated_pipeline->processAnimationFrame(currentFrameMesh);
    const auto& deformedNodes = g_animated_pipeline->getDeformedSkeleton();

    // Visualize the result
    // Visualize the result
    if (g_root) {
        g_root->removeAllChildren();
    }
    g_root->addChild(g_painter->getShapeSep(currentFrameMesh)); // Show the current frame's mesh

    // Reconstruct skeleton segments from deformed nodes and original connectivity
    const auto& adjacency = g_segmentation_result.skeletonAdjacency;
    std::vector<std::vector<Eigen::Vector3d>> deformedSegments;
    for (const auto& edge : adjacency) {
        if (edge.first < deformedNodes.size() && edge.second < deformedNodes.size()) {
            deformedSegments.push_back({ deformedNodes[edge.first], deformedNodes[edge.second] });
        }
    }
    VisualizationUtils::visualizeSkeletalSegments(deformedSegments, Eigen::Vector3f(0.0, 1.0, 0.0), g_root); // Show deformed skeleton in green

    g_viewer->viewAll();

    cout << "Visualizing deformed skeleton for frame " << g_currentFrameIndex << "." << endl;
}

/**
 * Load the horse gallop animation sequence into memory
 */
void loadAnimationSequence() {
    if (!g_mesh) {
        cout << "Please load the 'horse-gallop-reference.off' mesh first (Option 1, then 9)." << endl;
        return;
    }

    cout << "\nLoading animation sequence from 'horse_gallop_off'..." << endl;
    // Clear any previously loaded animation
    for (auto* m : g_animation_meshes) {
        delete m;
    }
    g_animation_meshes.clear();

    // The horse gallop sequence has 48 frames (01 to 48)
    for (int i = 1; i <= 48; ++i) {
        char filename[256];
        sprintf_s(filename, "models/horse_gallop_off/horse-gallop-%02d.off", i);
        Mesh* frameMesh = new Mesh();
        frameMesh->loadOff(filename); // loadOff is void, cannot be used in an if statement

        // Check if loading was successful by seeing if the mesh has vertices
        if (!frameMesh->verts.empty()) {
            frameMesh->normalizeCoordinates();
            g_animation_meshes.push_back(frameMesh);
        } else {
            cout << "Warning: Could not load or file is empty: " << filename << endl;
            delete frameMesh;
            break; // Stop if a file in the sequence is missing
        }
    }
    cout << "Loaded " << g_animation_meshes.size() << " animation frames." << endl;
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
