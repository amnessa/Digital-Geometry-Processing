/**
 * Pairwise Harmonics Segmentation - Clean Implementation
 *
 * Implementation of "Pairwise Harmonics for Shape Analysis"
 * by Zheng et al., IEEE TVCG 2013
 *
 * Focus: Simultaneous Skeletonization and Segmentation (Section 3.3 of the paper)
 */

#include "pch.h"
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

// Eigen headers
#include <Eigen/Dense>
#include <Eigen/Sparse>

// Windows headers
#define NOMINMAX
#include <windows.h>

// Coin3D headers
#include <Inventor/Win/SoWin.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>

// Project headers
#include "Mesh.h"
#include "Painter.h"
#include "helpers.h"
#include "PairwiseHarmonicsSegmentation.h"

using namespace std;

// Global variables
Mesh* g_mesh = nullptr;
Painter* g_painter = nullptr;
PairwiseHarmonics* g_harmonics = nullptr;
PairwiseHarmonicsSegmentation* g_segmentation = nullptr;
SoSeparator* g_root = nullptr;
SoWinExaminerViewer* g_viewer = nullptr;

// Segmentation results
PairwiseHarmonicsSegmentation::SegmentationResult g_result;

// Forward declarations
DWORD WINAPI ConsoleInputThread(LPVOID lpParam);
void showMenu();
void loadMesh();
void runSegmentation();
void visualizeResults();
void clearVisualization();
void testComponents();

/**
 * Main entry point
 */
int main(int argc, char** argv) {
    cout << "=== PAIRWISE HARMONICS SEGMENTATION ===" << endl;
    cout << "Clean implementation of 'Pairwise Harmonics for Shape Analysis'" << endl;
    cout << "by Zheng et al., IEEE TVCG 2013" << endl;
    cout << endl;

    // Initialize Coin3D
    HWND window = SoWin::init(argv[0]);
    if (window == NULL) {
        cout << "Error: Failed to initialize SoWin!" << endl;
        return 1;
    }

    // Initialize scene graph
    g_root = new SoSeparator;
    g_root->ref();

    // Create painter for visualization
    g_painter = new Painter();

    // Set up 3D viewer
    g_viewer = new SoWinExaminerViewer(window);
    g_viewer->setSceneGraph(g_root);
    g_viewer->setTitle("Pairwise Harmonics Segmentation");
    g_viewer->setSize(SbVec2s(1200, 800));
    g_viewer->show();

    // Show initial menu
    showMenu();

    // Create console input thread
    HANDLE consoleThread = CreateThread(NULL, 0, ConsoleInputThread, NULL, 0, NULL);
    if (consoleThread == NULL) {
        cout << "Error: Failed to create console input thread!" << endl;
        return 1;
    }

    // Start main event loop
    SoWin::show(window);
    SoWin::mainLoop();

    // Cleanup
    CloseHandle(consoleThread);
    if (g_segmentation) delete g_segmentation;
    if (g_harmonics) delete g_harmonics;
    if (g_mesh) delete g_mesh;
    delete g_painter;
    g_root->unref();

    return 0;
}

/**
 * Display the main menu
 */
void showMenu() {
    cout << "\n=== MAIN MENU ===" << endl;
    cout << "1. Load mesh" << endl;
    cout << "2. Run complete segmentation algorithm" << endl;
    cout << "3. Visualize segmentation results" << endl;
    cout << "4. Test individual components" << endl;
    cout << "5. Clear visualization" << endl;
    cout << "6. Exit" << endl;
    cout << "Enter choice (1-6): ";
}

/**
 * Load a mesh file
 */
void loadMesh() {
    cout << "\n=== LOAD MESH ===" << endl;
    cout << "Available meshes:" << endl;
    cout << "  - man0.off" << endl;
    cout << "  - Armadillo.off" << endl;
    cout << "  - 50.off" << endl;
    cout << "  - 249.off" << endl;
    cout << "  - 348.off" << endl;
    cout << "Enter mesh filename: ";

    string meshName;
    cin >> meshName;

    // Clear previous state
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
    }

    // Load mesh
    g_mesh = new Mesh();
    string meshPath = meshName; // Assume mesh files are in current directory

    cout << "Loading mesh from " << meshPath << "..." << endl;
    g_mesh->loadOff(const_cast<char*>(meshPath.c_str()));
    g_mesh->normalizeCoordinates();

    cout << "Mesh loaded successfully:" << endl;
    cout << "  Vertices: " << g_mesh->verts.size() << endl;
    cout << "  Triangles: " << g_mesh->tris.size() << endl;

    // Initialize pairwise harmonics
    g_harmonics = new PairwiseHarmonics(g_mesh);
    if (!g_harmonics->initializeLibIGL()) {
        cout << "Warning: LibIGL initialization failed, using custom implementation" << endl;
    }

    // Initialize segmentation algorithm
    g_segmentation = new PairwiseHarmonicsSegmentation(g_mesh, g_harmonics);

    // Visualize mesh
    g_root->addChild(g_painter->getShapeSep(g_mesh));
    g_viewer->viewAll();

    cout << "Mesh loaded and ready for segmentation!" << endl;
}

/**
 * Run the complete segmentation algorithm
 */
void runSegmentation() {
    if (!g_mesh || !g_segmentation) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }

    cout << "\n=== RUNNING SEGMENTATION ALGORITHM ===" << endl;

    // Get parameters from user
    auto params = g_segmentation->getParameters();

    cout << "Current parameters:" << endl;
    cout << "  FPS samples: " << params.numFPSSamples << endl;
    cout << "  Isocurves per field (K): " << params.numIsocurves << endl;
    cout << "  Rigidity threshold: " << params.rigidityThreshold << endl;
    cout << endl;

    cout << "Use default parameters? (y/n): ";
    char choice;
    cin >> choice;

    if (choice == 'n' || choice == 'N') {
        cout << "Enter number of FPS samples: ";
        cin >> params.numFPSSamples;
        cout << "Enter number of isocurves per field (K): ";
        cin >> params.numIsocurves;
        cout << "Enter rigidity threshold (0.85-0.95): ";
        cin >> params.rigidityThreshold;

        g_segmentation->setParameters(params);
    }

    // Run segmentation
    cout << "Starting segmentation algorithm..." << endl;
    auto start = clock();

    g_result = g_segmentation->performSegmentation();

    auto end = clock();
    double elapsed = double(end - start) / CLOCKS_PER_SEC;

    if (g_result.success) {
        cout << "\nSegmentation completed successfully in " << elapsed << " seconds!" << endl;
        cout << "Results:" << endl;
        cout << "  FPS samples: " << g_result.fpsPoints.size() << endl;
        cout << "  Skeletal segments: " << g_result.partialSkeleton.size() << endl;
        cout << "  Mesh components: " << g_result.meshComponents.size() << endl;
        cout << "  Skeleton nodes: " << g_result.skeletonNodes.size() << endl;
        cout << "\nUse option 3 to visualize results." << endl;
    } else {
        cout << "Error: Segmentation failed - " << g_result.errorMessage << endl;
    }
}

/**
 * Visualize segmentation results
 */
void visualizeResults() {
    if (!g_result.success) {
        cout << "Error: No valid segmentation results to visualize!" << endl;
        return;
    }

    cout << "\n=== VISUALIZING RESULTS ===" << endl;

    clearVisualization();

    // Add base mesh
    g_root->addChild(g_painter->getShapeSep(g_mesh));    // Visualize using the segmentation class
    PairwiseHarmonicsSegmentation::visualizeResults(
        g_result, g_root, g_painter, g_viewer, g_mesh
    );

    g_viewer->viewAll();

    cout << "Visualization complete!" << endl;
    cout << "  Red lines: Skeletal segments" << endl;
    cout << "  Colored spheres: FPS sample points" << endl;
    cout << "  Different colors: Mesh components" << endl;
}

/**
 * Test individual components of the algorithm
 */
void testComponents() {
    if (!g_mesh || !g_harmonics) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }

    cout << "\n=== COMPONENT TESTING ===" << endl;
    cout << "1. Test FPS sampling" << endl;
    cout << "2. Test pairwise harmonics" << endl;
    cout << "3. Test isocurve extraction" << endl;
    cout << "4. Test rigidity analysis" << endl;
    cout << "Enter choice (1-4): ";

    int choice;
    cin >> choice;

    switch (choice) {
        case 1: {
            cout << "Testing FPS sampling..." << endl;
            vector<int> fps = g_mesh->farthestPointSampling(15);
            cout << "Generated " << fps.size() << " FPS points" << endl;

            // Visualize FPS points
            clearVisualization();
            g_root->addChild(g_painter->getShapeSep(g_mesh));

            for (int i = 0; i < fps.size(); i++) {
                float r = 1.0f, g = 0.0f, b = 0.0f;
                g_root->addChild(g_painter->get1PointSep(g_mesh, fps[i], r, g, b, 8.0f, false));
            }
            g_viewer->viewAll();
            break;
        }

        case 2: {
            cout << "Testing pairwise harmonics..." << endl;
            vector<int> fps = g_mesh->farthestPointSampling(5);
            if (fps.size() >= 2) {
                Eigen::VectorXd field = g_harmonics->computePairwiseHarmonicLibIGL(fps[0], fps[1]);
                cout << "Computed harmonic field between vertices " << fps[0] << " and " << fps[1] << endl;
                cout << "Field range: [" << field.minCoeff() << ", " << field.maxCoeff() << "]" << endl;

                // Visualize field
                clearVisualization();
                g_root->addChild(g_harmonics->visualizeHarmonicField(field, g_painter));
                g_viewer->viewAll();
            }
            break;
        }

        case 3: {
            cout << "Testing isocurve extraction..." << endl;
            vector<int> fps = g_mesh->farthestPointSampling(5);
            if (fps.size() >= 2) {
                Eigen::VectorXd field = g_harmonics->computePairwiseHarmonicLibIGL(fps[0], fps[1]);
                auto segments = g_harmonics->extractIsoCurveSegments(field, 0.5);
                cout << "Extracted " << segments.size() << " isocurve segments at level 0.5" << endl;
            }
            break;
        }

        case 4: {
            cout << "Testing rigidity analysis..." << endl;
            cout << "This will be implemented as part of the full segmentation." << endl;
            break;
        }

        default:
            cout << "Invalid choice!" << endl;
            break;
    }
}

/**
 * Clear visualization
 */
void clearVisualization() {
    while (g_root->getNumChildren() > 0) {
        g_root->removeChild(0);
    }
}

/**
 * Console input thread
 */
DWORD WINAPI ConsoleInputThread(LPVOID lpParam) {
    int choice = 0;

    while (choice != 6) {
        cin >> choice;

        switch (choice) {
            case 1:
                loadMesh();
                break;
            case 2:
                runSegmentation();
                break;
            case 3:
                visualizeResults();
                break;
            case 4:
                testComponents();
                break;
            case 5:
                clearVisualization();
                cout << "Visualization cleared." << endl;
                break;
            case 6:
                cout << "Exiting..." << endl;
                PostQuitMessage(0);
                break;
            default:
                cout << "Invalid choice! Please enter 1-6." << endl;
                break;
        }

        if (choice != 6) {
            showMenu();
        }
    }

    return 0;
}
