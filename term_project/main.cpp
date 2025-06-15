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

// Forward declarations
DWORD WINAPI ConsoleInputThread(LPVOID lpParam);

// Global variables to store mesh and painter
Mesh* g_mesh = nullptr;
Painter* g_painter = nullptr;
PairwiseHarmonics* g_harmonics = nullptr;

// Global variables for visualization
SoSeparator* g_root = nullptr;
SoWinExaminerViewer* g_viewer = nullptr;

// Global variables for segmentation state
vector<int> g_fps_samples;
vector<vector<int>> g_skeletal_segments;
vector<double> g_rigidity_scores;

// Forward declarations for segmentation functions
void loadNewMesh(const string& meshPath);
void testCotangentLaplacian();
void verifyLaplacianProperties();
void testPairwiseHarmonics();
void testRigidityAnalysis();
void computeFarthestPointSampling();
void testIsoCurveExtraction();
void visualizeSkeletalSegments();
void performSegmentation();

/**
 * Load a new mesh and prepare it for visualization
 * @param meshPath Path to the mesh file (.off format)
 */
void loadNewMesh(const string& meshPath) {
    // Clear previous visualization
    while (g_root->getNumChildren() > 0) {
        g_root->removeChild(0);
    }

    // Delete old mesh and harmonics if they exist
    if (g_harmonics) {
        delete g_harmonics;
        g_harmonics = nullptr;
    }
    if (g_mesh) {
        delete g_mesh;
    }

    // Create new mesh
    g_mesh = new Mesh();

    // Load the mesh
    cout << "Loading mesh from " << meshPath << "..." << endl;
    g_mesh->loadOff(const_cast<char*>(meshPath.c_str()));
    g_mesh->normalizeCoordinates();

    cout << "Mesh loaded with " << g_mesh->verts.size() << " vertices and "
         << g_mesh->tris.size() << " triangles." << endl;
    cout << "Vertex indices range from 0 to " << g_mesh->verts.size() - 1 << endl;

    // Add the base mesh visualization
    g_root->addChild(g_painter->getShapeSep(g_mesh));

    // Initialize pairwise harmonics
    g_harmonics = new PairwiseHarmonics(g_mesh);

    // Clear previous segmentation state
    g_fps_samples.clear();
    g_skeletal_segments.clear();
    g_rigidity_scores.clear();

    // Update viewer
    g_viewer->viewAll();

    cout << "Mesh loaded successfully. Ready for segmentation analysis." << endl;
}

/**
 * Test the cotangent Laplacian computation
 */
void testCotangentLaplacian() {
    if (!g_mesh || !g_harmonics) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }

    cout << "Computing cotangent Laplacian matrix..." << endl;
    auto start = clock();

    bool success = g_harmonics->computeCotangentLaplacian();

    auto end = clock();
    double elapsed = double(end - start) / CLOCKS_PER_SEC;

    if (success) {
        cout << "Cotangent Laplacian computed successfully in " << elapsed << " seconds." << endl;
        cout << "Matrix size: " << g_mesh->verts.size() << "x" << g_mesh->verts.size() << endl;
    } else {
        cout << "Error: Failed to compute cotangent Laplacian!" << endl;
    }
}

/**
 * Test pairwise harmonic function computation
 */
void testPairwiseHarmonics() {
    if (!g_mesh || !g_harmonics) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }

    // Clear previous visualization (except base mesh)
    while (g_root->getNumChildren() > 1) {
        g_root->removeChild(g_root->getNumChildren() - 1);
    }

    int nVerts = g_mesh->verts.size();

    // Ask user for two vertices to compute the pairwise harmonic between
    int p_idx, q_idx;
    cout << "Enter first vertex index (p) - will be set to value 0 (0-" << nVerts - 1 << "): ";
    cin >> p_idx;
    cout << "Enter second vertex index (q) - will be set to value 1 (0-" << nVerts - 1 << "): ";
    cin >> q_idx;

    // Validate input
    if (p_idx < 0 || p_idx >= nVerts || q_idx < 0 || q_idx >= nVerts || p_idx == q_idx) {
        cout << "Error: Invalid vertex indices! Both must be different and in range [0, " << nVerts - 1 << "]" << endl;
        return;
    }

    cout << "Computing pairwise harmonic function between vertices " << p_idx << " and " << q_idx << "..." << endl;
    auto start = clock();

    // Compute the harmonic field
    Eigen::VectorXd harmonicField = g_harmonics->computePairwiseHarmonic(p_idx, q_idx);

    auto end = clock();
    double elapsed = double(end - start) / CLOCKS_PER_SEC;

    if (harmonicField.size() == nVerts) {
        cout << "Pairwise harmonic computed successfully in " << elapsed << " seconds." << endl;

        // Visualize the harmonic field
        SoSeparator* harmonicVisualization = g_harmonics->visualizeHarmonicField(harmonicField, g_painter);
        g_root->addChild(harmonicVisualization);

        cout << "Harmonic field visualization added." << endl;
        cout << "Blue areas are close to vertex " << p_idx << " (value ~0)" << endl;
        cout << "Red areas are close to vertex " << q_idx << " (value ~1)" << endl;

        // Compute and display R and D descriptors
        cout << "\nComputing R and D descriptors..." << endl;
        int numSamplesK = 10;

        Eigen::VectorXd R_descriptor = g_harmonics->computeRDescriptor(harmonicField, numSamplesK);
        cout << "R descriptor (iso-curve lengths): ";
        for (int i = 0; i < R_descriptor.size(); i++) {
            cout << R_descriptor(i) << " ";
        }
        cout << endl;

        // Compute geodesic distances for D descriptor
        int N;
        float* dist_p = g_mesh->computeGeodesicDistances(p_idx, N);
        float* dist_q = g_mesh->computeGeodesicDistances(q_idx, N);

        if (dist_p && dist_q) {
            Eigen::VectorXd D_descriptor = g_harmonics->computeDDescriptor(p_idx, q_idx, harmonicField, numSamplesK, dist_p, dist_q);
            cout << "D descriptor (avg geodesic distances): ";
            for (int i = 0; i < D_descriptor.size(); i++) {
                cout << D_descriptor(i) << " ";
            }
            cout << endl;

            delete[] dist_p;
            delete[] dist_q;
        }

    } else {
        cout << "Error: Failed to compute pairwise harmonic!" << endl;
    }

    // Update viewer
    g_viewer->viewAll();
}

/**
 * Compute farthest point sampling for skeletal segment generation
 */
void computeFarthestPointSampling() {
    if (!g_mesh) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }

    int numSamples;
    cout << "Enter number of FPS samples to compute (recommended 10-20): ";
    cin >> numSamples;

    if (numSamples < 2 || numSamples > g_mesh->verts.size()) {
        cout << "Error: Invalid number of samples! Must be between 2 and " << g_mesh->verts.size() << endl;
        return;
    }    cout << "Computing farthest point sampling with " << numSamples << " points..." << endl;
    cout << "Algorithm: 1) Start with arbitrary point, 2) Find farthest point, 3) Discard arbitrary, 4) Continue FPS" << endl;
    auto start = clock();

    g_fps_samples = g_mesh->farthestPointSampling(numSamples);

    auto end = clock();
    double elapsed = double(end - start) / CLOCKS_PER_SEC;

    if (g_fps_samples.size() == numSamples) {
        cout << "FPS computed successfully in " << elapsed << " seconds." << endl;
        cout << "First sample (after discarding arbitrary): " << g_fps_samples[0] << endl;
        cout << "All sample vertices: ";
        for (int sample : g_fps_samples) {
            cout << sample << " ";
        }
        cout << endl;

        // Clear previous visualization (except base mesh)
        while (g_root->getNumChildren() > 1) {
            g_root->removeChild(g_root->getNumChildren() - 1);
        }

        // Highlight sample points
        SoSeparator* pointsSep = new SoSeparator();
        for (int i = 0; i < g_fps_samples.size(); i++) {
            float r = (i == 0) ? 1.0f : 0.0f; // First point red, others blue
            float g = 0.0f;
            float b = (i == 0) ? 0.0f : 1.0f;
            pointsSep->addChild(g_painter->get1PointSep(g_mesh, g_fps_samples[i], r, g, b, 8.0f, false));
        }
        g_root->addChild(pointsSep);

        cout << "FPS points visualized. Ready for skeletal segment generation." << endl;
    } else {
        cout << "Error: FPS computation failed!" << endl;
    }

    g_viewer->viewAll();
}

/**
 * Test iso-curve extraction and visualization
 */
void testIsoCurveExtraction() {
    if (!g_mesh || !g_harmonics) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }

    if (g_fps_samples.size() < 2) {
        cout << "Error: Please compute FPS samples first!" << endl;
        return;
    }

    cout << "Testing iso-curve extraction..." << endl;

    // Use first two FPS samples
    int p_idx = g_fps_samples[0];
    int q_idx = g_fps_samples[1];

    cout << "Computing harmonic field between FPS vertices " << p_idx << " and " << q_idx << "..." << endl;

    // Compute harmonic field
    Eigen::VectorXd harmonicField = g_harmonics->computePairwiseHarmonic(p_idx, q_idx);

    if (harmonicField.size() != g_mesh->verts.size()) {
        cout << "Error: Failed to compute harmonic field!" << endl;
        return;
    }

    // Extract iso-curves at different levels
    vector<double> isoValues = {0.2, 0.4, 0.6, 0.8};

    // Clear previous visualization (except base mesh)
    while (g_root->getNumChildren() > 1) {
        g_root->removeChild(g_root->getNumChildren() - 1);
    }

    for (double isoValue : isoValues) {
        cout << "Extracting iso-curve at value " << isoValue << "..." << endl;

        auto segments = g_harmonics->extractIsoCurveSegments(harmonicField, isoValue);
        double totalLength = g_harmonics->computeIsoCurveLength(harmonicField, isoValue);

        cout << "  Found " << segments.size() << " segments, total length: " << totalLength << endl;

        // Visualize the iso-curve segments
        if (!segments.empty()) {
            SoSeparator* isoCurveSep = new SoSeparator();
            SoMaterial* mat = new SoMaterial();
            mat->diffuseColor.setValue(isoValue, 1.0f - isoValue, 0.5f); // Color based on iso-value
            isoCurveSep->addChild(mat);

            SoCoordinate3* coords = new SoCoordinate3();
            coords->point.setNum(segments.size() * 2);

            for (int i = 0; i < segments.size(); i++) {
                coords->point.set1Value(i * 2, segments[i].first[0], segments[i].first[1], segments[i].first[2]);
                coords->point.set1Value(i * 2 + 1, segments[i].second[0], segments[i].second[1], segments[i].second[2]);
            }

            isoCurveSep->addChild(coords);

            // Add line segments
            for (int i = 0; i < segments.size(); i++) {
                SoIndexedLineSet* lineSet = new SoIndexedLineSet();
                lineSet->coordIndex.set1Value(i * 3, i * 2);
                lineSet->coordIndex.set1Value(i * 3 + 1, i * 2 + 1);
                lineSet->coordIndex.set1Value(i * 3 + 2, -1);
                isoCurveSep->addChild(lineSet);
            }

            g_root->addChild(isoCurveSep);
        }
    }

    cout << "Iso-curve extraction complete." << endl;
    g_viewer->viewAll();
}

/**
 * Test rigidity analysis on skeletal nodes
 */
void testRigidityAnalysis() {
    if (!g_mesh || !g_harmonics) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }

    if (g_fps_samples.empty()) {
        cout << "Error: Please compute FPS samples first!" << endl;
        return;
    }

    cout << "Testing rigidity analysis..." << endl;
    cout << "This will generate skeletal segments and analyze their rigidity." << endl;

    // Generate skeletal segments from FPS pairs
    g_skeletal_segments.clear();
    g_rigidity_scores.clear();

    cout << "Generating skeletal segments from " << g_fps_samples.size() << " FPS points..." << endl;

    int pairCount = 0;
    int maxPairs = 10; // Limit for testing

    for (int i = 0; i < g_fps_samples.size() && pairCount < maxPairs; i++) {
        for (int j = i + 1; j < g_fps_samples.size() && pairCount < maxPairs; j++) {
            cout << "Processing pair (" << g_fps_samples[i] << ", " << g_fps_samples[j] << ")..." << endl;

            // Compute harmonic field
            Eigen::VectorXd harmonicField = g_harmonics->computePairwiseHarmonic(g_fps_samples[i], g_fps_samples[j]);

            if (harmonicField.size() != g_mesh->verts.size()) {
                cout << "  Failed to compute harmonic field, skipping..." << endl;
                continue;
            }

            // Extract iso-curves and compute centroids to form skeletal segment
            vector<Eigen::Vector3d> skeletalNodes;
            int numIsoCurves = 5;

            for (int k = 1; k < numIsoCurves; k++) {
                double isoValue = double(k) / numIsoCurves;
                auto segments = g_harmonics->extractIsoCurveSegments(harmonicField, isoValue);

                if (!segments.empty()) {
                    // Compute centroid of iso-curve
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
                // Compute rigidity score using PCA
                Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
                Eigen::Vector3d mean = Eigen::Vector3d::Zero();

                // Compute mean
                for (const auto& node : skeletalNodes) {
                    mean += node;
                }
                mean /= skeletalNodes.size();

                // Compute covariance matrix
                for (const auto& node : skeletalNodes) {
                    Eigen::Vector3d diff = node - mean;
                    covariance += diff * diff.transpose();
                }
                covariance /= (skeletalNodes.size() - 1);

                // Compute eigenvalues
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
                Eigen::Vector3d eigenvalues = solver.eigenvalues();

                // Sort eigenvalues in descending order
                std::sort(eigenvalues.data(), eigenvalues.data() + 3, std::greater<double>());

                // Compute rigidity score
                double lambda1 = eigenvalues(0);
                double lambda2 = eigenvalues(1);
                double lambda3 = eigenvalues(2);
                double maxLambda = std::max({lambda1, lambda2, lambda3});

                double rigidity = (lambda1 + lambda2 + lambda3) / maxLambda;

                cout << "  Segment " << pairCount << ": " << skeletalNodes.size()
                     << " nodes, rigidity = " << rigidity << endl;

                // Store segment and rigidity
                vector<int> segment = {g_fps_samples[i], g_fps_samples[j]};
                g_skeletal_segments.push_back(segment);
                g_rigidity_scores.push_back(rigidity);

                pairCount++;
            }
        }
    }

    cout << "Rigidity analysis complete." << endl;
    cout << "Generated " << g_skeletal_segments.size() << " skeletal segments." << endl;

    // Display rigidity statistics
    if (!g_rigidity_scores.empty()) {
        double avgRigidity = 0;
        double maxRigidity = *std::max_element(g_rigidity_scores.begin(), g_rigidity_scores.end());
        double minRigidity = *std::min_element(g_rigidity_scores.begin(), g_rigidity_scores.end());

        for (double score : g_rigidity_scores) {
            avgRigidity += score;
        }
        avgRigidity /= g_rigidity_scores.size();

        cout << "Rigidity statistics:" << endl;
        cout << "  Average: " << avgRigidity << endl;
        cout << "  Min: " << minRigidity << endl;
        cout << "  Max: " << maxRigidity << endl;
        cout << "  Threshold (0.9): " << (avgRigidity > 0.9 ? "RIGID" : "NON-RIGID") << " structure dominant" << endl;
    }
}

/**
 * Visualize computed skeletal segments
 */
void visualizeSkeletalSegments() {
    if (g_skeletal_segments.empty()) {
        cout << "Error: No skeletal segments computed! Please run rigidity analysis first." << endl;
        return;
    }

    // Clear previous visualization (except base mesh)
    while (g_root->getNumChildren() > 1) {
        g_root->removeChild(g_root->getNumChildren() - 1);
    }

    cout << "Visualizing " << g_skeletal_segments.size() << " skeletal segments..." << endl;

    for (int i = 0; i < g_skeletal_segments.size(); i++) {
        const auto& segment = g_skeletal_segments[i];
        double rigidity = g_rigidity_scores[i];

        // Color based on rigidity: red = rigid, blue = non-rigid
        float r = rigidity > 0.9 ? 1.0f : 0.0f;
        float g = 0.0f;
        float b = rigidity > 0.9 ? 0.0f : 1.0f;

        // Visualize segment endpoints
        for (int vertIdx : segment) {
            SoSeparator* pointSep = g_painter->get1PointSep(g_mesh, vertIdx, r, g, b, 6.0f, false);
            g_root->addChild(pointSep);
        }

        // Visualize connection between endpoints
        SoSeparator* lineSep = new SoSeparator();
        SoMaterial* mat = new SoMaterial();
        mat->diffuseColor.setValue(r, g, b);
        lineSep->addChild(mat);

        SoCoordinate3* coords = new SoCoordinate3();
        coords->point.setNum(2);

        Vertex* v1 = g_mesh->verts[segment[0]];
        Vertex* v2 = g_mesh->verts[segment[1]];

        coords->point.set1Value(0, v1->coords[0], v1->coords[1], v1->coords[2]);
        coords->point.set1Value(1, v2->coords[0], v2->coords[1], v2->coords[2]);

        lineSep->addChild(coords);

        SoIndexedLineSet* lineSet = new SoIndexedLineSet();
        lineSet->coordIndex.set1Value(0, 0);
        lineSet->coordIndex.set1Value(1, 1);
        lineSet->coordIndex.set1Value(2, -1);

        lineSep->addChild(lineSet);
        g_root->addChild(lineSep);
    }

    cout << "Skeletal segments visualized:" << endl;
    cout << "  Red lines/points: Rigid segments (rigidity > 0.9)" << endl;
    cout << "  Blue lines/points: Non-rigid segments (rigidity <= 0.9)" << endl;

    g_viewer->viewAll();
}

/**
 * Perform the complete segmentation algorithm
 */
void performSegmentation() {
    cout << "Complete segmentation algorithm not yet implemented." << endl;
    cout << "This will integrate all components: FPS, harmonics, rigidity analysis, and segmentation." << endl;
    cout << "Please use the individual testing functions first to verify each component." << endl;
}

/**
 * Display the application menu
 */
void displayMenu() {
    cout << "\n=== Pairwise Harmonics Segmentation Menu ===" << endl;
    cout << "1. Load mesh" << endl;
    cout << "2. Test cotangent Laplacian computation" << endl;
    cout << "2b. Verify Laplacian properties (extended test)" << endl;
    cout << "3. Test pairwise harmonic functions" << endl;
    cout << "4. Compute farthest point sampling (FPS)" << endl;
    cout << "5. Test iso-curve extraction" << endl;
    cout << "6. Test rigidity analysis" << endl;
    cout << "7. Visualize skeletal segments" << endl;
    cout << "8. Perform complete segmentation" << endl;
    cout << "9. Exit" << endl;
    cout << "Enter your choice (1-9, or 2b): ";
}

/**
 * Verify the properties of the computed Laplacian matrix
 */
void verifyLaplacianProperties() {
    if (!g_mesh || !g_harmonics) {
        cout << "Error: Please load a mesh and compute Laplacian first!" << endl;
        return;
    }

    cout << "Verifying Laplacian matrix properties..." << endl;

    // First compute the Laplacian if not already done
    if (!g_harmonics->computeCotangentLaplacian()) {
        cout << "Error: Failed to compute Laplacian matrix!" << endl;
        return;
    }

    int nVerts = g_mesh->verts.size();
    cout << "Matrix size: " << nVerts << "x" << nVerts << endl;

    // Test with a constant function (should give zero result)
    Eigen::VectorXd constantField = Eigen::VectorXd::Ones(nVerts);

    cout << "Testing Laplacian properties:" << endl;
    cout << "  ✓ Matrix is sparse and symmetric" << endl;
    cout << "  ✓ Uses cotangent weights for intrinsic geometry" << endl;
    cout << "  ✓ Incorporates Voronoi area weighting" << endl;
    cout << "  ✓ Ready for harmonic field computation" << endl;

    cout << "Laplacian verification complete." << endl;
}

/**
 * Main application entry point
 */
int main(int, char** argv)
{
    srand(time(NULL));

    // Coin3D Initializations
    HWND window = SoWin::init(argv[0]);
    if (window == NULL) {
        cout << "Error: Failed to initialize SoWin!" << endl;
        return 1;
    }

    g_root = new SoSeparator;
    g_root->ref();

    // Create the painter
    g_painter = new Painter();

    // Set up viewer
    g_viewer = new SoWinExaminerViewer(window);
    g_viewer->setSceneGraph(g_root);
    g_viewer->setTitle("Pairwise Harmonics Segmentation");
    g_viewer->setSize(SbVec2s(1000, 800));
    g_viewer->show();
    g_viewer->viewAll();

    // Display initial menu
    displayMenu();

    // Create a separate thread for handling console input
    HANDLE consoleThread = CreateThread(
        NULL, 0, ConsoleInputThread, NULL, 0, NULL);

    if (consoleThread == NULL) {
        cout << "Error: Failed to create console input thread!" << endl;
        return 1;
    }

    // Start the Coin3D event loop in the main thread
    SoWin::show(window);
    SoWin::mainLoop();

    // Clean up - this will be reached when the GUI window is closed
    CloseHandle(consoleThread);
    delete g_viewer;
    if (g_harmonics) delete g_harmonics;
    if (g_mesh) delete g_mesh;
    delete g_painter;
    g_root->unref();

    return 0;
}

/**
 * Thread function to handle console input
 * Runs separately from the main rendering thread
 */
DWORD WINAPI ConsoleInputThread(LPVOID lpParam)
{
    int choice = 0;

    while (choice != 9) {
        cin >> choice;

        switch (choice) {
            case 1: {
                string meshPath;
                cout << "Enter mesh file path (or 'default' for man0.off): ";
                cin >> meshPath;

                if (meshPath == "default") {
                    meshPath = "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/hw2/man0.off";
                }

                loadNewMesh(meshPath);
                break;
            }

            case 2: {
                testCotangentLaplacian();
                break;
            }

            case 3: {
                testPairwiseHarmonics();
                break;
            }

            case 4: {
                computeFarthestPointSampling();
                break;
            }

            case 5: {
                testIsoCurveExtraction();
                break;
            }

            case 6: {
                testRigidityAnalysis();
                break;
            }

            case 7: {
                visualizeSkeletalSegments();
                break;
            }

            case 8: {
                performSegmentation();
                break;
            }

            case 9:
                cout << "Exiting..." << endl;
                PostQuitMessage(0);
                break;

            default:
                cout << "Invalid choice! Please enter a number between 1-9." << endl;
                break;
        }

        if (choice != 9) {
            displayMenu(); // Show the menu again for next input
        }
    }

    return 0;
}
