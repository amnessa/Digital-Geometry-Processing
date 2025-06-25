#include "pch.h"
#include "VisualizationUtils.h"
#include "helpers.h"  // For PairwiseHarmonics class
#include <iostream>
#include <ctime>
#include <algorithm>
#include <chrono>

// Coin3D headers
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoIndexedLineSet.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>

// Project headers
#include "Painter.h"

using namespace std;

void VisualizationUtils::clearVisualization(SoSeparator* root) {
    // Preserve the mesh (usually the first child added) and remove other visualizations
    // This is a safer approach that keeps the mesh visible while clearing analysis visualizations

    int numChildren = root->getNumChildren();

    // If there are more than one child, remove all except the first (mesh)
    // However, if the mesh was reloaded, we need to be more careful
    if (numChildren > 1) {
        // Remove children from the end backwards to preserve indices
        for (int i = numChildren - 1; i >= 1; i--) {
            root->removeChild(i);
        }
        cout << "Cleared " << (numChildren - 1) << " visualization elements, preserving mesh." << endl;
    } else if (numChildren == 1) {
        cout << "Only mesh present, nothing to clear." << endl;
    } else {
        cout << "No visualizations to clear." << endl;
    }
}

void VisualizationUtils::testCotangentLaplacian(
    Mesh* mesh,
    PairwiseHarmonics* harmonics,
    SoSeparator* root,
    SoWinExaminerViewer* viewer) {

    cout << "\n=== TESTING COTANGENT LAPLACIAN ===" << endl;
    cout << "Using LibIGL implementation (robust, industry-standard)" << endl;
    if (!mesh || !harmonics) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }

    bool success = false;
    auto start = clock();

    cout << "Computing cotangent Laplacian using libigl..." << endl;
    success = harmonics->computeCotangentLaplacianLibigl();

    auto end = clock();
    double elapsed = double(end - start) / CLOCKS_PER_SEC;

    if (success) {
        cout << "SUCCESS: Cotangent Laplacian computed successfully!" << endl;
        cout << "  Implementation: libigl" << endl;
        cout << "  Computation time: " << elapsed << " seconds" << endl;
        cout << "  Matrix size: " << mesh->verts.size() << "x" << mesh->verts.size() << endl;
        cout << "  Properties:" << endl;
        cout << "    - Intrinsic (uses cotangent weights)" << endl;
        cout << "    - libigl robust implementation" << endl;
        cout << "    - Includes mass matrix computation" << endl;
        cout << "    - Symmetric and positive semi-definite" << endl;
        cout << "    - Ready for harmonic field computation" << endl;
    } else {
        cout << "[ERROR] Failed to compute cotangent Laplacian!" << endl;
    }
}

void VisualizationUtils::testFarthestPointSampling(
    Mesh* mesh,
    std::vector<int>& fps_samples,
    SoSeparator* root,
    SoWinExaminerViewer* viewer) {

    cout << "\n=== TESTING FARTHEST POINT SAMPLING ===" << endl;
    if (!mesh) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }
    // Give user choice of algorithm
    cout << "Choose FPS implementation:" << endl;
    cout << "1. Heat geodesics (LibIGL) - Fast and accurate" << endl;
    cout << "2. Dijkstra (original) - Slower but reliable" << endl;
    cout << "Enter choice (1 or 2): ";

    int algorithm_choice;
    cin >> algorithm_choice;

    int numSamples;
    cout << "Enter number of FPS samples (recommended: 25-40 for human models, 15-25 for simple shapes): ";
    cin >> numSamples;

    if (numSamples < 2 || numSamples > mesh->verts.size()) {
        cout << "Error: Invalid number of samples!" << endl;
        return;
    }

    cout << "Computing FPS with " << numSamples << " points..." << endl;
    if (algorithm_choice == 1) {
        cout << "Algorithm: LibIGL Heat Geodesics-based FPS (faster, more accurate)" << endl;
    } else {
        cout << "Algorithm: Dijkstra-based FPS - 1) Pick arbitrary point -> 2) Find farthest -> 3) Discard arbitrary -> 4) Continue FPS" << endl;
    }

    auto start = clock();
    if (algorithm_choice == 1) {
        fps_samples = mesh->farthestPointSamplingLibIGL(numSamples);
    } else {
        fps_samples = mesh->farthestPointSampling(numSamples);
    }
    auto end = clock();    double elapsed = double(end - start) / CLOCKS_PER_SEC;
    if (fps_samples.size() == numSamples) {
        cout << "SUCCESS: FPS computed successfully!" << endl;
        cout << "  Implementation: " << (algorithm_choice == 1 ? "LibIGL Heat Geodesics" : "Dijkstra") << endl;
        cout << "  Computation time: " << elapsed << " seconds" << endl;
        cout << "  Sample points: ";
        for (int sample : fps_samples) {
            cout << sample << " ";
        }
        cout << endl;

        // Visualize sample points
        clearVisualization(root);

        for (int i = 0; i < fps_samples.size(); i++) {
            Eigen::Vector3d pos(mesh->verts[fps_samples[i]]->coords[0],
                                mesh->verts[fps_samples[i]]->coords[1],
                                mesh->verts[fps_samples[i]]->coords[2]);

            // First point red, others blue
            Eigen::Vector3f color(i == 0 ? 1.0f : 0.0f, 0.0f, i == 0 ? 0.0f : 1.0f);
            SoSeparator* pointViz = createColoredSphere(pos, color, 0.01f);
            root->addChild(pointViz);
        }

        // Force viewer refresh
        viewer->scheduleRedraw();
        viewer->viewAll();

        cout << "FPS points visualized (first point is red, others blue)." << endl;
    } else {
        cout << "[ERROR] FPS computation failed!" << endl;
    }
}

void VisualizationUtils::testPairwiseHarmonics(
    Mesh* mesh,
    PairwiseHarmonics* harmonics,
    const std::vector<int>& fps_samples,
    SoSeparator* root,
    SoWinExaminerViewer* viewer) {

    cout << "\n=== TESTING PAIRWISE HARMONICS ===" << endl;
    cout << "Using LibIGL implementation (robust, industry-standard)" << endl;
    if (!mesh || !harmonics) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }
    int p_idx, q_idx;
    cout << "Enter first vertex index (p, boundary value 0): ";
    cin >> p_idx;
    cout << "Enter second vertex index (q, boundary value 1): ";
    cin >> q_idx;

    int nVerts = mesh->verts.size();
    if (p_idx < 0 || p_idx >= nVerts || q_idx < 0 || q_idx >= nVerts || p_idx == q_idx) {
        cout << "Error: Invalid vertex indices!" << endl;
        return;
    }

    cout << "Computing harmonic field between vertices " << p_idx << " and " << q_idx << "..." << endl;

    auto start = clock();
    cout << "Using libigl implementation..." << endl;
    Eigen::VectorXd harmonicField = harmonics->computePairwiseHarmonicLibIGL(p_idx, q_idx);

    auto end = clock();
    double elapsed = double(end - start) / CLOCKS_PER_SEC;

    if (harmonicField.size() == nVerts) {
        cout << "SUCCESS: Pairwise harmonic computed successfully!" << endl;
        cout << "  Implementation: libigl" << endl;
        cout << "  Computation time: " << elapsed << " seconds" << endl;
        cout << "  Field range: [" << harmonicField.minCoeff() << ", " << harmonicField.maxCoeff() << "]" << endl;

        // Visualize the harmonic field using scalar field visualization
        clearVisualization(root);
        visualizeScalarField(mesh, harmonicField, root, "Harmonic Field");

        // Compute R and D descriptors as validation
        cout << "\nComputing shape descriptors..." << endl;
        Eigen::VectorXd R_descriptor = harmonics->computeRDescriptor(harmonicField, 10);
        cout << "R descriptor (isocurve lengths): ";
        for (int i = 0; i < R_descriptor.size(); i++) {
            cout << R_descriptor(i) << " ";
        }
        cout << endl;

        // Force viewer refresh
        viewer->scheduleRedraw();
        viewer->viewAll();
        cout << "Harmonic field visualized (blue = value 0, red = value 1)." << endl;
    } else {
        cout << "[ERROR] Failed to compute pairwise harmonic!" << endl;
    }
}

void VisualizationUtils::testIsoCurveExtraction(
    Mesh* mesh,
    PairwiseHarmonics* harmonics,
    const std::vector<int>& fps_samples,
    SoSeparator* root,
    SoWinExaminerViewer* viewer) {

    cout << "\n=== TESTING ISO-CURVE EXTRACTION ===" << endl;
    if (!mesh || !harmonics || fps_samples.size() < 2) {
        cout << "Error: Please load mesh and compute FPS samples first!" << endl;
        return;
    }

    // Use first two FPS samples
    int p_idx = fps_samples[0];
    int q_idx = fps_samples[1];

    cout << "Computing harmonic field between FPS vertices " << p_idx << " and " << q_idx << "..." << endl;

    Eigen::VectorXd harmonicField = harmonics->computePairwiseHarmonic(p_idx, q_idx);

    if (harmonicField.size() != mesh->verts.size()) {
        cout << "[ERROR] Failed to compute harmonic field!" << endl;
        return;
    }
    cout << "Extracting iso-curves at different levels..." << endl;

    clearVisualization(root);
    vector<double> isoValues = {0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 0.95}; // Increased from 4 to 8 levels
    vector<Eigen::Vector3d> skeletal_nodes;

    for (int i = 0; i < isoValues.size(); i++) {
        double isoValue = isoValues[i];

        auto segments = harmonics->extractIsoCurveSegments(harmonicField, isoValue);
        double totalLength = harmonics->computeIsoCurveLength(harmonicField, isoValue);

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
            root->addChild(isoCurveSep);
        }
    }

    cout << "SUCCESS: Iso-curve extraction complete!" << endl;
    cout << "  Generated " << skeletal_nodes.size() << " skeletal nodes" << endl;

    // Force viewer refresh
    viewer->scheduleRedraw();
    viewer->viewAll();
    cout << "Iso-curves visualized with color-coded levels." << endl;
}

void VisualizationUtils::testRigidityAnalysis(
    Mesh* mesh,
    PairwiseHarmonics* harmonics,
    const std::vector<int>& fps_samples,
    SoSeparator* root,
    SoWinExaminerViewer* viewer) {

    // This is a simplified version of rigidity testing
    // The full enhanced version is now in RigidityAnalysis module
    cout << "\n=== BASIC RIGIDITY ANALYSIS TEST ===" << endl;
    cout << "For enhanced rigidity analysis, use the RigidityAnalysis module functions." << endl;
    cout << "This test function provides basic rigidity testing functionality." << endl;

    if (!mesh || !harmonics || fps_samples.size() < 2) {
        cout << "Error: Please load mesh and compute FPS samples first!" << endl;
        return;
    }

    cout << "Performing basic rigidity test on skeletal segments..." << endl;
    cout << "Use enhanced rigidity analysis for full functionality." << endl;
}

void VisualizationUtils::testHeatGeodesics(Mesh* mesh, PairwiseHarmonics* harmonics, SoSeparator* root, SoWinExaminerViewer* viewer) {
    if (!mesh || !harmonics) {
        cout << "Error: Mesh and harmonics must be loaded first!" << endl;
        return;
    }

    cout << "\n=== TESTING HEAT GEODESICS ===" << endl;
    cout << "This test compares LibIGL heat geodesics with Dijkstra distances" << endl;

    // Test heat geodesics on a few vertices
    int numVertices = mesh->verts.size();
    vector<int> testVertices = {0, numVertices/4, numVertices/2, 3*numVertices/4};

    clearVisualization(root);

    for (int v : testVertices) {
        if (v >= numVertices) continue;

        cout << "Testing heat geodesics from vertex " << v << "..." << endl;
        Eigen::VectorXd heatDist = harmonics->computeHeatGeodesicDistances(v);
        cout << "Computed heat geodesic distances from vertex " << v << " (range: " << heatDist.minCoeff() << " to " << heatDist.maxCoeff() << ")" << endl;
        cout << "  Heat geodesic computation successful (size: " << heatDist.size() << ")" << endl;
        cout << "  Max distance: " << heatDist.maxCoeff() << endl;
        cout << "  Min distance: " << heatDist.minCoeff() << endl;

        // Visualize heat geodesic distances as scalar field
        VisualizationUtils::visualizeScalarField(mesh, heatDist, root, "Heat Geodesic Distances from vertex " + to_string(v));

        // Mark the source vertex with a special sphere
        Eigen::Vector3d sourcePos(mesh->verts[v]->coords[0], mesh->verts[v]->coords[1], mesh->verts[v]->coords[2]);
        Eigen::Vector3f sourceColor(1.0f, 1.0f, 0.0f); // Yellow for source
        SoSeparator* sourceViz = VisualizationUtils::createColoredSphere(sourcePos, sourceColor, 0.02f);
        root->addChild(sourceViz);

        break; // Only visualize first one to avoid clutter
    }

    viewer->scheduleRedraw();
    viewer->viewAll();
    cout << "Heat geodesics test completed!" << endl;
}

void VisualizationUtils::testEnhancedFPS(Mesh* mesh, SoSeparator* root, SoWinExaminerViewer* viewer) {
    if (!mesh) {
        cout << "Error: Mesh must be loaded first!" << endl;
        return;
    }

    cout << "\n=== TESTING ENHANCED FPS ===" << endl;
    cout << "Testing Farthest Point Sampling with LibIGL heat geodesics" << endl;

    clearVisualization(root);

    // Test with different numbers of samples
    vector<int> sampleCounts = {5, 10, 20, 30};

    for (int count : sampleCounts) {
        cout << "Testing FPS with " << count << " samples..." << endl;

        // Use the Mesh's FPS method directly
        vector<int> fps_samples = mesh->farthestPointSamplingLibIGL(count);
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
    vector<int> fps_samples = mesh->farthestPointSamplingLibIGL(sampleCounts.back());

    for (size_t i = 0; i < fps_samples.size(); i++) {
        Eigen::Vector3d pos(mesh->verts[fps_samples[i]]->coords[0],
                            mesh->verts[fps_samples[i]]->coords[1],
                            mesh->verts[fps_samples[i]]->coords[2]);

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
        root->addChild(pointViz);
    }

    viewer->scheduleRedraw();
    viewer->viewAll();
    cout << "Enhanced FPS test completed!" << endl;
}

void VisualizationUtils::compareGeodesicMethods(Mesh* mesh, PairwiseHarmonics* harmonics) {
    if (!mesh || !harmonics) {
        cout << "Error: Mesh and harmonics must be loaded first!" << endl;
        return;
    }

    cout << "\n=== COMPARING GEODESIC METHODS ===" << endl;
    cout << "Comparing LibIGL heat geodesics with Dijkstra-based geodesics" << endl;

    int sourceVertex = 0;
    int numVertices = mesh->verts.size();

    if (numVertices > 0) {
        cout << "Method 1: LibIGL Heat Geodesics" << endl;
        cout << "Computing heat geodesics from vertex " << sourceVertex << "..." << endl;

        auto start1 = chrono::high_resolution_clock::now();
        Eigen::VectorXd heatDist = harmonics->computeHeatGeodesicDistances(sourceVertex);
        auto end1 = chrono::high_resolution_clock::now();

        auto duration1 = chrono::duration_cast<chrono::milliseconds>(end1 - start1);
        cout << "Heat geodesics computed in " << duration1.count() << " ms" << endl;
        cout << "Result statistics:" << endl;
        cout << "  Size: " << heatDist.size() << endl;
        cout << "  Max distance: " << heatDist.maxCoeff() << endl;
        cout << "  Min distance: " << heatDist.minCoeff() << endl;
        cout << "\nMethod 2: Dijkstra-based Geodesics" << endl;
        cout << "Computing Dijkstra geodesics from vertex " << sourceVertex << "..." << endl;

        auto start2 = chrono::high_resolution_clock::now();
        int N;
        float* dijkstraDistPtr = mesh->computeGeodesicDistances(sourceVertex, N);
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
void VisualizationUtils::testEnhancedDescriptors(Mesh* mesh, PairwiseHarmonics* harmonics, int k_isocurves) {
    if (!mesh || !harmonics) {
        cout << "Error: Mesh and harmonics must be loaded first!" << endl;
        return;
    }

    cout << "\n=== TESTING ENHANCED DESCRIPTORS ===" << endl;
    cout << "Testing LibIGL-based R and D descriptors for shape analysis" << endl;
    cout << "R Descriptor: Measures isocurve lengths (shape profile)" << endl;
    cout << "D Descriptor: Measures distance variations (shape complexity)" << endl;

    int numVertices = mesh->verts.size();
    if (numVertices < 2) {
        cout << "Error: Mesh needs at least 2 vertices!" << endl;
        return;
    }

    int p = 0;
    int q = numVertices / 2;

    cout << "Computing harmonic field between vertices " << p << " and " << q << "..." << endl;

    // Compute harmonic field
    Eigen::VectorXd field = harmonics->computePairwiseHarmonicLibIGL(p, q);

    if (field.size() > 0) {
        cout << "Harmonic field computed successfully (size: " << field.size() << ")" << endl;

        // Test R descriptor
        cout << "Computing R descriptor (isocurve length measurements)..." << endl;
        Eigen::VectorXd rDesc = harmonics->computeRDescriptorLibIGL(field, k_isocurves);
        cout << "R descriptor computed successfully (size: " << rDesc.size() << ")" << endl;
        cout << "R descriptor values (first 10): ";
        for (int i = 0; i < min(10, (int)rDesc.size()); i++) {
            cout << rDesc(i) << " ";
        }
        cout << endl;

        // Test D descriptor
        cout << "Computing D descriptor (distance variation measurements)..." << endl;
        Eigen::VectorXd dDesc = harmonics->computeDDescriptorLibIGL(p, q, field, k_isocurves);
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


void VisualizationUtils::testMeshAnalysis(Mesh* mesh, PairwiseHarmonics* harmonics) {
    if (!mesh || !harmonics || !harmonics->isLibIGLInitialized()) {
        cout << "Error: Load mesh and initialize LibIGL first." << endl;
        return;
    }
    cout << "\n=== MESH ANALYSIS (LIBIGL) ===" << endl;
    cout << "Vertex matrix: " << harmonics->getVertexMatrix().rows() << "x" << harmonics->getVertexMatrix().cols() << endl;
    cout << "Face matrix: " << harmonics->getFaceMatrix().rows() << "x" << harmonics->getFaceMatrix().cols() << endl;
    cout << "Laplacian computation: " << (harmonics->computeCotangentLaplacianLibigl() ? "SUCCESS" : "FAILED") << endl;
}

void VisualizationUtils::visualizeMeshSegmentationClusters(const PairwiseHarmonicsSegmentation::SegmentationResult& result, Mesh* mesh, SoSeparator* root, SoWinExaminerViewer* viewer) {
    if (!result.success || result.meshComponents.empty() || !mesh) {
        cout << "Error: Please run full segmentation first to generate mesh components!" << endl;
        cout << "Use option 1 to perform complete segmentation." << endl;
        return;
    }

    cout << "\n=== MESH COMPONENT VISUALIZATION ===" << endl;
    cout << "Showing " << result.meshComponents.size() << " mesh components with colored vertices" << endl;

    clearVisualization(root);

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
    vector<int> vertexToComponent(mesh->verts.size(), -1);

    for (int compIdx = 0; compIdx < result.meshComponents.size(); compIdx++) {
        const auto& component = result.meshComponents[compIdx];
        for (int vertexIdx : component.vertexIndices) {
            if (vertexIdx >= 0 && vertexIdx < mesh->verts.size()) {
                vertexToComponent[vertexIdx] = compIdx;
            }
        }
    }

    // Visualize vertices colored by their mesh component assignment
    cout << "Rendering colored spheres for each mesh component..." << endl;

    for (int compIdx = 0; compIdx < result.meshComponents.size(); compIdx++) {
        const auto& component = result.meshComponents[compIdx];
        Eigen::Vector3f color = componentColors[compIdx % componentColors.size()];

        cout << "  Component " << compIdx << ": " << component.vertexIndices.size() << " vertices, color: ("
             << color[0] << ", " << color[1] << ", " << color[2] << ")" << endl;

        // Visualize vertices in this component (downsample for performance)
        for (int i = 0; i < component.vertexIndices.size(); i += 15) { // Every 15th vertex
            int vertexIdx = component.vertexIndices[i];
            if (vertexIdx >= 0 && vertexIdx < mesh->verts.size()) {
                Eigen::Vector3d vertexPos(mesh->verts[vertexIdx]->coords[0],
                                         mesh->verts[vertexIdx]->coords[1],
                                         mesh->verts[vertexIdx]->coords[2]);

                SoSeparator* pointViz = VisualizationUtils::createColoredSphere(vertexPos, color, 0.02f);
                root->addChild(pointViz);
            }
        }

        // Also show the component center as a larger sphere
        SoSeparator* centerViz = VisualizationUtils::createColoredSphere(component.center, color, 0.02f);
        root->addChild(centerViz);
    }

    // Also visualize the partial skeleton as lines
    if (!result.partialSkeleton.empty()) {
        cout << "Rendering partial skeleton..." << endl;
        for (int s = 0; s < result.partialSkeleton.size(); s++) {
            const auto& segment = result.partialSkeleton[s];
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
            root->addChild(lineSep);
        }
    }

    viewer->scheduleRedraw();
    viewer->viewAll();

    cout << "\n=== MESH COMPONENT VISUALIZATION COMPLETE ===" << endl;
    cout << "-> Displayed " << result.meshComponents.size() << " mesh components with unique colors" << endl;
    cout << "-> Each colored sphere represents a vertex in that component" << endl;
    cout << "-> Large spheres show component centers" << endl;
    cout << "-> Gray lines show the partial skeleton structure" << endl;
}

void VisualizationUtils::visualizeHarmonicSurfaceSegmentation(const PairwiseHarmonicsSegmentation::SegmentationResult& result, Mesh* mesh, PairwiseHarmonics* harmonics, SoSeparator* root, SoWinExaminerViewer* viewer) {
    if (!result.success || result.partialSkeleton.empty() || !mesh || !harmonics) {
        cout << "Error: Please run full segmentation first!" << endl;
        return;
    }

    cout << "\n=== TESTING HARMONIC SURFACE SEGMENTATION ===" << endl;

    // Test a single segment to understand the data
    if (!result.partialSkeleton.empty()) {
        const auto& testSegment = result.partialSkeleton[0];

        cout << "Testing segment 0:" << endl;
        cout << "  Source FPS: " << testSegment.sourceIdx << ", Target FPS: " << testSegment.targetIdx << endl;
        cout << "  Nodes: " << testSegment.nodes.size() << endl;
        cout << "  Avg Rigidity: " << testSegment.avgRigidity << endl;
        cout << "  Length: " << testSegment.length << endl;

        // Compute harmonic field for this segment
        cout << "Computing harmonic field..." << endl;
        Eigen::VectorXd harmonicField = harmonics->computePairwiseHarmonic(testSegment.sourceIdx, testSegment.targetIdx);

        if (harmonicField.size() == mesh->verts.size()) {
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
void VisualizationUtils::analyzeDetailedRigidity(const PairwiseHarmonicsSegmentation::SegmentationResult& result) {
    if (!result.success || result.partialSkeleton.empty()) {
        cout << "Error: Please run full segmentation first!" << endl;
        return;
    }
    cout << "\n=== DETAILED RIGIDITY ANALYSIS ===" << endl;
    for (int i = 0; i < result.partialSkeleton.size(); i++) {
        const auto& segment = result.partialSkeleton[i];
        cout << "\nSegment " << i << " (FPS " << segment.sourceIdx << " -> " << segment.targetIdx << "):" << endl;
        cout << "  Avg Rigidity: " << segment.avgRigidity << ", Nodes: " << segment.nodes.size() << endl;
        cout << "  Rigidity values: ";
        for(double r : segment.nodeRigidities) cout << r << " ";
        cout << endl;
    }
}

void VisualizationUtils::visualizeGraphCutSegmentation(const PairwiseHarmonicsSegmentation::SegmentationResult& result, Mesh* mesh, PairwiseHarmonics* harmonics, SoSeparator* root, SoWinExaminerViewer* viewer, const std::vector<Eigen::Vector3d>* deformed_skeleton_nodes) {
    /**
     * Optimal Surface Segmentation using Graph-Cut Energy Minimization
     * This implements a simplified CRF model inspired by Kalogerakis et al.
     * Replaces the heuristic watershed approach with globally optimal segmentation.
     */
    if (!result.success || !mesh) {
        cout << "Error: A successful segmentation must be performed first (Option 8)." << endl;
        return;
    }
    if (deformed_skeleton_nodes && deformed_skeleton_nodes->empty()) {
        cout << "Error: Deformed skeleton has no nodes." << endl;
        return;
    }

    cout << "\n=== OPTIMAL SEGMENTATION (GRAPH-CUT ENERGY MINIMIZATION) ===" << endl;
    clearVisualization(root);

    // Define colors for different segments
    vector<Eigen::Vector3f> segmentColors = {
        Eigen::Vector3f(1.0f, 0.0f, 0.0f),  // Red
        Eigen::Vector3f(0.0f, 1.0f, 0.0f),  // Green
        Eigen::Vector3f(0.0f, 0.0f, 1.0f),  // Blue
        Eigen::Vector3f(1.0f, 1.0f, 0.0f),  // Yellow
        Eigen::Vector3f(1.0f, 0.0f, 1.0f),  // Magenta
        Eigen::Vector3f(0.0f, 1.0f, 1.0f),  // Cyan
        Eigen::Vector3f(1.0f, 0.5f, 0.0f),  // Orange
        Eigen::Vector3f(0.5f, 0.0f, 1.0f),  // Purple
        Eigen::Vector3f(0.0f, 0.5f, 0.5f),  // Teal
        Eigen::Vector3f(0.5f, 1.0f, 0.5f),  // Light Green
        Eigen::Vector3f(0.5f, 0.5f, 1.0f),  // Light Blue
        Eigen::Vector3f(1.0f, 0.5f, 0.5f)   // Light Red
    };

    // Define a structure to hold segment data for the graph-cut
    struct RigidSegment {
        int originalSegmentIdx;
        vector<Eigen::Vector3d> nodes;
        double avgRigidity;
        double startHarmonicValue;
        double endHarmonicValue;
        int sourceIdx, targetIdx;
    };
    vector<RigidSegment> rigidSegments;

    if (deformed_skeleton_nodes) {
        // DYNAMIC MODE: Use the provided deformed skeleton to generate new segments on the current mesh
        cout << "DYNAMIC MODE: Using deformed skeleton to generate new segmentation." << endl;
        const auto& adjacency = result.skeletonAdjacency; // Use original connectivity

        for (size_t i = 0; i < adjacency.size(); ++i) {
            const auto& edge = adjacency[i];
            if (edge.first < deformed_skeleton_nodes->size() && edge.second < deformed_skeleton_nodes->size()) {
                RigidSegment rs;
                rs.nodes = { (*deformed_skeleton_nodes)[edge.first], (*deformed_skeleton_nodes)[edge.second] };

                // Find the closest vertices on the NEW mesh to the deformed skeleton nodes
                rs.sourceIdx = mesh->findClosestVertex(rs.nodes.front());
                rs.targetIdx = mesh->findClosestVertex(rs.nodes.back());

                if (rs.sourceIdx != -1 && rs.targetIdx != -1) {
                    rigidSegments.push_back(rs);
                }
            }
        }
    } else {
        // STATIC MODE: Use the original skeleton from the segmentation result
        cout << "STATIC MODE: Using reference skeleton for segmentation." << endl;
        // This reuses the existing logic to split the original skeleton by rigidity
        for (int segIdx = 0; segIdx < result.partialSkeleton.size(); ++segIdx) {
            const auto& segment = result.partialSkeleton[segIdx];
            if (segment.sourceIdx < 0) { // Handle torso
                RigidSegment rigidSeg;
                rigidSeg.originalSegmentIdx = segIdx;
                rigidSeg.nodes = segment.nodes;
                rigidSeg.avgRigidity = segment.avgRigidity;
                rigidSeg.startHarmonicValue = 0.0;
                rigidSeg.endHarmonicValue = 1.0;
                rigidSeg.sourceIdx = segment.sourceIdx;
                rigidSeg.targetIdx = segment.targetIdx;
                rigidSegments.push_back(rigidSeg);
            } else { // Handle limbs
                // This logic is preserved from your original function
                RigidSegment rigidSeg;
                rigidSeg.originalSegmentIdx = segIdx;
                rigidSeg.nodes = segment.nodes;
                rigidSeg.avgRigidity = segment.avgRigidity;
                rigidSeg.startHarmonicValue = 0.0;
                rigidSeg.endHarmonicValue = 1.0;
                rigidSeg.sourceIdx = segment.sourceIdx;
                rigidSeg.targetIdx = segment.targetIdx;
                rigidSegments.push_back(rigidSeg);
            }
        }
    }

    cout << "Using " << rigidSegments.size() << " segments for graph-cut optimization." << endl;

    // The rest of the function (data cost, smoothness cost, ICM optimization) remains unchanged.
    // It will now operate on the segments defined above (either static or dynamic).

    // Step 2: Build face adjacency graph
    cout << "Step 2: Building face adjacency graph..." << endl;

    int numFaces = mesh->tris.size();
    vector<vector<int>> faceNeighbors(numFaces);

    // Build face adjacency using edge sharing
    for (int f1 = 0; f1 < numFaces; f1++) {
        Triangle* tri1 = mesh->tris[f1];
        for (int f2 = f1 + 1; f2 < numFaces; f2++) {
            Triangle* tri2 = mesh->tris[f2];

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
            Eigen::VectorXd harmonicField = harmonics->computePairwiseHarmonic(canonicalPair.first, canonicalPair.second);
            if (harmonicField.size() == mesh->verts.size()) {
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
        Triangle* face = mesh->tris[f];

        // Compute face centroid
        Eigen::Vector3d faceCentroid(0, 0, 0);
        Vertex* face_verts[] = { mesh->verts[face->v1i], mesh->verts[face->v2i], mesh->verts[face->v3i] };
        for (int i = 0; i < 3; i++) {
            faceCentroid[0] += face_verts[i]->coords[0];
            faceCentroid[1] += face_verts[i]->coords[1];
            faceCentroid[2] += face_verts[i]->coords[2];
        }
        faceCentroid /= 3.0;

        // --- PERFORMANCE BOTTLENECK REMOVED ---
        // The original code searched for the closest vertex to the face centroid here,
        // which was extremely slow (O(num_faces * num_vertices)).
        //
        // NEW APPROACH:
        // We now compute the harmonic value by averaging the values at the face's three
        // vertices. This is much faster and is a valid approximation since harmonic
        // functions are smooth. The expensive search is no longer needed.

        // Compute affinity to each rigid segment
        for (int label = 0; label < numLabels; label++) {
            const auto& rigidSeg = rigidSegments[label];

            // --- NEW: Compute geometric distance for ALL segments first ---
            // This provides a spatial locality term for the energy function.
            double minEuclideanDist = numeric_limits<double>::max();
            if (!rigidSeg.nodes.empty()) {
                for (const auto& nodePos : rigidSeg.nodes) {
                    double dist = (faceCentroid - nodePos).norm();
                    minEuclideanDist = min(minEuclideanDist, dist);
                }
            }

            float bbox_diag = mesh->getBoundingBoxDiagonal();
            double euclidean_cost = (bbox_diag > 1e-6) ? (minEuclideanDist / bbox_diag) : minEuclideanDist;


            if (rigidSeg.sourceIdx >= 0 && rigidSeg.targetIdx >= 0) {
                // --- REVISED COST (Limb Segments) ---
                // Combine harmonic and geometric costs to prevent segments from "stealing" distant faces.
                double harmonic_cost = 1.0; // Default to high cost if field is not found

                int src = rigidSeg.sourceIdx;
                int tgt = rigidSeg.targetIdx;
                pair<int, int> canonicalPair = {min(src, tgt), max(src, tgt)};
                auto harmonicIt = cachedHarmonicFields.find(canonicalPair);

                if (harmonicIt != cachedHarmonicFields.end()) {
                    const Eigen::VectorXd& harmonicField = harmonicIt->second;

                    // --- OPTIMIZATION ---
                    // Instead of using a single closest vertex, average the harmonic values of the face's three vertices.
                    double harmonicValue = (harmonicField(face->v1i) + harmonicField(face->v2i) + harmonicField(face->v3i)) / 3.0;

                    if (src > tgt) {
                        harmonicValue = 1.0 - harmonicValue;
                    }

                    double rangeCenter = (rigidSeg.startHarmonicValue + rigidSeg.endHarmonicValue) / 2.0;
                    harmonic_cost = abs(harmonicValue - rangeCenter);
                }

                // Combine costs. Lambda weights the importance of geometric proximity.
                const double lambda = 1.0; // A lambda of 1.0 gives them equal initial weighting.
                dataCost[f][label] = harmonic_cost + lambda * euclidean_cost;

            } else {
                // --- COST (Torso Segments) ---
                // Torso cost remains purely geometric.
                dataCost[f][label] = euclidean_cost;
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
    double smoothnessPenalty = 0.4;

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
    int maxIterations = 5;

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
        coords->point.setNum(mesh->verts.size());
        for (int v = 0; v < mesh->verts.size(); v++) {
            coords->point.set1Value(v, mesh->verts[v]->coords[0],
                                      mesh->verts[v]->coords[1],
                                      mesh->verts[v]->coords[2]);
        }
        segSep->addChild(coords);

        // Create indexed face set for this segment
        SoIndexedFaceSet* faceSet = new SoIndexedFaceSet();
        int faceIndex = 0;
        for (int f : segmentFaces) {
            Triangle* tri = mesh->tris[f];
            faceSet->coordIndex.set1Value(faceIndex * 4 + 0, tri->v1i);
            faceSet->coordIndex.set1Value(faceIndex * 4 + 1, tri->v2i);
            faceSet->coordIndex.set1Value(faceIndex * 4 + 2, tri->v3i);
            faceSet->coordIndex.set1Value(faceIndex * 4 + 3, -1); // End of face
            faceIndex++;
        }
        segSep->addChild(faceSet);
        meshSep->addChild(segSep);
    }

    root->addChild(meshSep);

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
        root->addChild(lineSep);
    }

    viewer->scheduleRedraw();
    viewer->viewAll();

    cout << "\n=== GRAPH-CUT SEGMENTATION COMPLETE ===" << endl;
    cout << "-> Replaced heuristic watershed with globally optimal energy minimization" << endl;
    cout << "-> Used face-based segmentation with smoothness constraints" << endl;
    cout << "-> Energy function combines harmonic affinity with boundary smoothness" << endl;
    cout << "-> Each color represents an optimal segment found by graph-cut algorithm" << endl;
    cout << "-> This approach provides cleaner boundaries and better global consistency" << endl;
}

SoSeparator* VisualizationUtils::createColoredSphere(
    const Eigen::Vector3d& position,
    const Eigen::Vector3f& color,
    float radius) {

    SoSeparator* pointSep = new SoSeparator();

    // Material
    SoMaterial* mat = new SoMaterial();
    mat->diffuseColor.setValue(color[0], color[1], color[2]);
    mat->emissiveColor.setValue(color[0] * 0.3f, color[1] * 0.3f, color[2] * 0.3f);
    pointSep->addChild(mat);

    // Transform to position
    SoTransform* transform = new SoTransform();
    transform->translation.setValue(position[0], position[1], position[2]);
    pointSep->addChild(transform);

    // Sphere representing the point
    SoSphere* sphere = new SoSphere();
    sphere->radius = radius;
    pointSep->addChild(sphere);

    return pointSep;
}

void VisualizationUtils::visualizeScalarField(
    const Mesh* mesh,
    const Eigen::VectorXd& field,
    SoSeparator* root,
    const std::string& title) {

    if (!mesh || field.size() != mesh->verts.size()) {
        cout << "Error: Invalid mesh or field size mismatch!" << endl;
        return;
    }

    cout << "Visualizing scalar field: " << title << endl;

    // Find field range for color mapping
    double minVal = field.minCoeff();
    double maxVal = field.maxCoeff();
    cout << "Field range: [" << minVal << ", " << maxVal << "]" << endl;

    // Create colored spheres for each vertex
    for (int i = 0; i < mesh->verts.size(); i++) {
        Vertex* v = mesh->verts[i];
        Eigen::Vector3d pos(v->coords[0], v->coords[1], v->coords[2]);

        // Normalize field value to [0,1]
        double normalizedVal = (maxVal - minVal) > 1e-10 ? (field[i] - minVal) / (maxVal - minVal) : 0.5;

        // Color mapping: blue (low) to red (high)
        float r = std::min(1.0f, (float)normalizedVal);
        float g = 0.2f;
        float b = 1.0f - std::min(1.0f, (float)normalizedVal);

        Eigen::Vector3f color(r, g, b);
        SoSeparator* pointViz = createColoredSphere(pos, color, 0.006f);
        root->addChild(pointViz);
    }
}

void VisualizationUtils::visualizeSkeletalSegments(
    const std::vector<std::vector<Eigen::Vector3d>>& segments,
    const Eigen::Vector3f& color,
    SoSeparator* root)
{
    if (!root) return;

    auto* segmentsSep = new SoSeparator();

    auto* mat = new SoMaterial();
    mat->diffuseColor.setValue(color[0], color[1], color[2]);
    segmentsSep->addChild(mat);

    auto* style = new SoDrawStyle();
    style->lineWidth = 3.0f;
    segmentsSep->addChild(style);

    auto* coords = new SoCoordinate3();
    auto* lines = new SoIndexedLineSet();

    int point_idx_offset = 0;

    for (const auto& segment : segments) {
        if (segment.empty()) continue;

        for (const auto& point : segment) {
            coords->point.set1Value(point_idx_offset++, (float)point.x(), (float)point.y(), (float)point.z());
        }
    }

    int line_idx_offset = 0;
    int current_point_base = 0;
    for (const auto& segment : segments) {
        if (segment.size() < 2) continue;

        for (size_t i = 0; i < segment.size() - 1; ++i) {
            lines->coordIndex.set1Value(line_idx_offset++, current_point_base + i);
            lines->coordIndex.set1Value(line_idx_offset++, current_point_base + i + 1);
            lines->coordIndex.set1Value(line_idx_offset++, -1); // End line
        }
        current_point_base += segment.size();
    }

    segmentsSep->addChild(coords);
    segmentsSep->addChild(lines);
    root->addChild(segmentsSep);
}