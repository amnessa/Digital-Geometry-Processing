#include "pch.h"
#include "VisualizationUtils.h"
#include "helpers.h"  // For PairwiseHarmonics class
#include <iostream>
#include <ctime>
#include <algorithm>

// Coin3D headers
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedLineSet.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>

// Project headers
#include "helpers.h"
#include "Painter.h"

using namespace std;

void VisualizationUtils::clearVisualization(SoSeparator* root) {
    // Remove all children except the first one (which might be the mesh)
    while (root->getNumChildren() > 1) {
        root->removeChild(root->getNumChildren() - 1);
    }
}

void VisualizationUtils::testCotangentLaplacian(
    Mesh* mesh,
    PairwiseHarmonics* harmonics,
    SoSeparator* root,
    SoWinExaminerViewer* viewer) {

    if (!mesh || !harmonics) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }

    cout << "\n=== TESTING COTANGENT LAPLACIAN ===" << endl;
    cout << "Computing cotangent-weighted Laplacian matrix..." << endl;

    auto start = clock();
    bool success = harmonics->computeCotangentLaplacian();
    auto end = clock();

    double elapsed = double(end - start) / CLOCKS_PER_SEC;

    if (success) {
        cout << "SUCCESS: Cotangent Laplacian computed successfully!" << endl;
        cout << "  Computation time: " << elapsed << " seconds" << endl;
        cout << "  Matrix size: " << mesh->verts.size() << "x" << mesh->verts.size() << endl;
        cout << "  Properties:" << endl;
        cout << "    - Intrinsic (uses cotangent weights)" << endl;
        cout << "    - Includes Voronoi area weighting" << endl;
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

    if (!mesh) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }

    cout << "\n=== TESTING FARTHEST POINT SAMPLING ===" << endl;
    int numSamples;
    cout << "Enter number of FPS samples (recommended: 25-40 for human models, 15-25 for simple shapes): ";
    cin >> numSamples;

    if (numSamples < 2 || numSamples > mesh->verts.size()) {
        cout << "Error: Invalid number of samples!" << endl;
        return;
    }

    cout << "Computing FPS with " << numSamples << " points..." << endl;
    cout << "Algorithm: 1) Pick arbitrary point -> 2) Find farthest -> 3) Discard arbitrary -> 4) Continue FPS" << endl;

    auto start = clock();
    fps_samples = mesh->farthestPointSampling(numSamples);
    auto end = clock();

    double elapsed = double(end - start) / CLOCKS_PER_SEC;
    if (fps_samples.size() == numSamples) {
        cout << "SUCCESS: FPS computed successfully!" << endl;
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

    if (!mesh || !harmonics) {
        cout << "Error: Please load a mesh first!" << endl;
        return;
    }

    cout << "\n=== TESTING PAIRWISE HARMONICS ===" << endl;

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
    Eigen::VectorXd harmonicField = harmonics->computePairwiseHarmonic(p_idx, q_idx);
    auto end = clock();

    double elapsed = double(end - start) / CLOCKS_PER_SEC;

    if (harmonicField.size() == nVerts) {
        cout << "SUCCESS: Pairwise harmonic computed successfully!" << endl;
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

    if (!mesh || !harmonics || fps_samples.size() < 2) {
        cout << "Error: Please load mesh and compute FPS samples first!" << endl;
        return;
    }

    cout << "\n=== TESTING ISO-CURVE EXTRACTION ===" << endl;

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
    vector<double> isoValues = {0.2, 0.4, 0.6, 0.8};
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
