#include "pch.h"
#include "IsocurveAnalysis.h"
#include "helpers.h"  // For PairwiseHarmonics class
#include <iostream>
#include <queue>
#include <set>

// Coin3D headers
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoIndexedLineSet.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>

// Project headers
#include "helpers.h"

using namespace std;

std::vector<std::vector<Eigen::Vector3d>> IsocurveAnalysis::extractIsocurvesTriangleMarching(
    const Mesh* mesh,
    const Eigen::VectorXd& harmonicField,
    double isovalue) {

    vector<vector<Eigen::Vector3d>> isocurves;

    if (!mesh) return isocurves;

    // Step 1: Find all edge intersections where isocurve crosses
    struct EdgeIntersection {
        int v1, v2;  // Vertex indices of the edge
        Eigen::Vector3d point;  // 3D intersection point
        int triangle1, triangle2;  // Adjacent triangles (-1 if boundary)
    };

    vector<EdgeIntersection> intersections;

    // Check all edges for intersections
    for (int t = 0; t < mesh->tris.size(); t++) {
        Triangle* tri = mesh->tris[t];

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

                Vertex* vert1 = mesh->verts[v1];
                Vertex* vert2 = mesh->verts[v2];

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
        for (int t = 0; t < mesh->tris.size(); t++) {
            Triangle* tri = mesh->tris[t];
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

void IsocurveAnalysis::improvedIsoCurveExtraction(
    Mesh* mesh,
    PairwiseHarmonics* harmonics,
    const std::vector<int>& fps_samples,
    SoSeparator* root,
    SoWinExaminerViewer* viewer,
    std::vector<std::vector<Eigen::Vector3d>>& skeletal_segments) {

    if (!mesh || !harmonics || fps_samples.size() < 2) {
        cout << "Error: Please load mesh and compute FPS samples first!" << endl;
        return;
    }

    cout << "\n=== IMPROVED ISO-CURVE EXTRACTION ===" << endl;
    cout << "Extracting dense iso-curves as 'bracelets' around the shape" << endl;

    // Use first two FPS samples for demonstration
    int p_idx = fps_samples[0];
    int q_idx = fps_samples[1];

    cout << "Computing harmonic field between FPS vertices " << p_idx << " and " << q_idx << "..." << endl;

    Eigen::VectorXd harmonicField = harmonics->computePairwiseHarmonic(p_idx, q_idx);
    if (harmonicField.size() != mesh->verts.size()) {
        cout << "[ERROR] Failed to compute harmonic field!" << endl;
        return;
    }

    // Clear any existing visualization (assume clearVisualization is available globally)
    root->removeAllChildren();

    // Extract many more isocurves (like bracelets around a limb)
    int numIsocurves = 50; // Much denser extraction as in the paper (increased from 25)
    vector<Eigen::Vector3d> skeletal_nodes;

    cout << "Extracting " << numIsocurves << " isocurves as parallel 'bracelets'..." << endl;

    for (int k = 1; k < numIsocurves; k++) {
        double isoValue = double(k) / numIsocurves;

        // Find all vertices that are approximately at this harmonic field level
        vector<int> braceletVertices;
        double tolerance = 0.008; // Much tighter tolerance for better precision (reduced from 0.02)

        for (int v = 0; v < mesh->verts.size(); v++) {
            if (abs(harmonicField(v) - isoValue) < tolerance) {
                braceletVertices.push_back(v);
            }
        }

        if (braceletVertices.size() >= 3) {
            // Compute centroid of this "bracelet" - this becomes a skeletal node
            Eigen::Vector3d braceletCenter(0, 0, 0);
            for (int v : braceletVertices) {
                braceletCenter += Eigen::Vector3d(mesh->verts[v]->coords[0],
                                                  mesh->verts[v]->coords[1],
                                                  mesh->verts[v]->coords[2]);
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
                Vertex* v = mesh->verts[braceletVertices[i]];
                coords->point.set1Value(i, v->coords[0], v->coords[1], v->coords[2]);
            }
            braceletSep->addChild(coords);

            SoPointSet* pointSet = new SoPointSet();
            pointSet->numPoints = braceletVertices.size();
            braceletSep->addChild(pointSet);

            root->addChild(braceletSep);

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
        root->addChild(skeletonSep);

        // Store for rigidity analysis
        skeletal_segments.clear();
        skeletal_segments.push_back(skeletal_nodes);

        cout << "\nSUCCESS: Generated skeletal segment with " << skeletal_nodes.size() << " nodes" << endl;
        cout << "Yellow line shows the skeletal segment connecting bracelet centroids" << endl;
    }

    viewer->scheduleRedraw();
    viewer->viewAll();
    cout << "Improved iso-curve extraction complete - showing dense parallel bracelets!" << endl;
}

Eigen::Vector3d IsocurveAnalysis::interpolateVertex(
    const Vertex* v1,
    const Vertex* v2,
    double field1,
    double field2,
    double isovalue) {

    if (abs(field1 - field2) < 1e-10) {
        // Fields are nearly equal, return midpoint
        return 0.5 * (Eigen::Vector3d(v1->coords[0], v1->coords[1], v1->coords[2]) +
                      Eigen::Vector3d(v2->coords[0], v2->coords[1], v2->coords[2]));
    }

    double t = (isovalue - field1) / (field2 - field1);
    t = std::max(0.0, std::min(1.0, t)); // Clamp to [0,1]

    Eigen::Vector3d p1(v1->coords[0], v1->coords[1], v1->coords[2]);
    Eigen::Vector3d p2(v2->coords[0], v2->coords[1], v2->coords[2]);

    return (1.0 - t) * p1 + t * p2;
}
