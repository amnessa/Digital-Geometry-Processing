#include "pch.h"
#include "IsocurveAnalysis.h"
#include "RigidityAnalysis.h"  // For rigidity computation
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
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoTransform.h>
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

    cout << "  Extracting isocurves at value " << isovalue << " using robust triangle marching..." << endl;

    // Step 1: Generate all triangle-marched segments
    struct TriangleSegment {
        Eigen::Vector3d p1, p2;  // Endpoints of the segment
        int triangleId;          // Triangle this segment belongs to
        int edgeIds[2];          // Mesh edge IDs that this segment connects
    };

    vector<TriangleSegment> segments;

    // Process each triangle to find isocurve segments
    for (int t = 0; t < mesh->tris.size(); t++) {
        Triangle* tri = mesh->tris[t];
        int vertices[3] = {tri->v1i, tri->v2i, tri->v3i};
        double fields[3] = {harmonicField(tri->v1i), harmonicField(tri->v2i), harmonicField(tri->v3i)};

        // Find intersections on triangle edges
        vector<pair<Eigen::Vector3d, int>> intersections; // point and edge index

        for (int e = 0; e < 3; e++) {
            int v1 = vertices[e];
            int v2 = vertices[(e + 1) % 3];
            double f1 = fields[e];
            double f2 = fields[(e + 1) % 3];

            // Check if isocurve crosses this edge
            bool crosses = (f1 < isovalue && f2 > isovalue) || (f1 > isovalue && f2 < isovalue);
            if (crosses && abs(f1 - f2) > 1e-10) {
                // Linear interpolation to find intersection point
                double t = (isovalue - f1) / (f2 - f1);

                Vertex* vert1 = mesh->verts[v1];
                Vertex* vert2 = mesh->verts[v2];

                Eigen::Vector3d p1(vert1->coords[0], vert1->coords[1], vert1->coords[2]);
                Eigen::Vector3d p2(vert2->coords[0], vert2->coords[1], vert2->coords[2]);

                Eigen::Vector3d intersection = (1.0 - t) * p1 + t * p2;

                // Create unique edge ID (smaller vertex index first)
                int edgeId = (v1 < v2) ? (v1 * mesh->verts.size() + v2) : (v2 * mesh->verts.size() + v1);

                intersections.push_back(make_pair(intersection, edgeId));
            }
        }

        // If triangle has exactly 2 intersections, create a segment
        if (intersections.size() == 2) {
            TriangleSegment seg;
            seg.p1 = intersections[0].first;
            seg.p2 = intersections[1].first;
            seg.triangleId = t;
            seg.edgeIds[0] = intersections[0].second;
            seg.edgeIds[1] = intersections[1].second;
            segments.push_back(seg);
        }
    }

    cout << "    Generated " << segments.size() << " triangle segments" << endl;

    if (segments.empty()) {
        return isocurves;
    }

    // Step 2: Unify endpoints and build connectivity graph
    // Use spatial hashing to efficiently find nearby points
    const double eps = 1e-8; // Tolerance for point equality

    struct GraphNode {
        Eigen::Vector3d point;
        vector<int> neighbors; // Indices of connected nodes
        bool visited;

        GraphNode(const Eigen::Vector3d& p) : point(p), visited(false) {}
    };

    vector<GraphNode> nodes;

    // Build node list with unified endpoints
    for (int i = 0; i < segments.size(); i++) {
        const TriangleSegment& seg = segments[i];

        // Find or create node for p1
        int node1 = -1;
        for (int j = 0; j < nodes.size(); j++) {
            if ((nodes[j].point - seg.p1).norm() < eps) {
                node1 = j;
                break;
            }
        }
        if (node1 == -1) {
            nodes.push_back(GraphNode(seg.p1));
            node1 = nodes.size() - 1;
        }

        // Find or create node for p2
        int node2 = -1;
        for (int j = 0; j < nodes.size(); j++) {
            if ((nodes[j].point - seg.p2).norm() < eps) {
                node2 = j;
                break;
            }
        }
        if (node2 == -1) {
            nodes.push_back(GraphNode(seg.p2));
            node2 = nodes.size() - 1;
        }

        // Add bidirectional connectivity
        nodes[node1].neighbors.push_back(node2);
        nodes[node2].neighbors.push_back(node1);
    }

    cout << "    Unified to " << nodes.size() << " unique endpoints" << endl;

    // Step 3: Traverse the graph to extract continuous polylines
    for (int startNode = 0; startNode < nodes.size(); startNode++) {
        if (nodes[startNode].visited) continue;

        // Start a new polyline from this node
        vector<Eigen::Vector3d> polyline;

        // Find an unvisited endpoint (degree 1) or any unvisited node
        int current = startNode;
        bool foundEndpoint = false;

        // Try to start from an endpoint (degree 1 node)
        for (int i = 0; i < nodes.size(); i++) {
            if (!nodes[i].visited && nodes[i].neighbors.size() == 1) {
                current = i;
                foundEndpoint = true;
                break;
            }
        }

        if (!foundEndpoint && nodes[current].visited) {
            continue; // All nodes in this component are visited
        }

        // Traverse the polyline
        int previous = -1;
        while (current != -1 && !nodes[current].visited) {
            nodes[current].visited = true;
            polyline.push_back(nodes[current].point);

            // Find next unvisited neighbor
            int next = -1;
            for (int neighbor : nodes[current].neighbors) {
                if (neighbor != previous && !nodes[neighbor].visited) {
                    next = neighbor;
                    break;
                }
            }

            previous = current;
            current = next;
        }

        // Only keep polylines with multiple points
        if (polyline.size() >= 2) {
            isocurves.push_back(polyline);
        }
    }

    cout << "    Extracted " << isocurves.size() << " continuous isocurves" << endl;

    // Print some statistics
    for (int i = 0; i < isocurves.size(); i++) {
        cout << "      Isocurve " << i << ": " << isocurves[i].size() << " points" << endl;
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
    cout << "Extracting robust isocurves using triangle marching and graph stitching" << endl;

    // Use first two FPS samples for demonstration
    int p_idx = fps_samples[0];
    int q_idx = fps_samples[1];

    cout << "Computing harmonic field between FPS vertices " << p_idx << " and " << q_idx << "..." << endl;    Eigen::VectorXd harmonicField = harmonics->computePairwiseHarmonic(p_idx, q_idx);
    if (harmonicField.size() != mesh->verts.size()) {
        cout << "[ERROR] Failed to compute harmonic field!" << endl;
        return;
    }

    // Remove previous isocurve and skeletal visualizations (but keep the mesh)
    // We'll look for nodes with specific names or materials to remove selectively
    // For now, we'll add our new visualization without clearing everything

    // Extract robust isocurves using triangle marching
    int numIsocurves = 25; // Number of isocurve levels to extract
    vector<Eigen::Vector3d> skeletal_nodes;
    vector<vector<Eigen::Vector3d>> all_isocurves;

    cout << "Extracting " << numIsocurves << " isocurves using robust triangle marching..." << endl;

    for (int k = 1; k < numIsocurves; k++) {
        double isoValue = double(k) / numIsocurves;

        // Extract isocurves at this level using robust triangle marching
        vector<vector<Eigen::Vector3d>> levelIsocurves = extractIsocurvesTriangleMarching(
            mesh, harmonicField, isoValue);

        // Process each isocurve at this level
        for (const auto& isocurve : levelIsocurves) {
            if (isocurve.size() >= 3) {
                all_isocurves.push_back(isocurve);

                // Compute centroid of this isocurve for skeletal node
                Eigen::Vector3d isocurveCenter(0, 0, 0);
                for (const auto& point : isocurve) {
                    isocurveCenter += point;
                }
                isocurveCenter /= isocurve.size();
                skeletal_nodes.push_back(isocurveCenter);

                // Visualize this isocurve
                SoSeparator* isocurveSep = new SoSeparator();
                SoMaterial* mat = new SoMaterial();

                // Color based on position along harmonic field (blue to red gradient)
                float r = isoValue;
                float g = 0.2f;
                float b = 1.0f - isoValue;
                mat->diffuseColor.setValue(r, g, b);
                isocurveSep->addChild(mat);

                // Draw the isocurve as a polyline
                SoCoordinate3* coords = new SoCoordinate3();
                coords->point.setNum(isocurve.size());

                for (int i = 0; i < isocurve.size(); i++) {
                    coords->point.set1Value(i, isocurve[i][0], isocurve[i][1], isocurve[i][2]);
                }
                isocurveSep->addChild(coords);

                // Create line set to visualize the polyline
                SoIndexedLineSet* lineSet = new SoIndexedLineSet();
                for (int i = 0; i < isocurve.size(); i++) {
                    lineSet->coordIndex.set1Value(i, i);
                }
                lineSet->coordIndex.set1Value(isocurve.size(), -1); // End marker
                isocurveSep->addChild(lineSet);

                root->addChild(isocurveSep);

                cout << "  Isocurve " << k << " (value=" << isoValue << "): "
                     << isocurve.size() << " points in continuous polyline" << endl;
            }
        }
    }    // Visualize the skeletal segment by connecting centroids
    if (skeletal_nodes.size() >= 3) {
        // Compute rigidity scores for each skeletal node (isocurve centroid)
        cout << "\nComputing rigidity scores for isocurve centroids..." << endl;
        vector<double> rigidity_scores;

        for (int i = 0; i < skeletal_nodes.size(); i++) {
            double rigidity = RigidityAnalysis::computeNodeRigidity(skeletal_nodes, i, 7);
            rigidity_scores.push_back(rigidity);
            cout << "  Centroid " << i << ": rigidity = " << rigidity << endl;
        }

        // Create skeletal line visualization
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

        // Visualize rigidity points with color coding (green=rigid, red=non-rigid)
        cout << "Visualizing rigidity points on isocurve centroids..." << endl;

        // Normalize rigidity scores for color mapping
        double minRig = *std::min_element(rigidity_scores.begin(), rigidity_scores.end());
        double maxRig = *std::max_element(rigidity_scores.begin(), rigidity_scores.end());
        double rigRange = maxRig - minRig;
        if (rigRange < 1e-10) rigRange = 1.0; // Avoid division by zero

        for (int i = 0; i < skeletal_nodes.size(); i++) {
            SoSeparator* pointSep = new SoSeparator();

            // Position the sphere at the skeletal node
            SoTransform* transform = new SoTransform();
            transform->translation.setValue(
                skeletal_nodes[i][0],
                skeletal_nodes[i][1],
                skeletal_nodes[i][2]
            );
            pointSep->addChild(transform);

            // Color based on rigidity score (green = rigid, red = non-rigid)
            SoMaterial* mat = new SoMaterial();
            double normalizedRig = (rigidity_scores[i] - minRig) / rigRange;

            // Green to red gradient: rigid (high score) = green, non-rigid (low score) = red
            float r = 1.0f - normalizedRig;  // More red for lower rigidity
            float g = normalizedRig;         // More green for higher rigidity
            float b = 0.1f;                  // Minimal blue

            mat->diffuseColor.setValue(r, g, b);
            pointSep->addChild(mat);

            // Create sphere to mark the rigidity point
            SoSphere* sphere = new SoSphere();
            sphere->radius = 0.02f; // Adaptive sizing based on mesh bounds
            pointSep->addChild(sphere);

            root->addChild(pointSep);
        }

        // Store for rigidity analysis
        skeletal_segments.clear();
        skeletal_segments.push_back(skeletal_nodes);

        cout << "\nSUCCESS: Generated skeletal segment with " << skeletal_nodes.size() << " nodes" << endl;
        cout << "Yellow line shows the skeletal segment connecting isocurve centroids" << endl;
        cout << "Colored spheres show rigidity points (green=rigid, red=non-rigid)" << endl;
        cout << "Rigidity range: [" << minRig << ", " << maxRig << "]" << endl;
        cout << "Total " << all_isocurves.size() << " robust isocurves extracted" << endl;
    }

    viewer->scheduleRedraw();
    viewer->viewAll();
    cout << "Robust iso-curve extraction complete - showing continuous polylines!" << endl;
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
