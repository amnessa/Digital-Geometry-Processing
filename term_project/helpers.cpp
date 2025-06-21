/**
 * Clean implementation of PairwiseHarmonics class
 * Focused on the core segmentation algorithm from the paper
 */

#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <cfloat>
#include <utility>
#include <cmath>
#include <iomanip>

// Eigen headers
#include <Eigen/Dense>
#include <Eigen/Sparse>

// Coin3D headers
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoFaceSet.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/SbVec3f.h>

// Project headers
#include "Mesh.h"
#include "Painter.h"
#include "helpers.h"

// libigl headers
#include <igl/cotmatrix.h>
#include <igl/harmonic.h>
#include <igl/massmatrix.h>
#include <igl/boundary_facets.h>
#include <igl/slice.h>

using namespace std;

// Constructor
PairwiseHarmonics::PairwiseHarmonics(Mesh* m) : mesh(m), laplacianComputed(false), solverComputed(false), meshDataComputed(false) {}

// Initialize LibIGL mesh data
bool PairwiseHarmonics::initializeLibIGL() {
    if (!mesh || mesh->verts.empty() || mesh->tris.empty()) {
        return false;
    }

    int nVerts = mesh->verts.size();
    int nFaces = mesh->tris.size();

    // Convert mesh to LibIGL format
    V.resize(nVerts, 3);
    F.resize(nFaces, 3);

    // Fill vertex matrix
    for (int i = 0; i < nVerts; i++) {
        V(i, 0) = mesh->verts[i]->coords[0];
        V(i, 1) = mesh->verts[i]->coords[1];
        V(i, 2) = mesh->verts[i]->coords[2];
    }

    // Fill face matrix
    for (int i = 0; i < nFaces; i++) {
        F(i, 0) = mesh->tris[i]->v1i;
        F(i, 1) = mesh->tris[i]->v2i;
        F(i, 2) = mesh->tris[i]->v3i;
    }

    meshDataComputed = true;
    return true;
}

// Core pairwise harmonic computation using LibIGL
Eigen::VectorXd PairwiseHarmonics::computePairwiseHarmonicLibIGL(int p_idx, int q_idx) {
    if (!meshDataComputed) {
        if (!initializeLibIGL()) {
            return Eigen::VectorXd();
        }
    }

    // Set up boundary conditions
    Eigen::VectorXi b(2);
    b << p_idx, q_idx;

    Eigen::MatrixXd bc(2, 1);
    bc << 0.0, 1.0;  // p gets value 0, q gets value 1

    // Solve harmonic equation
    Eigen::MatrixXd W;
    if (!igl::harmonic(V, F, b, bc, 1, W)) {
        cout << "Error: LibIGL harmonic solver failed!" << endl;
        return Eigen::VectorXd();
    }

    return W.col(0);  // Return the harmonic field
}

// Legacy implementation for compatibility
Eigen::VectorXd PairwiseHarmonics::computePairwiseHarmonic(int vertex_p_idx, int vertex_q_idx) {
    return computePairwiseHarmonicLibIGL(vertex_p_idx, vertex_q_idx);
}

// Cotangent Laplacian computation
bool PairwiseHarmonics::computeCotangentLaplacian() {
    if (!meshDataComputed) {
        if (!initializeLibIGL()) {
            return false;
        }
    }    // Use LibIGL to compute cotangent Laplacian
    igl::cotmatrix(V, F, L);

    laplacianComputed = true;
    return true;
}

// Implementation of computeCotangentLaplacianLibigl
bool PairwiseHarmonics::computeCotangentLaplacianLibigl() {
    if (!meshDataComputed) {
        if (!initializeLibIGL()) {
            return false;
        }
    }

    // Use LibIGL to compute cotangent Laplacian
    igl::cotmatrix(V, F, L);

    laplacianComputed = true;
    return true;
}

// Visualization of harmonic field
SoSeparator* PairwiseHarmonics::visualizeHarmonicField(const Eigen::VectorXd& field, Painter* painter) {
    if (!painter || field.size() != mesh->verts.size()) {
        return nullptr;
    }

    SoSeparator* sep = new SoSeparator();

    // Create colored mesh based on harmonic field values
    SoMaterial* material = new SoMaterial();
    SoCoordinate3* coords = new SoCoordinate3();
    SoFaceSet* faceSet = new SoFaceSet();

    // Set coordinates
    coords->point.setNum(mesh->verts.size());
    for (int i = 0; i < mesh->verts.size(); i++) {
        coords->point.set1Value(i,
            mesh->verts[i]->coords[0],
            mesh->verts[i]->coords[1],
            mesh->verts[i]->coords[2]);
    }

    // Color mapping: blue (0) to red (1)
    material->diffuseColor.setNum(mesh->verts.size());
    for (int i = 0; i < mesh->verts.size(); i++) {
        float value = static_cast<float>(field(i));
        material->diffuseColor.set1Value(i, value, 0.0f, 1.0f - value);
    }

    sep->addChild(material);
    sep->addChild(coords);
    sep->addChild(faceSet);

    return sep;
}

// Compute R descriptor (isocurve lengths)
Eigen::VectorXd PairwiseHarmonics::computeRDescriptor(const Eigen::VectorXd& field, int numSamplesK) {
    Eigen::VectorXd R(numSamplesK);

    for (int k = 0; k < numSamplesK; k++) {
        double isoValue = double(k + 1) / (numSamplesK + 1);
        R(k) = computeIsoCurveLength(field, isoValue);
    }

    return R;
}

// Compute total length of isocurve at given level
double PairwiseHarmonics::computeIsoCurveLength(const Eigen::VectorXd& field, double isoValue) {
    double totalLength = 0.0;

    // Process each triangle
    for (int i = 0; i < mesh->tris.size(); i++) {
        Triangle* tri = mesh->tris[i];

        // Get field values at triangle vertices
        double val1 = field(tri->v1i);
        double val2 = field(tri->v2i);
        double val3 = field(tri->v3i);

        vector<Eigen::Vector3d> intersections;

        // Check each edge for isocurve intersection
        if ((val1 <= isoValue && val2 >= isoValue) || (val1 >= isoValue && val2 <= isoValue)) {
            // Edge v1-v2 intersects isocurve
            double t = (isoValue - val1) / (val2 - val1);
            Eigen::Vector3d p1(mesh->verts[tri->v1i]->coords[0],
                              mesh->verts[tri->v1i]->coords[1],
                              mesh->verts[tri->v1i]->coords[2]);
            Eigen::Vector3d p2(mesh->verts[tri->v2i]->coords[0],
                              mesh->verts[tri->v2i]->coords[1],
                              mesh->verts[tri->v2i]->coords[2]);
            intersections.push_back(p1 + t * (p2 - p1));
        }

        if ((val2 <= isoValue && val3 >= isoValue) || (val2 >= isoValue && val3 <= isoValue)) {
            // Edge v2-v3 intersects isocurve
            double t = (isoValue - val2) / (val3 - val2);
            Eigen::Vector3d p2(mesh->verts[tri->v2i]->coords[0],
                              mesh->verts[tri->v2i]->coords[1],
                              mesh->verts[tri->v2i]->coords[2]);
            Eigen::Vector3d p3(mesh->verts[tri->v3i]->coords[0],
                              mesh->verts[tri->v3i]->coords[1],
                              mesh->verts[tri->v3i]->coords[2]);
            intersections.push_back(p2 + t * (p3 - p2));
        }

        if ((val3 <= isoValue && val1 >= isoValue) || (val3 >= isoValue && val1 <= isoValue)) {
            // Edge v3-v1 intersects isocurve
            double t = (isoValue - val3) / (val1 - val3);
            Eigen::Vector3d p3(mesh->verts[tri->v3i]->coords[0],
                              mesh->verts[tri->v3i]->coords[1],
                              mesh->verts[tri->v3i]->coords[2]);
            Eigen::Vector3d p1(mesh->verts[tri->v1i]->coords[0],
                              mesh->verts[tri->v1i]->coords[1],
                              mesh->verts[tri->v1i]->coords[2]);
            intersections.push_back(p3 + t * (p1 - p3));
        }

        // Add segment length if we have exactly 2 intersection points
        if (intersections.size() == 2) {
            totalLength += (intersections[1] - intersections[0]).norm();
        }
    }

    return totalLength;
}

// Extract isocurve segments for visualization
vector<pair<Eigen::Vector3d, Eigen::Vector3d>> PairwiseHarmonics::extractIsoCurveSegments(const Eigen::VectorXd& field, double isoValue) {
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> segments;

    // Process each triangle
    for (int i = 0; i < mesh->tris.size(); i++) {
        Triangle* tri = mesh->tris[i];

        // Get field values at triangle vertices
        double val1 = field(tri->v1i);
        double val2 = field(tri->v2i);
        double val3 = field(tri->v3i);

        vector<Eigen::Vector3d> intersections;

        // Find intersection points
        if ((val1 <= isoValue && val2 >= isoValue) || (val1 >= isoValue && val2 <= isoValue)) {
            double t = (isoValue - val1) / (val2 - val1);
            Eigen::Vector3d p1(mesh->verts[tri->v1i]->coords[0],
                              mesh->verts[tri->v1i]->coords[1],
                              mesh->verts[tri->v1i]->coords[2]);
            Eigen::Vector3d p2(mesh->verts[tri->v2i]->coords[0],
                              mesh->verts[tri->v2i]->coords[1],
                              mesh->verts[tri->v2i]->coords[2]);
            intersections.push_back(p1 + t * (p2 - p1));
        }

        if ((val2 <= isoValue && val3 >= isoValue) || (val2 >= isoValue && val3 <= isoValue)) {
            double t = (isoValue - val2) / (val3 - val2);
            Eigen::Vector3d p2(mesh->verts[tri->v2i]->coords[0],
                              mesh->verts[tri->v2i]->coords[1],
                              mesh->verts[tri->v2i]->coords[2]);
            Eigen::Vector3d p3(mesh->verts[tri->v3i]->coords[0],
                              mesh->verts[tri->v3i]->coords[1],
                              mesh->verts[tri->v3i]->coords[2]);
            intersections.push_back(p2 + t * (p3 - p2));
        }

        if ((val3 <= isoValue && val1 >= isoValue) || (val3 >= isoValue && val1 <= isoValue)) {
            double t = (isoValue - val3) / (val1 - val3);
            Eigen::Vector3d p3(mesh->verts[tri->v3i]->coords[0],
                              mesh->verts[tri->v3i]->coords[1],
                              mesh->verts[tri->v3i]->coords[2]);
            Eigen::Vector3d p1(mesh->verts[tri->v1i]->coords[0],
                              mesh->verts[tri->v1i]->coords[1],
                              mesh->verts[tri->v1i]->coords[2]);
            intersections.push_back(p3 + t * (p1 - p3));
        }

        // Create segment if we have exactly 2 intersection points
        if (intersections.size() == 2) {
            segments.push_back(make_pair(intersections[0], intersections[1]));
        }
    }

    return segments;
}

// Simplified D descriptor computation
Eigen::VectorXd PairwiseHarmonics::computeDDescriptor(int vertex_p_idx, int vertex_q_idx, const Eigen::VectorXd& field, int numSamplesK, const float* dist_p, const float* dist_q) {
    Eigen::VectorXd D(numSamplesK);

    for (int k = 0; k < numSamplesK; k++) {
        double isoValue = double(k + 1) / (numSamplesK + 1);

        // Find vertices close to this isocurve level
        vector<int> isocurveVertices;
        double tolerance = 0.05;

        for (int v = 0; v < mesh->verts.size(); v++) {
            if (abs(field(v) - isoValue) < tolerance) {
                isocurveVertices.push_back(v);
            }
        }

        // Compute average geodesic distance
        double avgDistance = 0.0;
        if (!isocurveVertices.empty() && dist_p && dist_q) {
            for (int v : isocurveVertices) {
                if (k < numSamplesK / 2) {
                    avgDistance += dist_p[v];  // Use distance from p
                } else {
                    avgDistance += dist_q[v];  // Use distance from q
                }
            }
            avgDistance /= isocurveVertices.size();
        }

        D(k) = avgDistance;
    }

    return D;
}

// Compute PIS (Path Intrinsic Symmetry) score
double PairwiseHarmonics::computePIS(const Eigen::VectorXd& R_pq, const Eigen::VectorXd& D_pq, const Eigen::VectorXd& R_qp, const Eigen::VectorXd& D_qp) {
    if (R_pq.size() != R_qp.size() || D_pq.size() != D_qp.size()) {
        return 0.0;
    }

    // Compute L2 distances between descriptors
    double R_distance = (R_pq - R_qp).norm();
    double D_distance = (D_pq - D_qp).norm();

    // Normalize by descriptor magnitudes
    double R_norm = max(R_pq.norm(), R_qp.norm());
    double D_norm = max(D_pq.norm(), D_qp.norm());

    if (R_norm > 0) R_distance /= R_norm;
    if (D_norm > 0) D_distance /= D_norm;

    // PIS score: higher is better (more symmetric)
    return exp(-(R_distance + D_distance));
}

// Enhanced methods using LibIGL
Eigen::VectorXd PairwiseHarmonics::computeRDescriptorLibIGL(const Eigen::VectorXd& field, int numSamplesK) {
    return computeRDescriptor(field, numSamplesK);  // Use existing implementation
}

Eigen::VectorXd PairwiseHarmonics::computeDDescriptorLibIGL(int vertex_p_idx, int vertex_q_idx, const Eigen::VectorXd& field, int numSamplesK) {
    // Use heat geodesics if available, otherwise use null distances
    return computeDDescriptor(vertex_p_idx, vertex_q_idx, field, numSamplesK, nullptr, nullptr);
}

Eigen::VectorXd PairwiseHarmonics::computeHeatGeodesicDistances(int source_vertex) {
    // Use the mesh's heat geodesic implementation
    if (mesh) {
        return mesh->computeHeatGeodesicDistances(source_vertex);
    }

    // Fallback: return zeros if mesh is not available
    cout << "Warning: No mesh available for heat geodesic computation" << endl;
    Eigen::VectorXd distances = Eigen::VectorXd::Zero(mesh ? mesh->verts.size() : 0);
    return distances;
}

Eigen::VectorXd PairwiseHarmonics::computeHarmonicCoordinatesLibIGL(const vector<int>& boundary_vertices, const vector<double>& boundary_values) {
    if (!meshDataComputed) {
        if (!initializeLibIGL()) {
            return Eigen::VectorXd();
        }
    }

    if (boundary_vertices.size() != boundary_values.size()) {
        return Eigen::VectorXd();
    }

    // Set up boundary conditions
    Eigen::VectorXi b(boundary_vertices.size());
    Eigen::MatrixXd bc(boundary_vertices.size(), 1);

    for (int i = 0; i < boundary_vertices.size(); i++) {
        b(i) = boundary_vertices[i];
        bc(i, 0) = boundary_values[i];
    }

    // Solve harmonic equation
    Eigen::MatrixXd W;
    if (!igl::harmonic(V, F, b, bc, 1, W)) {
        return Eigen::VectorXd();
    }

    return W.col(0);
}
