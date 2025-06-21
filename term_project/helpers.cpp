#include <stdio.h>
#include <iostream>
#include <vector>
#include <string> // Add this
#include <algorithm>
#include <cfloat>
#include <utility>
#include <cmath>    // Add this for std::round, std::max
#include <iomanip>  // Add this for std::setw, std::fixed, std::setprecision

// Standard C++ headers and Eigen headers BEFORE Windows.h and Coin3D headers
#include <Eigen/Dense>
#include <Eigen/Sparse>

// Windows headers with macro protection
#define NOMINMAX
#include <windows.h>

// Coin3D/SoWin headers
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

// libigl headers - include after project headers to avoid conflicts
#include <igl/cotmatrix.h>
#include <igl/harmonic.h>
#include <igl/massmatrix.h>
#include <igl/boundary_facets.h>
#include <igl/slice.h>

using namespace std;

// PairwiseHarmonics private methods implementation

double PairwiseHarmonics::calculateCotangent(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    double dot = a.dot(b);
    Eigen::Vector3d cross = a.cross(b);
    double crossNorm = cross.norm();

    // Check for near-zero cross product (parallel vectors)
    if (crossNorm < 1e-10) {
        // Handle the case of parallel vectors
        // If dot product is positive, vectors are nearly parallel
        // If dot product is negative, vectors are nearly anti-parallel
        // In either case, return a large value with appropriate sign
        return (dot >= 0) ? 1e10 : -1e10;
    }

    return dot / crossNorm;
}

double PairwiseHarmonics::calculateAngle(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    double normA = a.norm();
    double normB = b.norm();
    // Handle potential division by zero if vectors have zero length
    if (normA < 1e-10 || normB < 1e-10) {
        return 0.0; // Or handle as error
    }
    double cosAngle = a.dot(b) / (normA * normB);
    // Ensure the value is within [-1, 1] to avoid numerical errors
    // Always use std:: namespace explicitly for min/max
    cosAngle = std::max(-1.0, std::min(1.0, cosAngle));
    return std::acos(cosAngle);
}

double PairwiseHarmonics::computeTriangleArea(const Eigen::Vector3d& v1,
                           const Eigen::Vector3d& v2,
                           const Eigen::Vector3d& v3) {
    Eigen::Vector3d edge1 = v2 - v1;
    Eigen::Vector3d edge2 = v3 - v1;
    return 0.5 * (edge1.cross(edge2)).norm();
}

double PairwiseHarmonics::computeVoronoiArea(int vertexIdx) {
    double area = 0.0;
    const SbVec3f& p_coords = mesh->verts[vertexIdx]->coords;
    Eigen::Vector3d p(p_coords[0], p_coords[1], p_coords[2]); // Initialize with components

    // Iterate through all triangles containing this vertex
    for (int triIdx : mesh->verts[vertexIdx]->triList) {
        Triangle* tri = mesh->tris[triIdx];

        // Get the indices of the three vertices
        int v1i = tri->v1i;
        int v2i = tri->v2i;
        int v3i = tri->v3i;

        // Ensure we're working with the correct vertex as v1
        if (vertexIdx != v1i && vertexIdx != v2i && vertexIdx != v3i) {
            continue; // Skip if this vertex is not part of this triangle
        }

        // Rearrange vertices so that vertexIdx is v1i
        if (vertexIdx == v2i) {
            std::swap(v1i, v2i);
        } else if (vertexIdx == v3i) {
            std::swap(v1i, v3i);
        }

        const SbVec3f& v1_coords = mesh->verts[v1i]->coords;
        const SbVec3f& v2_coords = mesh->verts[v2i]->coords;
        const SbVec3f& v3_coords = mesh->verts[v3i]->coords;
        Eigen::Vector3d v1(v1_coords[0], v1_coords[1], v1_coords[2]);
        Eigen::Vector3d v2(v2_coords[0], v2_coords[1], v2_coords[2]);
        Eigen::Vector3d v3(v3_coords[0], v3_coords[1], v3_coords[2]);


        // Compute edges
        Eigen::Vector3d e1 = v2 - v1; // edge from v1 to v2
        Eigen::Vector3d e2 = v3 - v1; // edge from v1 to v3

        // Compute angles
        double angle = calculateAngle(e1, e2);
        double angle_v2 = calculateAngle(v3-v2, v1-v2); // Angle at v2
        double angle_v3 = calculateAngle(v1-v3, v2-v3); // Angle at v3

        // Compute triangle area
        double triangleArea = computeTriangleArea(v1, v2, v3);

        // For obtuse triangles, use special case handling
        if (angle > M_PI / 2.0 || angle_v2 > M_PI / 2.0 || angle_v3 > M_PI / 2.0) {
            // Obtuse triangle - use fractional area
            if (angle > M_PI/2) {
                // Angle at vertexIdx is obtuse
                area += triangleArea / 2;
            } else {
                // Angle at another vertex is obtuse
                area += triangleArea / 4;
            }
        } else {
            // Non-obtuse triangle - use Voronoi area
            // Compute cotangents of the angles opposite to the edges
            double cotAlpha = calculateCotangent(v3-v2, v1-v2); // cot of angle at v2
            double cotBeta = calculateCotangent(v1-v3, v2-v3);  // cot of angle at v3

            // Compute squared edge lengths
            double e1SquaredLength = e1.squaredNorm();
            double e2SquaredLength = e2.squaredNorm();

            // Compute the Voronoi area contribution
            area += (cotAlpha * e1SquaredLength + cotBeta * e2SquaredLength) / 8;
        }
    }

    return area;
}

// PairwiseHarmonics public methods implementation

PairwiseHarmonics::PairwiseHarmonics(Mesh* m) : mesh(m), laplacianComputed(false), solverComputed(false), meshDataComputed(false) {}

bool PairwiseHarmonics::computeCotangentLaplacian() {
    int nVerts = mesh->verts.size();
    L.resize(nVerts, nVerts);
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(nVerts * 7);
    std::vector<double> diagonalValues(nVerts, 0.0);

    // First, compute Voronoi areas for all vertices (mass matrix)
    std::vector<double> voronoiAreas(nVerts);
    for (int i = 0; i < nVerts; i++) {
        voronoiAreas[i] = computeVoronoiArea(i);
        // Ensure minimum area to avoid numerical issues
        if (voronoiAreas[i] < 1e-10) {
            voronoiAreas[i] = 1e-10;
        }
    }

    // Process each edge in the mesh
    for (Edge* edge : mesh->edges) {
        int i = edge->v1i;
        int j = edge->v2i;
        if (i == j) continue;

        // Find triangles sharing this edge
        std::vector<int> sharedTriangles;
        for (int triIdxI : mesh->verts[i]->triList) {
            for (int triIdxJ : mesh->verts[j]->triList) {
                if (triIdxI == triIdxJ) {
                    sharedTriangles.push_back(triIdxI);
                    break;
                }
            }
        }

        double cotWeight = 0.0;
        for (int triIdx : sharedTriangles) {
            Triangle* tri = mesh->tris[triIdx];

            // Find the third vertex in this triangle
            int k;
            if ((tri->v1i == i && tri->v2i == j) || (tri->v1i == j && tri->v2i == i)) {
                k = tri->v3i;
            } else if ((tri->v2i == i && tri->v3i == j) || (tri->v2i == j && tri->v3i == i)) {
                k = tri->v1i;
            } else {
                k = tri->v2i;
            }

            // Get coordinates of vertices
            const SbVec3f& vi_coords = mesh->verts[i]->coords;
            const SbVec3f& vj_coords = mesh->verts[j]->coords;
            const SbVec3f& vk_coords = mesh->verts[k]->coords;
            Eigen::Vector3d vi(vi_coords[0], vi_coords[1], vi_coords[2]);
            Eigen::Vector3d vj(vj_coords[0], vj_coords[1], vj_coords[2]);
            Eigen::Vector3d vk(vk_coords[0], vk_coords[1], vk_coords[2]);

            // Compute vectors
            Eigen::Vector3d vecA = vi - vk;
            Eigen::Vector3d vecB = vj - vk;

            // Check that vectors aren't too small (degenerate triangle)
            if (vecA.norm() < 1e-10 || vecB.norm() < 1e-10) {
                continue; // Skip degenerate triangles
            }

            // Compute cotangent of the angle at vertex k
            double cotTheta = calculateCotangent(vecA, vecB);

            // Clamp cotangent values to avoid numerical issues
            if (std::abs(cotTheta) > 1e6) {
                cotTheta = (cotTheta > 0) ? 1e6 : -1e6;
            }

            // Add to total weight (multiply by 0.5 as per standard cotangent formula)
            cotWeight += cotTheta * 0.5;
        }

        if (std::abs(cotWeight) > 1e-10) {
            if (std::abs(cotWeight) > 1e6) {
                cotWeight = (cotWeight > 0) ? 1e6 : -1e6;
            }

            // Apply mass matrix weighting: Δd = M^(-1)L
            // For off-diagonal elements: L_ij / M_i
            double weightedCotWeight_i = cotWeight / voronoiAreas[i];
            double weightedCotWeight_j = cotWeight / voronoiAreas[j];

            triplets.push_back(Eigen::Triplet<double>(i, j, -weightedCotWeight_i));
            triplets.push_back(Eigen::Triplet<double>(j, i, -weightedCotWeight_j));

            // Update diagonal values
            diagonalValues[i] += weightedCotWeight_i;
            diagonalValues[j] += weightedCotWeight_j;
        }
    }

    // Add regularization term
    double alpha = 1e-4;

    // Add diagonal entries with mass matrix weighting
    for (int i = 0; i < nVerts; i++) {
        if (diagonalValues[i] < 1e-10) {
            diagonalValues[i] = 1e-10;
        }
        triplets.push_back(Eigen::Triplet<double>(i, i, diagonalValues[i] + alpha));
    }

    // Build the sparse matrix
    L.setFromTriplets(triplets.begin(), triplets.end());

    // Check for NaN/inf values in the Laplacian matrix
    for (int k = 0; k < L.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(L, k); it; ++it) {
            if (std::isnan(it.value()) || std::isinf(it.value())) {
                std::cerr << "Warning: Found NaN or inf in Laplacian at ("
                          << it.row() << ", " << it.col() << ")" << std::endl;
                it.valueRef() = 0.0;
            }
        }
    }

    laplacianComputed = true;

    // Re-compute the factorization with the regularized L using SimplicialLDLT
    std::cout << "Computing LDLT factorization of regularized Laplacian (alpha=" << alpha << ")..." << std::endl; // --- Update message ---
    laplacianSolver.compute(L); // Compute factorization

    if (laplacianSolver.info() != Eigen::Success) {
        // --- Update error message ---
        std::cerr << "ERROR: Failed to compute LDLT factorization of the regularized Laplacian matrix! Solver status: " << laplacianSolver.info() << std::endl;
        solverComputed = false;
        return false;
    } else {
        std::cout << "LDLT factorization computed successfully." << std::endl; // --- Update message ---
        solverComputed = true;
    }

    return true;
}

Eigen::VectorXd PairwiseHarmonics::computePairwiseHarmonic(int vertex_p_idx, int vertex_q_idx) {
    // Ensure Laplacian and its factorization are ready
    if (!laplacianComputed) {
        std::cout << "Laplacian not computed. Computing now..." << std::endl;
        if (!computeCotangentLaplacian()) {
            std::cerr << "Failed to compute Laplacian matrix!" << std::endl;
            return Eigen::VectorXd();
        }
    }
    if (!solverComputed) {
         std::cerr << "ERROR: Laplacian solver factorization is not available!" << std::endl;
         return Eigen::VectorXd();
    }

    int nVerts = mesh->verts.size();

    // --- Direct Constrained Solve ---
    Eigen::SparseMatrix<double> L_mod = L; // Start with a copy of the regularized Laplacian L
    Eigen::VectorXd b_mod = Eigen::VectorXd::Zero(nVerts);

    // --- Enforce f(p) = 0 ---
    // 1. Adjust b_mod for rows j != p: b_mod(j) -= L(j, p) * 0. No change needed.

    // --- Enforce f(q) = 1 ---
    // 1. Adjust b_mod for rows j != p, j != q: b_mod(j) -= Original_L(j, q) * 1
    //    Iterate through column q of the *original* L matrix
    for (Eigen::SparseMatrix<double>::InnerIterator it(L, vertex_q_idx); it; ++it) {
        if (it.row() != vertex_p_idx && it.row() != vertex_q_idx) {
            b_mod(it.row()) -= it.value() * 1.0; // Subtract L(j, q) * 1
        }
    }

    // --- Modify L_mod structure ---
    // Create a list of triplets for the modified matrix. This is often safer than direct coeffRef modification.
    std::vector<Eigen::Triplet<double>> triplets_mod;
    triplets_mod.reserve(L.nonZeros()); // Reserve estimate

    for (int k = 0; k < L_mod.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(L_mod, k); it; ++it) {
            int row = it.row();
            int col = it.col();
            double value = it.value();

            if (row == vertex_p_idx) { // Row p modifications
                if (col == vertex_p_idx) {
                    triplets_mod.push_back(Eigen::Triplet<double>(row, col, 1.0));
                }
                // else: omit other elements in row p (effectively zeroing them)
            } else if (row == vertex_q_idx) { // Row q modifications
                 if (col == vertex_q_idx) {
                    triplets_mod.push_back(Eigen::Triplet<double>(row, col, 1.0));
                }
                 // else: omit other elements in row q (effectively zeroing them)
            } else { // Modifications for other rows j != p, j != q
                if (col == vertex_p_idx || col == vertex_q_idx) {
                    // Omit elements in columns p and q for rows j != p, j != q
                } else {
                    triplets_mod.push_back(Eigen::Triplet<double>(row, col, value));
                }
            }
        }
    }
    // Ensure diagonal elements for p and q are present if they weren't in the original sparsity pattern
    bool p_diag_found = false;
    bool q_diag_found = false;
    for(const auto& triplet : triplets_mod) {
        if(triplet.row() == vertex_p_idx && triplet.col() == vertex_p_idx) p_diag_found = true;
        if(triplet.row() == vertex_q_idx && triplet.col() == vertex_q_idx) q_diag_found = true;
    }
    if (!p_diag_found) triplets_mod.push_back(Eigen::Triplet<double>(vertex_p_idx, vertex_p_idx, 1.0));
    if (!q_diag_found) triplets_mod.push_back(Eigen::Triplet<double>(vertex_q_idx, vertex_q_idx, 1.0));


    L_mod.setFromTriplets(triplets_mod.begin(), triplets_mod.end()); // Rebuild L_mod

    // --- Set b_mod constraints ---
    b_mod(vertex_p_idx) = 0.0;
    b_mod(vertex_q_idx) = 1.0;


    // --- Solve L_mod * f = b_mod ---
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> modSolver;
    modSolver.compute(L_mod);
    if (modSolver.info() != Eigen::Success) {
        std::cerr << "Failed to factorize modified Laplacian L_mod! Status: " << modSolver.info() << std::endl;
        // +++ Debug: Print matrix info if factorization fails +++
        // std::cerr << "L_mod dimensions: " << L_mod.rows() << "x" << L_mod.cols() << ", Nonzeros: " << L_mod.nonZeros() << std::endl;
        // You could potentially write L_mod and b_mod to a file here for inspection
        // --- End Debug ---
        return Eigen::VectorXd();
    }

    Eigen::VectorXd f = modSolver.solve(b_mod);
    if (modSolver.info() != Eigen::Success) {
        std::cerr << "Failed to solve system L_mod * f = b_mod! Status: " << modSolver.info() << std::endl;
        return Eigen::VectorXd();
    }

    return f;
}

Eigen::VectorXd PairwiseHarmonics::computeDDescriptor(int vertex_p_idx, int vertex_q_idx, const Eigen::VectorXd& field, int numSamplesK, const float* dist_p, const float* dist_q) {
    Eigen::VectorXd D_descriptor = Eigen::VectorXd::Zero(numSamplesK);
    int N = mesh->verts.size();

    // --- Check if precomputed distances are provided ---
    if (!dist_p || !dist_q) {
        std::cerr << "Error in computeDDescriptor: Precomputed distance arrays are null." << std::endl;
        return D_descriptor; // Return zero vector
    }
    // --- End Check ---

    // Sample K iso-values between 0 and 1
    for (int k = 0; k < numSamplesK; ++k) {
        double isoValue = (double)(k + 1) / (numSamplesK + 1);
        double totalDistanceSum = 0.0;
        int intersectionCount = 0; // Count intersection points for averaging

        // Iterate through each triangle to find intersections for this isoValue
        for (const auto& tri : mesh->tris) {
            int v1i = tri->v1i;
            int v2i = tri->v2i;
            int v3i = tri->v3i;

            double val1 = field(v1i);
            double val2 = field(v2i);
            double val3 = field(v3i);

            // Helper lambda to process an edge intersection
            auto processIntersection = [&](int ui, int vi, double val_u, double val_v) {
                // Avoid division by zero or cases where edge is nearly flat w.r.t field
                if (std::abs(val_u - val_v) > 1e-9) {
                    double t = (isoValue - val_u) / (val_v - val_u);

                    // Clamp t to [0, 1] for robustness
                    t = std::max(0.0, std::min(1.0, t));

                    // Get precomputed geodesic distances for edge endpoints using the passed arrays
                    double d_up = dist_p[ui];
                    double d_vp = dist_p[vi];
                    double d_uq = dist_q[ui];
                    double d_vq = dist_q[vi];

                    // Check for infinite distances (unreachable vertices)
                    if (std::isinf(d_up) || std::isinf(d_vp) || std::isinf(d_uq) || std::isinf(d_vq)) {
                        return; // Skip this intersection if endpoints are unreachable from p or q
                    }

                    // Interpolate geodesic distances to the intersection point x
                    double d_xp = (1.0 - t) * d_up + t * d_vp;
                    double d_xq = (1.0 - t) * d_uq + t * d_vq;

                    totalDistanceSum += (d_xp + d_xq);
                    intersectionCount++;
                }
            };

            // Check edge 1-2
            if ((val1 < isoValue && val2 > isoValue) || (val1 > isoValue && val2 < isoValue)) {
                processIntersection(v1i, v2i, val1, val2);
            }
            // Check edge 2-3
            if ((val2 < isoValue && val3 > isoValue) || (val2 > isoValue && val3 < isoValue)) {
                processIntersection(v2i, v3i, val2, val3);
            }
            // Check edge 3-1
            if ((val3 < isoValue && val1 > isoValue) || (val3 > isoValue && val1 < isoValue)) {
                processIntersection(v3i, v1i, val3, val1);
            }
        } // End triangle loop

        // Calculate average distance sum for this iso-value
        // Each segment contributes two points, so divide by count/2 (or multiply sum by 2/count)
        if (intersectionCount > 0) {
            D_descriptor(k) = 2.0 * totalDistanceSum / intersectionCount;
        } else {
            D_descriptor(k) = 0.0; // Or handle as error/special value
        }

    } // End iso-value loop

    return D_descriptor;
}

SoSeparator* PairwiseHarmonics::visualizeHarmonicField(const Eigen::VectorXd& field, Painter* painter) {
    SoSeparator* sep = new SoSeparator();

    // Create a material for the field visualization
    SoMaterial* material = new SoMaterial();
    sep->addChild(material);

    // Create coordinates for all vertices
    SoCoordinate3* coords = new SoCoordinate3();
    for (size_t i = 0; i < mesh->verts.size(); i++) {
        // we use SbVec3f to match Coin3D's SoCoordinate3
        coords->point.set1Value(i, mesh->verts[i]->coords);
    }
    sep->addChild(coords);

    // Create a face set with colors based on field values
    SoIndexedFaceSet* faceSet = new SoIndexedFaceSet();

    // Set up face indices
    int faceIdx = 0;
    for (size_t i = 0; i < mesh->tris.size(); i++) {
        faceSet->coordIndex.set1Value(faceIdx++, mesh->tris[i]->v1i);
        faceSet->coordIndex.set1Value(faceIdx++, mesh->tris[i]->v2i);
        faceSet->coordIndex.set1Value(faceIdx++, mesh->tris[i]->v3i);
        faceSet->coordIndex.set1Value(faceIdx++, -1);
    }

    // Set up color mapping for faces based on average vertex values
    SoMaterial* mat = new SoMaterial();
    for (size_t i = 0; i < mesh->tris.size(); i++) {
        // Compute average field value for this face
        double avgValue = 0.0;
        avgValue += field(mesh->tris[i]->v1i);
        avgValue += field(mesh->tris[i]->v2i);
        avgValue += field(mesh->tris[i]->v3i);
        avgValue /= 3.0;

        // Map value from [0,1] to color (blue to red rainbow)
        float r = static_cast<float>(avgValue);
        float g = 0.0f;
        float b = static_cast<float>(1.0 - avgValue);

        // Add color to material
        mat->diffuseColor.set1Value(i, r, g, b);
    }

    sep->addChild(mat);
    sep->addChild(faceSet);

    return sep;
}

double PairwiseHarmonics::computeIsoCurveLength(const Eigen::VectorXd& field, double isoValue){
    double totalLength = 0.0;
    double tolerance = 1e-9; // Tolerance for checking equality with isoValue


    // Part 1: Handle segments crossing *through* triangles
    // This part find intersection points for edges strictly crossed by isoValue
    // Iterate through each triangle
    for (const auto& tri : mesh->tris){
        int v1i = tri->v1i;
        int v2i = tri->v2i;
        int v3i = tri->v3i;

        double val1 = field(v1i);
        double val2 = field(v2i);
        double val3 = field(v3i);

        const SbVec3f& p1_coords = mesh->verts[v1i]->coords;
        const SbVec3f& p2_coords = mesh->verts[v2i]->coords;
        const SbVec3f& p3_coords = mesh->verts[v3i]->coords;

        Eigen::Vector3d p1(p1_coords[0], p1_coords[1], p1_coords[2]);
        Eigen::Vector3d p2(p2_coords[0], p2_coords[1], p2_coords[2]);
        Eigen::Vector3d p3(p3_coords[0], p3_coords[1], p3_coords[2]);

        std::vector<Eigen::Vector3d> intersectionPoints;

        // Check edge 1-2
        // Only ad intersection if isoValue is strictly between val1 and val2
        if((val1 < isoValue && val2 > isoValue) || (val1 > isoValue && val2 < isoValue)){
            if(std::abs(val1-val2) > tolerance){ // Avoid division by zero
                double t = (isoValue - val1) / (val2 - val1);
                // Clamp t to [0, 1] for robustness, although it should be if condition is met
                t = std::max(0.0, std::min(1.0, t));
                intersectionPoints.push_back(p1 + t * (p2 - p1));

            }
            // If abs(val1-val2) <= tolerance, but they are on opposite sides,
            // it implies one or both are very close to isoValue.
            // The edge case (Part 2) might handle edges exactly on isoValue.
        }

        // Check edge 2-3
        if ((val2 < isoValue && val3 > isoValue) || (val2 > isoValue && val3 < isoValue)) {
            if (std::abs(val2 - val3) > tolerance) {
               double t = (isoValue - val2) / (val3 - val2);

               t = std::max(0.0, std::min(1.0, t));
               intersectionPoints.push_back(p2 + t * (p3 - p2));
           }
       }
       // Check edge 3-1
       if ((val3 < isoValue && val1 > isoValue) || (val3 > isoValue && val1 < isoValue)) {
            if (std::abs(val3 - val1) > tolerance) {
               double t = (isoValue - val3) / (val1 - val3); // Corrected interpolation
               t = std::max(0.0, std::min(1.0, t)); // Clamp t to [0, 1]
               intersectionPoints.push_back(p3 + t * (p1 - p3));
           }
        }


        // If exactly two intersection points are found in this triangle, add the segment length
        if (intersectionPoints.size() == 2) {
            totalLength += (intersectionPoints[0] - intersectionPoints[1]).norm();

        }
        // Cases where size is 0, 1, or 3 (e.g., isoValue passes exactly through a vertex)
        // are not explicitly handled by this part.
        // Part 2 handles edges lying exactly on the contour.

    }
    // Part 2: Handle edges lying exactly on the isocontour
    // Iterate through unique edges to avoid double counting.

    std::vector<bool> edgeProcessed(mesh->edges.size(), false); // Keep track of processed edges
    for (size_t edgeIdx = 0; edgeIdx < mesh->edges.size(); ++edgeIdx) {
        if (edgeProcessed[edgeIdx]) continue; // Skip if already processed

        Edge* edge = mesh->edges[edgeIdx];
        int v1i = edge->v1i;
        int v2i = edge->v2i;

        double val1 = field(v1i);
        double val2 = field(v2i);

        // Check if both vertices are (close enough to ) isoValue
        if (std::abs(val1 - isoValue) < tolerance && std::abs(val2 - isoValue) < tolerance) {
            const SbVec3f& p1_coords = mesh->verts[v1i]->coords;
            const SbVec3f& p2_coords = mesh->verts[v2i]->coords;
            Eigen::Vector3d p1(p1_coords[0], p1_coords[1], p1_coords[2]);
            Eigen::Vector3d p2(p2_coords[0], p2_coords[1], p2_coords[2]);

            totalLength += (p1 - p2).norm(); // Add the length of the edge
            // Mark edge as processed (optional if loop structure guarantees single visit)
            // edgeProcessed[edgeIdx] = true;
        }

    }

    return totalLength;
}


Eigen::VectorXd PairwiseHarmonics::computeRDescriptor(const Eigen::VectorXd& field, int numSamplesK) {
    Eigen::VectorXd R_descriptor(numSamplesK);

    // Sample K iso-values between 0 and 1
    for (int k = 0; k < numSamplesK; ++k) {
        double isoValue = (double)(k + 1) / (numSamplesK + 1);
        double totalLength = 0.0;
        int segmentCount = 0;

        // Iterate through each triangle to find segments for this isoValue
        for (const auto& tri : mesh->tris) {
            int v1i = tri->v1i;
            int v2i = tri->v2i;
            int v3i = tri->v3i;

            double val1 = field(v1i);
            double val2 = field(v2i);
            double val3 = field(v3i);

            std::vector<Eigen::Vector3d> intersectionPoints;

            // Check edge 1-2
            if ((val1 < isoValue && val2 > isoValue) || (val1 > isoValue && val2 < isoValue)) {
                double t = (isoValue - val1) / (val2 - val1);
                t = std::max(0.0, std::min(1.0, t)); // Clamp t to [0, 1]
                intersectionPoints.push_back(mesh->verts[v1i]->coords + t * (mesh->verts[v2i]->coords - mesh->verts[v1i]->coords));
            }
            // Check edge 2-3
            if ((val2 < isoValue && val3 > isoValue) || (val2 > isoValue && val3 < isoValue)) {
                double t = (isoValue - val2) / (val3 - val2);
                t = std::max(0.0, std::min(1.0, t));
                intersectionPoints.push_back(mesh->verts[v2i]->coords + t * (mesh->verts[v3i]->coords - mesh->verts[v2i]->coords));
            }
            // Check edge 3-1
            if ((val3 < isoValue && val1 > isoValue) || (val3 > isoValue && val1 < isoValue)) {
                double t = (isoValue - val3) / (val1 - val3);
                t = std::max(0.0, std::min(1.0, t));
                intersectionPoints.push_back(mesh->verts[v3i]->coords + t * (mesh->verts[v1i]->coords - mesh->verts[v3i]->coords));
            }

            // If exactly two intersection points are found, calculate the segment length
            if (intersectionPoints.size() == 2) {
                totalLength += (intersectionPoints[0] - intersectionPoints[1]).norm();
                segmentCount++;
            }
        }

        // Average length per segment for this iso-value
        R_descriptor(k) = (segmentCount > 0) ? totalLength / segmentCount : 0.0;
    }

    return R_descriptor;
}

Eigen::VectorXd PairwiseHarmonics::computeDDescriptor(int vertex_p_idx, int vertex_q_idx, const Eigen::VectorXd& field, int numSamplesK) {
    Eigen::VectorXd D_descriptor = Eigen::VectorXd::Zero(numSamplesK);
    int N = mesh->verts.size();

    // --- Check if precomputed distances are provided ---
    // (This version assumes distances are always computed fresh)
    // --- End Check ---

    // Sample K iso-values between 0 and 1
    for (int k = 0; k < numSamplesK; ++k) {
        double isoValue = (double)(k + 1) / (numSamplesK + 1);
        double totalDistanceSum = 0.0;
        int intersectionCount = 0; // Count intersection points for averaging

        // Iterate through each triangle to find intersections for this isoValue
        for (const auto& tri : mesh->tris) {
            int v1i = tri->v1i;
            int v2i = tri->v2i;
            int v3i = tri->v3i;

            double val1 = field(v1i);
            double val2 = field(v2i);
            double val3 = field(v3i);

            // Helper lambda to process an edge intersection
            auto processIntersection = [&](int ui, int vi, double val_u, double val_v) {
                // Avoid division by zero or cases where edge is nearly flat w.r.t field
                if (std::abs(val_u - val_v) > 1e-9) {
                    double t = (isoValue - val_u) / (val_v - val_u);

                    // Clamp t to [0, 1] for robustness
                    t = std::max(0.0, std::min(1.0, t));

                    // Get precomputed geodesic distances for edge endpoints using the passed arrays
                    double d_up = mesh->verts[ui]->geodesicDistance;
                    double d_vp = mesh->verts[vi]->geodesicDistance;
                    double d_uq = mesh->verts[ui]->geodesicDistance;
                    double d_vq = mesh->verts[vi]->geodesicDistance;

                    // Check for infinite distances (unreachable vertices)
                    if (std::isinf(d_up) || std::isinf(d_vp) || std::isinf(d_uq) || std::isinf(d_vq)) {
                        return; // Skip this intersection if endpoints are unreachable from p or q
                    }

                    // Interpolate geodesic distances to the intersection point x
                    double d_xp = (1.0 - t) * d_up + t * d_vp;
                    double d_xq = (1.0 - t) * d_uq + t * d_vq;

                    totalDistanceSum += (d_xp + d_xq);
                    intersectionCount++;
                }
            };

            // Check edge 1-2
            if ((val1 < isoValue && val2 > isoValue) || (val1 > isoValue && val2 < isoValue)) {
                processIntersection(v1i, v2i, val1, val2);
            }
            // Check edge 2-3
            if ((val2 < isoValue && val3 > isoValue) || (val2 > isoValue && val3 < isoValue)) {
                processIntersection(v2i, v3i, val2, val3);
            }
            // Check edge 3-1
            if ((val3 < isoValue && val1 > isoValue) || (val3 > isoValue && val1 < isoValue)) {
                processIntersection(v3i, v1i, val3, val1);
            }
        } // End triangle loop

        // Calculate average distance sum for this iso-value
        // Each segment contributes two points, so divide by count/2 (or multiply sum by 2/count)
        if (intersectionCount > 0) {
            D_descriptor(k) = 2.0 * totalDistanceSum / intersectionCount;
        } else {
            D_descriptor(k) = 0.0; // Or handle as error/special value
        }

    } // End iso-value loop

    return D_descriptor;
}

// =============================================
// ENHANCED LIBIGL-BASED METHODS IMPLEMENTATION
// =============================================

Eigen::VectorXd PairwiseHarmonics::computeRDescriptorLibIGL(const Eigen::VectorXd& field, int numSamplesK) {
    if (!meshDataComputed) {
        std::cout << "Error: LibIGL not initialized" << std::endl;
        return computeRDescriptor(field, numSamplesK); // Fall back to original
    }

    try {
        std::cout << "Computing R descriptor using enhanced LibIGL methods..." << std::endl;

        Eigen::VectorXd R_descriptor(numSamplesK);

        for (int k = 0; k < numSamplesK; k++) {
            double isovalue = (k + 1.0) / (numSamplesK + 1.0);

            // Use the enhanced isocurve extraction with LibIGL mesh data
            double length = computeIsoCurveLength(field, isovalue);
            R_descriptor(k) = length;
        }

        std::cout << "Enhanced R descriptor computed with " << numSamplesK << " samples" << std::endl;
        return R_descriptor;

    } catch (const std::exception& e) {
        std::cout << "Error in enhanced R descriptor computation: " << e.what() << std::endl;
        return computeRDescriptor(field, numSamplesK); // Fall back to original
    }
}

Eigen::VectorXd PairwiseHarmonics::computeDDescriptorLibIGL(
    int vertex_p_idx, int vertex_q_idx,
    const Eigen::VectorXd& field, int numSamplesK) {

    if (!meshDataComputed) {
        std::cout << "Error: LibIGL not initialized" << std::endl;

        // Fall back to original implementation
        int N;
        float* dist_p = mesh->computeGeodesicDistances(vertex_p_idx, N);
        float* dist_q = mesh->computeGeodesicDistances(vertex_q_idx, N);

        if (dist_p && dist_q) {
            Eigen::VectorXd result = computeDDescriptor(vertex_p_idx, vertex_q_idx, field, numSamplesK, dist_p, dist_q);
            delete[] dist_p;
            delete[] dist_q;
            return result;
        }
        return Eigen::VectorXd();
    }

    try {
        std::cout << "Computing D descriptor using LibIGL heat geodesics..." << std::endl;

        // Use the mesh's LibIGL heat geodesics for fast and accurate distance computation
        Eigen::VectorXd dist_p = mesh->computeHeatGeodesicDistances(vertex_p_idx);
        Eigen::VectorXd dist_q = mesh->computeHeatGeodesicDistances(vertex_q_idx);

        if (dist_p.size() != V.rows() || dist_q.size() != V.rows()) {
            std::cout << "Error: Heat geodesic computation failed" << std::endl;
            return Eigen::VectorXd();
        }

        Eigen::VectorXd D_descriptor(numSamplesK);

        for (int k = 0; k < numSamplesK; k++) {
            double isovalue = (k + 1.0) / (numSamplesK + 1.0);

            // Find vertices close to this isovalue
            std::vector<int> isocurve_vertices;
            double tolerance = 0.1 / numSamplesK; // Adaptive tolerance

            for (int i = 0; i < field.size(); i++) {
                if (std::abs(field(i) - isovalue) < tolerance) {
                    isocurve_vertices.push_back(i);
                }
            }

            if (isocurve_vertices.empty()) {
                D_descriptor(k) = 0.0;
                continue;
            }

            // Compute average geodesic distances for this isocurve
            double avg_dist_p = 0.0, avg_dist_q = 0.0;
            for (int vertex_idx : isocurve_vertices) {
                avg_dist_p += dist_p(vertex_idx);
                avg_dist_q += dist_q(vertex_idx);
            }
            avg_dist_p /= isocurve_vertices.size();
            avg_dist_q /= isocurve_vertices.size();

            D_descriptor(k) = (avg_dist_p + avg_dist_q) / 2.0;
        }

        std::cout << "Enhanced D descriptor computed using heat geodesics" << std::endl;
        return D_descriptor;

    } catch (const std::exception& e) {
        std::cout << "Error in enhanced D descriptor computation: " << e.what() << std::endl;

        // Fall back to original implementation
        int N;
        float* dist_p = mesh->computeGeodesicDistances(vertex_p_idx, N);
        float* dist_q = mesh->computeGeodesicDistances(vertex_q_idx, N);

        if (dist_p && dist_q) {
            Eigen::VectorXd result = computeDDescriptor(vertex_p_idx, vertex_q_idx, field, numSamplesK, dist_p, dist_q);
            delete[] dist_p;
            delete[] dist_q;
            return result;
        }
        return Eigen::VectorXd();
    }
}

Eigen::VectorXd PairwiseHarmonics::computeHeatGeodesicDistances(int source_vertex) {
    if (!mesh) {
        std::cout << "Error: No mesh available" << std::endl;
        return Eigen::VectorXd();
    }

    return mesh->computeHeatGeodesicDistances(source_vertex);
}