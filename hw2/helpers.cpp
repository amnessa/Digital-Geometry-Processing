#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cfloat>
#include <utility>

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

PairwiseHarmonics::PairwiseHarmonics(Mesh* m) : mesh(m), laplacianComputed(false) {}

bool PairwiseHarmonics::computeCotangentLaplacian() {
    int nVerts = mesh->verts.size();
    
    // Initialize the sparse matrix with estimated non-zeros per row
    L.resize(nVerts, nVerts);
    
    // List to store triplets (i, j, value) for sparse matrix construction
    std::vector<Eigen::Triplet<double>> triplets;
    
    // Estimate number of non-zeros based on average vertex degree
    triplets.reserve(nVerts * 7);  // Assuming average vertex degree of 7
    
    // Diagonal values will be computed as sum of off-diagonal entries
    std::vector<double> diagonalValues(nVerts, 0.0);
    
    // Process each edge in the mesh
    for (Edge* edge : mesh->edges) {
        int i = edge->v1i;
        int j = edge->v2i;
        
        // Skip self-loops if any
        if (i == j) continue;
        
        // Find triangles sharing this edge
        std::vector<int> sharedTriangles;
        
        // Find triangles common to both vertices
        for (int triIdxI : mesh->verts[i]->triList) {
            for (int triIdxJ : mesh->verts[j]->triList) {
                if (triIdxI == triIdxJ) {
                    sharedTriangles.push_back(triIdxI);
                    break;
                }
            }
        }
        
        double cotWeight = 0.0;
        
        // For each triangle sharing this edge, compute cotangent weight
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
            // Extremely large values can cause problems in the solver
            if (std::abs(cotTheta) > 1e6) {
                cotTheta = (cotTheta > 0) ? 1e6 : -1e6;
            }
            
            // Add to total weight
            cotWeight += cotTheta * 0.5;
        }
        
        // Handle boundary edges (only one adjacent triangle)
        if (sharedTriangles.size() == 1) {
            // For boundary edges, we only have one cotangent value
            // No modification needed as we're already halving the cotangent
        }
        
        // Skip extremely small weights to maintain sparsity
        if (std::abs(cotWeight) > 1e-10) {
            // Limit extremely large weights to improve conditioning
            if (std::abs(cotWeight) > 1e6) {
                cotWeight = (cotWeight > 0) ? 1e6 : -1e6;
            }
            
            // Negative weight for off-diagonal entries
            triplets.push_back(Eigen::Triplet<double>(i, j, -cotWeight));
            triplets.push_back(Eigen::Triplet<double>(j, i, -cotWeight));
            
            // Add contribution to diagonal entries
            diagonalValues[i] += cotWeight;
            diagonalValues[j] += cotWeight;
        }
    }
    
    // Add diagonal entries
    for (int i = 0; i < nVerts; i++) {
        // Ensure the diagonal is positive to maintain positive-definiteness
        // This helps with numerical stability for the LDLT solver
        if (diagonalValues[i] < 1e-10) {
            diagonalValues[i] = 1e-10;
        }
        triplets.push_back(Eigen::Triplet<double>(i, i, diagonalValues[i]));
    }
    
    // Build the sparse matrix
    L.setFromTriplets(triplets.begin(), triplets.end());
    
    // Check for NaN values in the Laplacian matrix
    for (int k = 0; k < L.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(L, k); it; ++it) {
            if (std::isnan(it.value()) || std::isinf(it.value())) {
                std::cerr << "Warning: Found NaN or inf in Laplacian at (" 
                          << it.row() << ", " << it.col() << ")" << std::endl;
                it.valueRef() = 0.0;
            }
        }
    }
    
    // Mark as computed
    laplacianComputed = true;
    
    return true;
}

Eigen::VectorXd PairwiseHarmonics::computePairwiseHarmonic(int vertex_p_idx, int vertex_q_idx) {
    // Compute Laplacian if not already done
    if (!laplacianComputed) {
        if (!computeCotangentLaplacian()) {
            std::cerr << "Failed to compute Laplacian matrix!" << std::endl;
            return Eigen::VectorXd();
        }
    }
    
    int nVerts = mesh->verts.size();
    
    // Check if indices are valid
    if (vertex_p_idx < 0 || vertex_p_idx >= nVerts || 
        vertex_q_idx < 0 || vertex_q_idx >= nVerts) {
        std::cerr << "Invalid vertex indices for pairwise harmonic computation!" << std::endl;
        return Eigen::VectorXd();
    }
    
    // Make a copy of the Laplacian matrix
    Eigen::SparseMatrix<double> A = L;
    
    // Initialize the right-hand side vector b to zeros
    Eigen::VectorXd b = Eigen::VectorXd::Zero(nVerts);
    
    // Ensure the matrix is in compressed mode for faster coefficient access
    A.makeCompressed();
    
    // Modify A and b for boundary conditions
    
    // For vertex p: f(p) = 0
    // Zero out row p
    for (int k = 0; k < A.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
            if (it.row() == vertex_p_idx) {
                it.valueRef() = 0.0;
            }
        }
    }
    // Set diagonal element A(p,p) = 1.0
    A.coeffRef(vertex_p_idx, vertex_p_idx) = 1.0;
    // Set b(p) = 0.0 (already zero)
    
    // For vertex q: f(q) = 1
    // Zero out row q
    for (int k = 0; k < A.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
            if (it.row() == vertex_q_idx) {
                it.valueRef() = 0.0;
            }
        }
    }
    // Set diagonal element A(q,q) = 1.0
    A.coeffRef(vertex_q_idx, vertex_q_idx) = 1.0;
    // Set b(q) = 1.0
    b(vertex_q_idx) = 1.0;
    
    // Ensure the matrix is in a consistent state after modifications
    A.prune(1e-10); // Remove extremely small values (helps with numerical stability)
    
    // Solve the system Af = b using SparseLU 
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    
    
    // Compute the decomposition
    solver.compute(A);
    
    if (solver.info() != Eigen::Success) {
        std::cerr << "Failed to decompose the Laplacian matrix with SparseLU!" << std::endl;
        return Eigen::VectorXd(); // Return empty vector on failure

    }
    
    // Solve the system with LDLT
    Eigen::VectorXd f = solver.solve(b);
    
    if (solver.info() != Eigen::Success) {
        std::cerr << "Failed to solve the linear system!" << std::endl;
        return Eigen::VectorXd();
    }
    
    // Verify boundary conditions
    if (std::abs(f(vertex_p_idx)) > 1e-6 || std::abs(f(vertex_q_idx) - 1.0) > 1e-6) {
        // This warning should ideally disappear after switching to SparseLU
        std::cerr << "Warning: Boundary conditions still not satisfied precisely after LU solve!" << std::endl;
        std::cerr << "f(" << vertex_p_idx << ") = " << f(vertex_p_idx) << " (should be 0)" << std::endl;
        std::cerr << "f(" << vertex_q_idx << ") = " << f(vertex_q_idx) << " (should be 1)" << std::endl;
        // *not* forcing the values if LU still fails, to better diagnose
        // f(vertex_p_idx) = 0.0; 
        // f(vertex_q_idx) = 1.0;
    }
    
    return f;
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

        if((val1 < isoValue && val2 > isoValue) || (val1 > isoValue && val2 < isoValue)){
            if(std::abs(val1-val2) > 1e-9){ // Avoid division by zero
                double t = (isoValue - val1) / (val2 - val1);
                intersectionPoints.push_back(p1 + t * (p2 - p1));
                
            }
        }

        // Check edge 2-3
        if ((val2 < isoValue && val3 > isoValue) || (val2 > isoValue && val3 < isoValue)) {
            if (std::abs(val2 - val3) > 1e-9) {
               double t = (isoValue - val2) / (val3 - val2);
               intersectionPoints.push_back(p2 + t * (p3 - p2));
           }
       }
       // Check edge 3-1
       if ((val3 < isoValue && val1 > isoValue) || (val3 > isoValue && val1 < isoValue)) {
            if (std::abs(val3 - val1) > 1e-9) {
               double t = (isoValue - val3) / (p1 - p3).norm(); // Corrected interpolation
               intersectionPoints.push_back(p3 + t * (p1 - p3));
           }
        }
        
    
        // If exactly two intersection points are found in this triangle, add the segment length
        if (intersectionPoints.size() == 2) {
            totalLength += (intersectionPoints[0] - intersectionPoints[1]).norm();
        
        }
        // Handle cases with more intersections if necessary (e.g., iso-value passes through a vertex)
        // For simplicity, we only handle the standard case of crossing two edges here.

    }
    return totalLength;
}


Eigen::VectorXd PairwiseHarmonics::computeRDescriptor(const Eigen::VectorXd& field, int numSamplesK) {
    Eigen::VectorXd R(numSamplesK);

    for (int k = 0; k<numSamplesK; ++k){
        // Sample iso-value uniformly from (0, 1)
        double isoValue = static_cast<double>(k +1) / (numSamplesK + 1);
        R(k) = computeIsoCurveLength(field, isoValue);
    }
    
    return R;
}

Eigen::VectorXd PairwiseHarmonics::computeDDescriptor(int vertex_p_idx, int vertex_q_idx, const Eigen::VectorXd& field, int numSamplesK) {
 
    Eigen::VectorXd D(numSamplesK);
    D.setZero(); // Initialize descriptor to zeros
    int N = mesh->verts.size();

    // --- Precompute Geodesic Distances ---
    std::cout << "Precomputing geodesic distances from p=" << vertex_p_idx << "..." << std::endl;
    int numVerts_p;
    float* dist_p_raw = mesh->computeGeodesicDistances(vertex_p_idx, numVerts_p);
    if (!dist_p_raw || numVerts_p != N) {
        std::cerr << "Error computing distances from p!" << std::endl;
        if (dist_p_raw) delete[] dist_p_raw;
        return D; // Retrun zero vector on error

    }

    // Convert to Eigen::VectorXd for easier access (optional, but convenient)
    Eigen::VectorXd dist_p(N);
    for(int i=0;i<N;++i) dist_p(i) = (dist_p_raw[i] == FLT_MAX) ? std::numeric_limits<double>::infinity() : static_cast<double>(dist_p_raw[i]);


    std::cout << " Precomputing geodesic distance from q=" << vertex_q_idx << "..." << std::endl;
    int numVerts_q;
    float* dist_q_raw = mesh->computeGeodesicDistances(vertex_q_idx, numVerts_q);
    if (!dist_q_raw || numVerts_q != N) {
        std::cerr << "Error computing distances from q!" << std::endl;
        if (dist_q_raw) delete[] dist_q_raw;
        return D; // Return zero vector on error
    }
    Eigen::VectorXd dist_q(N);
    for(int i=0;i<N;++i) dist_q(i) = (dist_q_raw[i] == FLT_MAX) ? std::numeric_limits<double>::infinity() : static_cast<double>(dist_q_raw[i]);

    std::cout << " Geodesic distances precomputed." << std::endl;

    // --- End Precompute Geodesic Distances ---

    for (int k = 0; k< numSamplesK; ++k){
        double isoValue = static_cast<double>(k + 1) / (numSamplesK + 1);
        double totalDistanceSum = 0.0;
        int intersectionCount = 0;

        // Iterate through each triangle to find intersections for this isoValue
        for (const auto& tri: mesh -> tris){
            int v1i = tri->v1i;
            int v2i = tri->v2i;
            int v3i = tri->v3i;

            double val1 = field(v1i);
            double val2 = field(v2i);
            double val3 = field(v3i);

            // Helper lambda to process an edge intersection
            auto processIntersection = [&](int ui, int vi, double val_u, double val_v) {
                if (std::abs(val_u - val_v)> 1e-9) { // Avoid division by zero
                    double t = (isoValue - val_u) / (val_v - val_u);

                    // Get precomputed geodesic distances for edge endpoints
                    double d_up = dist_p(ui);
                    double d_vp = dist_p(vi);
                    double d_uq = dist_q(ui);
                    double d_vq = dist_q(vi);

                    // Check for infinite distances (unreachable vertices)
                    if (std::isinf(d_up) || std::isinf(d_vp) || std::isinf(d_uq) || std::isinf(d_vq)) {
                        // Skip this intersection if endpoints are unreachable from p or q
                        // Or handle differently (e.g., assign a large penalty)
                        return;
                        
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

        // Calculate average distance for this iso-value
        if (intersectionCount > 0) {
            D(k) = totalDistanceSum / static_cast<double>(intersectionCount);
} else {
    // Handle cases where no intersections are found for an iso-value
    // (e.g., isoValue matches a vertex value exactly, or numerical issues)
    // Setting to 0 or NaN might be options depending on desired behavior
    D(k) = 0.0; // Or std::numeric_limits<double>::quiet_NaN();
    //std::cerr << "Warning: No intersections found for isoValue " << isoValue << std::endl;
}
    } 
    // Clean up raw distance arrays
    delete[] dist_p_raw;
    delete[] dist_q_raw;

    return D;
}

double PairwiseHarmonics::computePIS(const Eigen::VectorXd& R_pq, const Eigen::VectorXd& D_pq, const Eigen::VectorXd& R_qp, const Eigen::VectorXd& D_qp){
    int K = R_pq.size();
    if (K == 0 || K != D_pq.size() || K != R_qp.size() || K != D_qp.size()) {
        std::cerr << "Error: Descriptor size mismatch in computePIS. K=" << K << std::endl;
        return 0.0; // Indicate error or poor symmetry
    }

    double sum_L = 0.0;
    for (int k = 0; k<K; ++k){
        double r_pq_k = R_pq(k);
        double d_pq_k = D_pq(k);
        // Note :Paper uses 1-based indexing, C++ uses 0-based
        // k maps to paper's k+1. K-k maps to paper's K-(K+1)+1 = K-k.
        // We need R_qp(K-1-k) and D_qp(K-1-k) for 0-based index.
        int k_sym = K - 1 - k;
        double r_qp_sym = R_qp(k_sym);
        double d_qp_sym = D_qp(k_sym);

        // L(x, y) = |x- y| / (x+ y) - handle division by zero

        auto compute_L = [](double x, double y) {
            if (std::abs(x+y)<1e-9) {
                return 0.0; // If both are near zero, difference is small, treat as symmetric
            }
            return std::abs(x-y) / (x+y);
        };

        sum_L += compute_L(r_pq_k, r_qp_sym);
        sum_L += compute_L(d_pq_k, d_qp_sym);
    }

    // PIS = exp(- (1/K)*sum_L)
    double pis_score = std::exp(-sum_L / static_cast<double>(K));
    return pis_score;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> PairwiseHarmonics::extractIsoCurveSegments(const Eigen::VectorXd& field, double isoValue) {
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> segments;

    // Iterate through each triangle
    for (const auto& tri : mesh->tris) {
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
        if ((val1 < isoValue && val2 > isoValue) || (val1 > isoValue && val2 < isoValue)) {
            if (std::abs(val1 - val2) > 1e-9) {
                double t = (isoValue - val1) / (val2 - val1);
                intersectionPoints.push_back(p1 + t * (p2 - p1));
            }
        }
        // Check edge 2-3
        if ((val2 < isoValue && val3 > isoValue) || (val2 > isoValue && val3 < isoValue)) {
            if (std::abs(val2 - val3) > 1e-9) {
                double t = (isoValue - val2) / (val3 - val2);
                intersectionPoints.push_back(p2 + t * (p3 - p2));
            }
        }
        // Check edge 3-1
        if ((val3 < isoValue && val1 > isoValue) || (val3 > isoValue && val1 < isoValue)) {
            if (std::abs(val3 - val1) > 1e-9) {
                double t = (isoValue - val3) / (val1 - val3);
                intersectionPoints.push_back(p3 + t * (p1 - p3));
            }
        }

        // If exactly two intersection points are found in this triangle, add the segment
        if (intersectionPoints.size() == 2) {
            segments.push_back({intersectionPoints[0], intersectionPoints[1]});
        }
        // Handle cases with more intersections if necessary (e.g., iso-value passes through a vertex)
        // For simplicity, we only handle the standard case of crossing two edges here.
    }
    return segments;
}

// Free function implementation
SoSeparator* testPairwiseHarmonic(Mesh* mesh, int vertex_p_idx, int vertex_q_idx, Painter* painter) {
    // Create a result separator
    SoSeparator* result = new SoSeparator();
    
    // Create the PairwiseHarmonics object
    PairwiseHarmonics ph(mesh);
    
    // Compute the Laplacian
    std::cout << "Computing Cotangent Laplacian..." << std::endl;
    if (!ph.computeCotangentLaplacian()) {
        std::cerr << "Failed to compute Laplacian!" << std::endl;
        return result;
    }
    std::cout << "Laplacian computed." << std::endl;
    
    // Compute the pairwise harmonic function
    std::cout << "Computing Pairwise Harmonic for vertices " << vertex_p_idx << " and " << vertex_q_idx << "..." << std::endl;
    Eigen::VectorXd harmonicField = ph.computePairwiseHarmonic(vertex_p_idx, vertex_q_idx);
    
    if (harmonicField.size() == 0) {
        std::cerr << "Failed to compute pairwise harmonic!" << std::endl;
        return result;
    }
    std::cout << "Pairwise harmonic computed." << std::endl;

    // Visualize the harmonic field
    SoSeparator* fieldVisualization = ph.visualizeHarmonicField(harmonicField, painter);
    result->addChild(fieldVisualization);
    
    // Highlight the source and target vertices
    result->addChild(painter->get1PointSep(mesh, vertex_p_idx, 0.0f, 0.0f, 1.0f, 5.0f, false)); // Blue for p (value 0)
    result->addChild(painter->get1PointSep(mesh, vertex_q_idx, 1.0f, 0.0f, 0.0f, 5.0f, false)); // Red for q (value 1)

    // --- Test R descriptor ---
    int numSamplesK = 10; // Number of samples for R descriptor
    std::cout << "Computing R Descriptor with K=" << numSamplesK << "..." << std::endl;
    Eigen::VectorXd R_descriptor = ph.computeRDescriptor(harmonicField, numSamplesK);

    if (R_descriptor.size() ==  numSamplesK) {
        std::cout << "R Descriptor computed successfully." << std::endl;
        std::cout << R_descriptor.transpose() << std::endl; // Print the vector horizontally
    } else {
        std::cerr << "Failed to compute R Descriptor or size mismatch!" << std::endl;
    }
    // --- End R descriptor test ---

    // --- Test D descriptor ---
    std::cout << "Computing D Descriptor with K=" << numSamplesK << "..." << std::endl;
    Eigen::VectorXd D_descriptor = ph.computeDDescriptor(vertex_p_idx, vertex_q_idx, harmonicField, numSamplesK);

    if (D_descriptor.size() == numSamplesK) {
        std::cout << "D Descriptor computed successfully." << std::endl;
        std::cout << D_descriptor.transpose() << std::endl; // Print the vector horizontally
    } else {
        std::cerr << "Failed to compute D Descriptor or size mismatch!" << std::endl;
    }

    // --- End D descriptor test ---
    
    return result;
}