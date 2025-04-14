#include "helpers.h"
#include "Painter.h"

// Include Inventor headers
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>

#include <algorithm>

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
    Eigen::Vector3d p(mesh->verts[vertexIdx]->coords);
    
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
        
        // Get the coordinates of all three vertices
        Eigen::Vector3d v1(mesh->verts[v1i]->coords);
        Eigen::Vector3d v2(mesh->verts[v2i]->coords);
        Eigen::Vector3d v3(mesh->verts[v3i]->coords);
        
        // Compute edges
        Eigen::Vector3d e1 = v2 - v1; // edge from v1 to v2
        Eigen::Vector3d e2 = v3 - v1; // edge from v1 to v3
        
        // Compute angles
        double angle = calculateAngle(e1, e2);
        
        // Compute triangle area
        double triangleArea = computeTriangleArea(v1, v2, v3);
        
        // For obtuse triangles, use special case handling
        if (angle > M_PI/2 || 
            calculateAngle(v2-v3, v1-v3) > M_PI/2 || 
            calculateAngle(v3-v2, v1-v2) > M_PI/2) {
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
            Eigen::Vector3d vi(mesh->verts[i]->coords);
            Eigen::Vector3d vj(mesh->verts[j]->coords);
            Eigen::Vector3d vk(mesh->verts[k]->coords);
            
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
    
    // Solve the system Af = b using LDLT decomposition (stable for symmetric matrices)
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    
    // Set solver parameters for better numerical stability
    solver.setShift(1e-10); // Add a small shift to diagonal if needed for stability
    
    // Compute the decomposition
    solver.compute(A);
    
    if (solver.info() != Eigen::Success) {
        std::cerr << "Failed to decompose the Laplacian matrix!" << std::endl;
        // Try a more robust solver as fallback
        Eigen::SparseLU<Eigen::SparseMatrix<double>> luSolver;
        luSolver.compute(A);
        
        if (luSolver.info() != Eigen::Success) {
            return Eigen::VectorXd();
        }
        
        // Solve with LU decomposition instead
        Eigen::VectorXd f = luSolver.solve(b);
        
        if (luSolver.info() != Eigen::Success) {
            std::cerr << "Failed to solve the linear system with fallback solver!" << std::endl;
            return Eigen::VectorXd();
        }
        
        return f;
    }
    
    // Solve the system with LDLT
    Eigen::VectorXd f = solver.solve(b);
    
    if (solver.info() != Eigen::Success) {
        std::cerr << "Failed to solve the linear system!" << std::endl;
        return Eigen::VectorXd();
    }
    
    // Verify boundary conditions
    if (std::abs(f(vertex_p_idx)) > 1e-6 || std::abs(f(vertex_q_idx) - 1.0) > 1e-6) {
        std::cerr << "Warning: Boundary conditions not satisfied precisely!" << std::endl;
        std::cerr << "f(" << vertex_p_idx << ") = " << f(vertex_p_idx) << " (should be 0)" << std::endl;
        std::cerr << "f(" << vertex_q_idx << ") = " << f(vertex_q_idx) << " (should be 1)" << std::endl;
        // Force boundary conditions to be exact
        f(vertex_p_idx) = 0.0;
        f(vertex_q_idx) = 1.0;
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
        float r = avgValue;
        float g = 0.0f;
        float b = 1.0f - avgValue;
        
        // Add color to material
        mat->diffuseColor.set1Value(i, r, g, b);
    }
    
    sep->addChild(mat);
    sep->addChild(faceSet);
    
    return sep;
}

// Free function implementation
SoSeparator* testPairwiseHarmonic(Mesh* mesh, int vertex_p_idx, int vertex_q_idx, Painter* painter) {
    // Create a result separator
    SoSeparator* result = new SoSeparator();
    
    // Create the PairwiseHarmonics object
    PairwiseHarmonics ph(mesh);
    
    // Compute the Laplacian
    if (!ph.computeCotangentLaplacian()) {
        std::cerr << "Failed to compute Laplacian!" << std::endl;
        return result;
    }
    
    // Compute the pairwise harmonic function
    Eigen::VectorXd harmonicField = ph.computePairwiseHarmonic(vertex_p_idx, vertex_q_idx);
    
    if (harmonicField.size() == 0) {
        std::cerr << "Failed to compute pairwise harmonic!" << std::endl;
        return result;
    }
    
    // Visualize the harmonic field
    SoSeparator* fieldVisualization = ph.visualizeHarmonicField(harmonicField, painter);
    result->addChild(fieldVisualization);
    
    // Highlight the source and target vertices
    result->addChild(painter->get1PointSep(mesh, vertex_p_idx, 0.0f, 0.0f, 1.0f, 10.0f, false)); // Blue for p (value 0)
    result->addChild(painter->get1PointSep(mesh, vertex_q_idx, 1.0f, 0.0f, 0.0f, 10.0f, false)); // Red for q (value 1)
    
    return result;
}