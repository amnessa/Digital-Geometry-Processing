#pragma once

// Pre-include STL headers before Eigen to help avoid conflicts
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>

// Explicitly ensure min/max are from std namespace
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

// Forward declaration of Inventor classes
class SoSeparator;
class SoMaterial;
class SoCoordinate3;
class SoIndexedFaceSet;

// Include Eigen headers after potential macro cleanup
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>

// Include libigl headers for robust geometry processing
#include <igl/cotmatrix.h>
#include <igl/harmonic.h>
#include <igl/massmatrix.h>
#include <igl/boundary_facets.h>
#include <igl/slice.h>
#include <igl/avg_edge_length.h>
#include <igl/heat_geodesics.h>
#include <igl/exact_geodesic.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>

#include "Mesh.h"

// Forward declarations
class Painter;

/**
 * Helper class for pairwise harmonics calculations
 * Implements Phase 1 of the pairwise harmonics algorithm
 * for intrinsic symmetry detection
 */
class PairwiseHarmonics {
private:
    // The mesh we're working with
    Mesh* mesh;

    // Sparse matrix representing the cotangent Laplacian
    Eigen::SparseMatrix<double> L;

    // Mass matrix for the mesh
    Eigen::SparseMatrix<double> M;

    // Flag indicating if the Laplacian has been computed
    bool laplacianComputed;

    // +++ Change Solver Type +++
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> laplacianSolver; // Use SimplicialLDLT
    bool solverComputed; // Flag to track if solver is ready

    // Mesh data in libigl format
    Eigen::MatrixXd V; // Vertices
    Eigen::MatrixXi F; // Faces
    bool meshDataComputed;

    // Helper methods
    double calculateCotangent(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
    double calculateAngle(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
    double computeTriangleArea(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3);
    double computeVoronoiArea(int vertexIdx);

    // New libigl-based helper methods
    bool prepareMeshDataForLibigl();

public:
    /**
     * Constructor
     * @param m Pointer to the mesh
     */
    // +++ Modify Constructor Declaration +++
    PairwiseHarmonics(Mesh* m); // Remove the inline definition body {}

    /**
     * Compute the cotangent Laplacian matrix for the mesh using libigl
     * This provides more robust computation than manual implementation
     * @return True if computation was successful
     */
    bool computeCotangentLaplacianLibigl();

    /**
     * Compute the cotangent Laplacian matrix for the mesh (original implementation)
     * This implements the cotangent formulation of the Laplace-Beltrami operator
     * @return True if computation was successful
     */
    bool computeCotangentLaplacian();

    /**
     * Compute the pairwise harmonic function between two vertices using libigl
     * This uses libigl's harmonic function solver for more robust computation
     * @param vertex_p_idx Index of the first boundary vertex (f=0)
     * @param vertex_q_idx Index of the second boundary vertex (f=1)
     * @return Vector of harmonic function values for each vertex
     */
    Eigen::VectorXd computePairwiseHarmonicLibIGL(int vertex_p_idx, int vertex_q_idx);

    /**
     * Compute the pairwise harmonic function between two vertices (original implementation)
     * Solves the Laplace equation with Dirichlet boundary conditions
     * @param vertex_p_idx Index of the first boundary vertex (f=0)
     * @param vertex_q_idx Index of the second boundary vertex (f=1)
     * @return Vector of harmonic function values for each vertex
     */
    Eigen::VectorXd computePairwiseHarmonic(int vertex_p_idx, int vertex_q_idx);

    /**
     * Visualize the harmonic field on the mesh
     * @param field Vector of harmonic function values
     * @param painter Pointer to the Painter class for visualization
     * @return SoSeparator containing the visualization
     */
    SoSeparator* visualizeHarmonicField(const Eigen::VectorXd& field, Painter* painter);

    /**
     * Computes the total length of the iso-curve for a given value in the harmonic field.
     * Uses a simplified approach by summing segment lengths within triangles.
     * @param field The computed harmonic field (vector of values per vertex).
     * @param isoValue The target value for the iso-curve (between 0 and 1).
     * @return The total length of the iso-curve segments.
     */
    double computeIsoCurveLength(const Eigen::VectorXd& field, double isoValue);

    /**
     * Computes the R descriptor (perimeter distribution) for a given harmonic field.
     * @param field The computed harmonic field.
     * @param numSamplesK The number of iso-curves to sample for the descriptor.
     * @return An Eigen::VectorXd of size numSamplesK containing the lengths of the sampled iso-curves.
     */
    Eigen::VectorXd computeRDescriptor(const Eigen::VectorXd& field, int numSamplesK);

    /**
     * Computes the D descriptor (average geodesic distance distribution) for a given harmonic field.
     *  Requires geodesic distance from the source (p) and target (q) vertices.
     * @param vertex_p_idx Index of the source vertex p (field value 0).
     * @param vertex_q_idx Index of the target vertex q (field value 1).
     * @param field The computed harmonic field.
     * @param numSamplesK The umber of iso-curves to sample for the descriptor.
     * @param dist_p Precomputed geodesic distances from vertex p.
     * @param dist_q Precomputed geodesic distances from vertex q.
     * @return An Eigen::VectorXd of size numSamplesK containing the average geodesic distances for sampled iso-curves.
    */
   Eigen::VectorXd computeDDescriptor(int vertex_p_idx, int vertex_q_idx, const Eigen::VectorXd& field, int numSamplesK, const float* dist_p, const float* dist_q);   /**
    * Computes the Path Intrinsic Symmetry (PIS) score between two vertices p and q.
    * @param R_pq R descriptor for the harmonic field f_pq.
    * @param D_pq D descriptor for the harmonic field f_pq.
    * @param R_qp R descriptor for the harmonic field f_qp.
    * @param D_qp D descriptor for the harmonic field f_qp.
    * @return The scalar PIS score. Higher values indicate better symmetry.
    */
   double computePIS(const Eigen::VectorXd& R_pq, const Eigen::VectorXd& D_pq, const Eigen::VectorXd& R_qp, const Eigen::VectorXd& D_qp);

    /**
     * Extract isocurve segments from a harmonic field
     * @param field The harmonic field values
     * @param isoValue The isocurve level (0 to 1)
     * @return Vector of line segments representing the isocurve
     */
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> extractIsoCurveSegments(const Eigen::VectorXd& field, double isoValue);

    // ========== CORE SEGMENTATION FUNCTIONS (Primary Implementation) ==========    /**
     // Initialize LibIGL matrices and mesh data
     // Should be called once after mesh loading

    bool initializeLibIGL();

    /**
     * Compute harmonic coordinates using LibIGL
     * Supports multiple boundary constraints
     * @param boundary_vertices Vector of boundary vertex indices
     * @param boundary_values Vector of values at boundary vertices
     * @return Harmonic field satisfying boundary conditions
     */
    Eigen::VectorXd computeHarmonicCoordinatesLibIGL(
        const std::vector<int>& boundary_vertices,
        const std::vector<double>& boundary_values
    );

    /**
     * Enhanced R descriptor computation using LibIGL heat geodesics
     * More accurate and faster than the original implementation
     * @param field The computed harmonic field
     * @param numSamplesK The number of iso-curves to sample
     * @return R descriptor vector
     */
    Eigen::VectorXd computeRDescriptorLibIGL(const Eigen::VectorXd& field, int numSamplesK);

    /**
     * Enhanced D descriptor computation using LibIGL heat geodesics
     * Much faster and more accurate than Dijkstra-based computation
     * @param vertex_p_idx Index of source vertex p
     * @param vertex_q_idx Index of target vertex q
     * @param field The computed harmonic field
     * @param numSamplesK Number of iso-curves to sample
     * @return D descriptor vector
     */
    Eigen::VectorXd computeDDescriptorLibIGL(
        int vertex_p_idx, int vertex_q_idx,
        const Eigen::VectorXd& field, int numSamplesK
    );

    /**
     * Get mesh data in LibIGL format
     */
    const Eigen::MatrixXd& getVertexMatrix() const { return V; }
    const Eigen::MatrixXi& getFaceMatrix() const { return F; }

    /**
     * Check if LibIGL data is properly initialized
     */
    bool isLibIGLInitialized() const { return meshDataComputed; }

    /**
     * Use LibIGL heat geodesics for enhanced distance computation
     * @param source_vertex Source vertex index
     * @return Heat geodesic distances to all vertices
     */
    Eigen::VectorXd computeHeatGeodesicDistances(int source_vertex);

};

/**
 * Helper function to test the pairwise harmonic computation
 * @param mesh Pointer to the mesh
 * @param vertex_p_idx Index of the first vertex
 * @param vertex_q_idx Index of the second vertex
 * @param painter Pointer to the Painter class for visualization
 * @return SoSeparator containing the visualization
 */
SoSeparator* testPairwiseHarmonic(Mesh* mesh, int vertex_p_idx, int vertex_q_idx, Painter* painter);