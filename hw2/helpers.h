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
    
    // Flag indicating if the Laplacian has been computed
    bool laplacianComputed;

    // Helper methods
    double calculateCotangent(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
    double calculateAngle(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
    double computeTriangleArea(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3);
    double computeVoronoiArea(int vertexIdx);
    
public:
    /**
     * Constructor
     * @param m Pointer to the mesh
     */
    PairwiseHarmonics(Mesh* m);
    
    /**
     * Compute the cotangent Laplacian matrix for the mesh
     * This implements the cotangent formulation of the Laplace-Beltrami operator
     * @return True if computation was successful
     */
    bool computeCotangentLaplacian();
    
    /**
     * Compute the pairwise harmonic function between two vertices
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
     * @return An Eigen::VectorXd of size numSamplesK containing the average geodesic distances for sampled iso-curves.
    */
   Eigen::VectorXd computeDDescriptor(int vertex_p_idx, int vertex_q_idx, const Eigen::VectorXd& field, int numSamplesK);

   /**
    * Computes the Path Intrinsic Symmetry (PIS) score between two vertices p and q.
    * Requires the R and D descriptors computed for both (p,q) and (q,p) pairs.
    * @param R_pq R descriptor for the harmonic field f_pq.
    * @param D_pq D descriptor for the harmonic field f_pq.
    * @param R_qp R descriptor for the harmonic field f_qp.
    * @param D_qp D descriptor for the harmonic field f_qp.
    * @return The scalar PIS score. Higher values indicate better symmetry.
    */
   double computePIS(const Eigen::VectorXd& R_pq, const Eigen::VectorXd& D_pq, const Eigen::VectorXd& R_qp, const Eigen::VectorXd& D_qp);

    /**
     * Extracts the line segments forming the iso-curve for a given value.
     * @param field The scalar field (e.g., harmonic function) defined on vertices.
     * @param isoValue The value for which to extract the iso-curve.
     * @return A vector of pairs, where each pair represents the start and end
     *         3D coordinates of an iso-curve segment.
     */
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> extractIsoCurveSegments(const Eigen::VectorXd& field, double isoValue);

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