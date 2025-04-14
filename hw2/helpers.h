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