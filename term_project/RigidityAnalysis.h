#pragma once

#include <vector>
#include <Eigen/Dense>
#include "Mesh.h"

// Forward declarations for Coin3D types
class SoSeparator;
class SoWinExaminerViewer;
class PairwiseHarmonics;

/**
 * Rigidity Analysis Module
 *
 * This module contains functions for analyzing the rigidity of skeletal segments
 * using PCA-based methods and threshold testing for 3D mesh segmentation.
 */

class RigidityAnalysis {
public:
    /**
     * Enhanced rigidity analysis that computes rigidity for ALL skeletal nodes
     * without applying a binary threshold, storing all rigidity scores.
     * Generates skeletal segments from MULTIPLE point pairs for better analysis.
     *
     * @param mesh The mesh structure
     * @param harmonics The pairwise harmonics object
     * @param fps_samples The FPS sampling points
     * @param skeletal_segments Output: generated skeletal segments
     * @param rigidity_scores Output: rigidity scores for all nodes
     * @param nonrigid_nodes Output: classification of non-rigid nodes
     */
    static void enhancedRigidityAnalysis(
        Mesh* mesh,
        PairwiseHarmonics* harmonics,
        const std::vector<int>& fps_samples,
        std::vector<std::vector<Eigen::Vector3d>>& skeletal_segments,
        std::vector<double>& rigidity_scores,
        std::vector<bool>& nonrigid_nodes
    );

    /**
     * Visualize ALL rigidity points with continuous color coding from green to red
     * This gives much better feedback than binary rigid/non-rigid classification
     *
     * @param mesh The mesh structure
     * @param skeletal_segments The skeletal segments to visualize
     * @param rigidity_scores The rigidity scores for color mapping
     * @param root The scene graph root for visualization
     * @param viewer The 3D viewer for rendering
     */
    static void visualizeAllRigidityPoints(
        const Mesh* mesh,
        const std::vector<std::vector<Eigen::Vector3d>>& skeletal_segments,
        const std::vector<double>& rigidity_scores,
        SoSeparator* root,
        SoWinExaminerViewer* viewer
    );    /**
     * Interactive rigidity threshold testing - allows user to experiment with different thresholds
     * and see how many junction points are detected at each threshold level
     *
     * @param skeletal_segments The skeletal segments to analyze
     * @param rigidity_scores The rigidity scores for threshold testing
     * @param root The scene graph root for visualization
     * @param viewer The 3D viewer for rendering
     */
    static void interactiveRigidityThresholdTesting(
        const std::vector<std::vector<Eigen::Vector3d>>& skeletal_segments,
        const std::vector<double>& rigidity_scores,
        SoSeparator* root,
        SoWinExaminerViewer* viewer
    );

    /**
     * Compute rigidity score for a single skeletal node using PCA analysis
     * Uses the corrected formula from the paper: ξ = λ₁ / (λ₁ + λ₂ + λ₃)
     *
     * @param nodes The skeletal segment nodes
     * @param nodeIdx The index of the node to analyze
     * @param windowSize The neighborhood size for PCA analysis
     * @return Rigidity score in range [0.33, 1.0]
     */
    static double computeNodeRigidity(
        const std::vector<Eigen::Vector3d>& nodes,
        int nodeIdx,
        int windowSize = 7
    );

private:

    /**
     * Clear visualization by removing all children from root
     * @param root The scene graph root
     */
    static void clearVisualization(SoSeparator* root);

    /**
     * Create a colored sphere for visualization     * @param position The 3D position
     * @param color RGB color (0-1 range)
     * @param radius Sphere radius
     * @return SoSeparator containing the sphere visualization
     */
    static SoSeparator* createColoredSphere(
        const Eigen::Vector3d& position,
        const Eigen::Vector3f& color,
        float radius
    );

private:
    /**
     * Find critical points of harmonic field for skeletal extraction
     * @param mesh The mesh structure
     * @param harmonicField The harmonic field values
     * @return Vector of critical point positions
     */
    static std::vector<Eigen::Vector3d> findHarmonicCriticalPoints(
        const Mesh* mesh,
        const Eigen::VectorXd& harmonicField
    );

    /**
     * Compute medial axis point from isocurve by projecting toward volume center
     * @param mesh The mesh structure
     * @param curve The isocurve points
     * @return Medial axis approximation point
     */
    static Eigen::Vector3d computeMedialAxisPoint(
        const Mesh* mesh,
        const std::vector<Eigen::Vector3d>& curve
    );
};
