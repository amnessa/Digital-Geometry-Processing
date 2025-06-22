#pragma once

#include <vector>
#include "Mesh.h"

// Forward declarations for Coin3D types
class SoSeparator;
class SoWinExaminerViewer;
class PairwiseHarmonics;

/**
 * Visualization Utilities Module
 *
 * This module contains functions for testing individual components of the segmentation
 * pipeline and general visualization utilities.
 */

class VisualizationUtils {
public:
    /**
     * Clear visualization by removing all children from the scene graph root
     * @param root The scene graph root
     */
    static void clearVisualization(SoSeparator* root);

    /**
     * Test cotangent Laplacian computation with visualization
     * @param mesh The mesh structure
     * @param harmonics The pairwise harmonics object
     * @param root The scene graph root for visualization
     * @param viewer The 3D viewer for rendering
     */
    static void testCotangentLaplacian(
        Mesh* mesh,
        PairwiseHarmonics* harmonics,
        SoSeparator* root,
        SoWinExaminerViewer* viewer
    );

    /**
     * Test farthest point sampling with visualization
     * @param mesh The mesh structure
     * @param fps_samples Output: FPS sampling points
     * @param root The scene graph root for visualization
     * @param viewer The 3D viewer for rendering
     */
    static void testFarthestPointSampling(
        Mesh* mesh,
        std::vector<int>& fps_samples,
        SoSeparator* root,
        SoWinExaminerViewer* viewer
    );

    /**
     * Test pairwise harmonics computation with visualization
     * @param mesh The mesh structure
     * @param harmonics The pairwise harmonics object
     * @param fps_samples The FPS sampling points
     * @param root The scene graph root for visualization
     * @param viewer The 3D viewer for rendering
     */
    static void testPairwiseHarmonics(
        Mesh* mesh,
        PairwiseHarmonics* harmonics,
        const std::vector<int>& fps_samples,
        SoSeparator* root,
        SoWinExaminerViewer* viewer
    );

    /**
     * Test isocurve extraction with visualization
     * @param mesh The mesh structure
     * @param harmonics The pairwise harmonics object
     * @param fps_samples The FPS sampling points
     * @param root The scene graph root for visualization
     * @param viewer The 3D viewer for rendering
     */
    static void testIsoCurveExtraction(
        Mesh* mesh,
        PairwiseHarmonics* harmonics,
        const std::vector<int>& fps_samples,
        SoSeparator* root,
        SoWinExaminerViewer* viewer
    );

    /**
     * Test rigidity analysis with visualization
     * @param mesh The mesh structure
     * @param harmonics The pairwise harmonics object
     * @param fps_samples The FPS sampling points
     * @param root The scene graph root for visualization
     * @param viewer The 3D viewer for rendering
     */
    static void testRigidityAnalysis(
        Mesh* mesh,
        PairwiseHarmonics* harmonics,
        const std::vector<int>& fps_samples,
        SoSeparator* root,
        SoWinExaminerViewer* viewer    );

    /**
     * Create a colored sphere for visualization
     * @param position The 3D position
     * @param color RGB color (0-1 range)
     * @param radius Sphere radius
     * @return SoSeparator containing the sphere visualization
     */
    static SoSeparator* createColoredSphere(
        const Eigen::Vector3d& position,
        const Eigen::Vector3f& color,
        float radius
    );

    /**
     * Visualize mesh vertices with color mapping based on scalar field
     * @param mesh The mesh structure
     * @param field The scalar field for color mapping
     * @param root The scene graph root for visualization
     * @param title Title for the visualization
     */
    static void visualizeScalarField(
        const Mesh* mesh,
        const Eigen::VectorXd& field,
        SoSeparator* root,
        const std::string& title
    );

private:
};
