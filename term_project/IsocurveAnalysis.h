#pragma once

#include <vector>
#include <Eigen/Dense>
#include "Mesh.h"

// Forward declarations
class PairwiseHarmonics;
class SoSeparator;
class SoWinExaminerViewer;

/**
 * Isocurve Analysis Module
 *
 * This module contains functions for extracting isocurves from harmonic fields
 * using triangle marching and other geometric techniques for 3D mesh segmentation.
 */

class IsocurveAnalysis {
public:
    /**
     * Extract isocurves from a harmonic field using triangle marching algorithm
     * This is the improved implementation that uses linear interpolation on triangle edges
     * @param mesh The mesh structure containing vertices and triangles
     * @param harmonicField The harmonic field values for each vertex
     * @param isovalue The isovalue to extract (0.0 to 1.0)
     * @return Vector of isocurves, each isocurve is a vector of 3D points
     */
    static std::vector<std::vector<Eigen::Vector3d>> extractIsocurvesTriangleMarching(
        const Mesh* mesh,
        const Eigen::VectorXd& harmonicField,
        double isovalue
    );

    /**
     * Improved isocurve extraction that generates dense parallel "bracelets"
     * This function demonstrates the isocurve extraction with visualization
     * @param mesh The mesh structure
     * @param harmonics The pairwise harmonics object
     * @param fps_samples The FPS sampling points
     * @param root The scene graph root for visualization
     * @param viewer The 3D viewer for rendering
     * @param skeletal_segments Output: extracted skeletal segments
     */
    static void improvedIsoCurveExtraction(
        Mesh* mesh,
        PairwiseHarmonics* harmonics,
        const std::vector<int>& fps_samples,
        SoSeparator* root,
        SoWinExaminerViewer* viewer,
        std::vector<std::vector<Eigen::Vector3d>>& skeletal_segments
    );

private:
    /**
     * Helper function to interpolate between two vertices based on harmonic field values
     * @param v1 First vertex
     * @param v2 Second vertex
     * @param field1 Harmonic field value at v1
     * @param field2 Harmonic field value at v2
     * @param isovalue Target isovalue
     * @return Interpolated 3D point
     */
    static Eigen::Vector3d interpolateVertex(
        const Vertex* v1,
        const Vertex* v2,
        double field1,
        double field2,
        double isovalue
    );
};
