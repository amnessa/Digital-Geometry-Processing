#include "pch.h"
#include "AnimatedMeshPipeline.h"
#include "Mesh.h"
#include <iostream>
#include <limits>

AnimatedMeshPipeline::AnimatedMeshPipeline(Mesh* referenceMesh, PairwiseHarmonicsSegmentation::SegmentationResult& staticSegmentationResult)
    : m_referenceMesh(referenceMesh), m_staticSegmentationResult(staticSegmentationResult) {}

void AnimatedMeshPipeline::runStaticAnalysis() {
    std::cout << "Running static analysis..." << std::endl;
    bindSkeleton();
    std::cout << "Static analysis complete." << std::endl;
}

void AnimatedMeshPipeline::processAnimationFrame(Mesh* frameMesh) {
    deformSkeleton(frameMesh);
}

void AnimatedMeshPipeline::bindSkeleton() {
    std::cout << "  Binding skeleton to reference mesh..." << std::endl;
    // CORRECT: skeletonNodes is a vector of 3D points, not indices.
    const auto& skeletonNodes = m_staticSegmentationResult.skeletonNodes;
    m_bindingData.resize(skeletonNodes.size());

    for (size_t i = 0; i < skeletonNodes.size(); ++i) {
        // CORRECT: The joint is the 3D point directly from the skeleton data.
        const Eigen::Vector3d& joint = skeletonNodes[i];

        int closest_triangle = -1;
        double min_dist_sq = std::numeric_limits<double>::max();
        Eigen::Vector3d closest_point;

        // CORRECT: Iterate over 'tris' (triangles) which contains pointers.
        for (int j = 0; j < m_referenceMesh->tris.size(); ++j) {
            const auto* face = m_referenceMesh->tris[j];
            // CORRECT: Get vertex pointers from 'verts' and access their 'coords' member.
            const auto* v0_ptr = m_referenceMesh->verts[face->v1i];
            const auto* v1_ptr = m_referenceMesh->verts[face->v2i];
            const auto* v2_ptr = m_referenceMesh->verts[face->v3i];

            Eigen::Vector3d p = joint;
            Eigen::Vector3d a(v0_ptr->coords[0], v0_ptr->coords[1], v0_ptr->coords[2]);
            Eigen::Vector3d b(v1_ptr->coords[0], v1_ptr->coords[1], v1_ptr->coords[2]);
            Eigen::Vector3d c(v2_ptr->coords[0], v2_ptr->coords[1], v2_ptr->coords[2]);

            // Closest point on triangle logic (from Real-Time Collision Detection)
            Eigen::Vector3d ab = b - a;
            Eigen::Vector3d ac = c - a;
            Eigen::Vector3d ap = p - a;

            double d1 = ab.dot(ap);
            double d2 = ac.dot(ap);

            if (d1 <= 0.0 && d2 <= 0.0) {
                closest_point = a;
            } else {
                Eigen::Vector3d bp = p - b;
                double d3 = ab.dot(bp);
                double d4 = ac.dot(bp);
                if (d3 >= 0.0 && d4 <= d3) {
                    closest_point = b;
                } else {
                    double vc = d1 * d4 - d3 * d2;
                    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
                        double v = d1 / (d1 - d3);
                        closest_point = a + v * ab;
                    } else {
                        Eigen::Vector3d cp = p - c;
                        double d5 = ab.dot(cp);
                        double d6 = ac.dot(cp);
                        if (d6 >= 0.0 && d5 <= d6) {
                            closest_point = c;
                        } else {
                            double vb = d5 * d2 - d1 * d6;
                            if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
                                double w = d2 / (d2 - d6);
                                closest_point = a + w * ac;
                            } else {
                                double va = d3 * d6 - d5 * d4;
                                if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
                                    double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                                    closest_point = b + w * (c - b);
                                } else {
                                    double denom = 1.0 / (va + vb + vc);
                                    double v = vb * denom;
                                    double w = vc * denom;
                                    closest_point = a + v * ab + w * ac;
                                }
                            }
                        }
                    }
                }
            }

            double dist_sq = (joint - closest_point).squaredNorm();
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                closest_triangle = j;
            }
        }

        // CORRECT: Use -> for pointers and ->coords for coordinates
        const auto* face = m_referenceMesh->tris[closest_triangle];
        const auto* v0_ptr = m_referenceMesh->verts[face->v1i];
        const auto* v1_ptr = m_referenceMesh->verts[face->v2i];
        const auto* v2_ptr = m_referenceMesh->verts[face->v3i];

        Eigen::Vector3d p = joint;
        Eigen::Vector3d a(v0_ptr->coords[0], v0_ptr->coords[1], v0_ptr->coords[2]);
        Eigen::Vector3d b(v1_ptr->coords[0], v1_ptr->coords[1], v1_ptr->coords[2]);
        Eigen::Vector3d c(v2_ptr->coords[0], v2_ptr->coords[1], v2_ptr->coords[2]);

        // Barycentric coordinate calculation
        Eigen::Vector3d v01 = b - a, v02 = c - a, v0p = p - a;
        double d00 = v01.dot(v01);
        double d01 = v01.dot(v02);
        double d11 = v02.dot(v02);
        double d20 = v0p.dot(v01);
        double d21 = v0p.dot(v02);
        double denom = d00 * d11 - d01 * d01;
        double v = (d11 * d20 - d01 * d21) / denom;
        double w = (d00 * d21 - d01 * d20) / denom;
        double u = 1.0 - v - w;

        m_bindingData[i] = { (int)i, closest_triangle, {u, v, w} };
    }
    std::cout << "  Skeleton binding complete." << std::endl;
}

void AnimatedMeshPipeline::deformSkeleton(Mesh* frameMesh) {
    m_deformedSkeleton.clear();
    m_deformedSkeleton.resize(m_bindingData.size());

    for (const auto& bind : m_bindingData) {
        // CORRECT: Use -> for pointers
        const auto* face = frameMesh->tris[bind.triangle_index];
        // CORRECT: Use -> for pointers and ->coords for coordinates
        const auto* v0_ptr = frameMesh->verts[face->v1i];
        const auto* v1_ptr = frameMesh->verts[face->v2i];
        const auto* v2_ptr = frameMesh->verts[face->v3i];

        Eigen::Vector3d a(v0_ptr->coords[0], v0_ptr->coords[1], v0_ptr->coords[2]);
        Eigen::Vector3d b(v1_ptr->coords[0], v1_ptr->coords[1], v1_ptr->coords[2]);
        Eigen::Vector3d c(v2_ptr->coords[0], v2_ptr->coords[1], v2_ptr->coords[2]);

        // Deform the joint position using the stored barycentric coordinates
        Eigen::Vector3d new_pos = bind.barycentric_coords.x() * a +
                                bind.barycentric_coords.y() * b +
                                bind.barycentric_coords.z() * c;
        m_deformedSkeleton[bind.joint_index] = new_pos;
    }
}