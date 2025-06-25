#pragma once

#include "Mesh.h"
#include "PairwiseHarmonicsSegmentation.h"
#include "Binding.h"
#include <vector>
#include <string>
#include <Eigen/Dense>

class AnimatedMeshPipeline {
public:
    AnimatedMeshPipeline(Mesh* referenceMesh, PairwiseHarmonicsSegmentation::SegmentationResult& staticSegmentationResult);

    void runStaticAnalysis();
    void processAnimationFrame(Mesh* frameMesh);

    const std::vector<Eigen::Vector3d>& getDeformedSkeleton() const { return m_deformedSkeleton; }

private:
    void bindSkeleton();
    void deformSkeleton(Mesh* frameMesh);

    Mesh* m_referenceMesh;
    PairwiseHarmonicsSegmentation::SegmentationResult& m_staticSegmentationResult;
    std::vector<BindingData> m_bindingData;
    std::vector<Eigen::Vector3d> m_deformedSkeleton;
};
