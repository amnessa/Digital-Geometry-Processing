#include "pch.h"
#include "PairwiseHarmonicsSegmentation.h"
#include "Painter.h"
#include <iostream>
#include <algorithm>
#include <set>
#include <queue>

// Coin3D headers
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedLineSet.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>

using namespace std;

PairwiseHarmonicsSegmentation::PairwiseHarmonicsSegmentation(Mesh* m, PairwiseHarmonics* h)
    : mesh(m), harmonics(h) {
    // Initialize with default parameters
}

PairwiseHarmonicsSegmentation::SegmentationResult PairwiseHarmonicsSegmentation::performSegmentation() {
    SegmentationResult result;
    result.success = false;

    if (!mesh || !harmonics) {
        result.errorMessage = "Invalid mesh or harmonics object";
        return result;
    }

    cout << "\n=== PAIRWISE HARMONICS SEGMENTATION ===" << endl;
    cout << "Implementation based on 'Pairwise Harmonics for Shape Analysis'" << endl;
    cout << "by Zheng et al., IEEE TVCG 2013" << endl;
    cout << "\nParameters:" << endl;
    cout << "  - FPS samples: " << params.numFPSSamples << endl;
    cout << "  - Isocurves per field (K): " << params.numIsocurves << endl;
    cout << "  - Rigidity threshold: " << params.rigidityThreshold << endl;

    try {
        // =============================================================================
        // STAGE 1: Initial Point Sampling
        // =============================================================================
        cout << "\n--- STAGE 1: FARTHEST POINT SAMPLING ---" << endl;
        result.fpsPoints = computeFPSSamples();
        cout << "Generated " << result.fpsPoints.size() << " FPS samples" << endl;

        // =============================================================================
        // STAGE 2: Skeletal Segment Generation
        // =============================================================================
        cout << "\n--- STAGE 2: SKELETAL SEGMENT GENERATION ---" << endl;
        vector<SkeletalSegment> rawSegments = generateSkeletalSegments(result.fpsPoints);
        cout << "Generated " << rawSegments.size() << " raw skeletal segments" << endl;

        // =============================================================================
        // STAGE 3: Rigidity Analysis and Partial Skeleton Construction
        // =============================================================================
        cout << "\n--- STAGE 3: RIGIDITY ANALYSIS ---" << endl;
        computeRigidityAnalysis(rawSegments);
        result.partialSkeleton = selectPartialSkeleton(rawSegments);
        cout << "Selected " << result.partialSkeleton.size() << " segments for partial skeleton" << endl;

        // =============================================================================
        // STAGE 4: Initial Segmentation
        // =============================================================================
        cout << "\n--- STAGE 4: MESH SEGMENTATION ---" << endl;
        result.meshComponents = createInitialSegmentation(result.partialSkeleton);
        cout << "Created " << result.meshComponents.size() << " mesh components" << endl;

        // =============================================================================
        // STAGE 5: Skeleton Completion and Refinement
        // =============================================================================
        cout << "\n--- STAGE 5: SKELETON COMPLETION ---" << endl;
        completeSkeleton(result.meshComponents, result.skeletonNodes);
        refineSegmentation(result.meshComponents, result.skeletonNodes);
        cout << "Completed skeleton with " << result.skeletonNodes.size() << " nodes" << endl;

        result.success = true;
        cout << "\n=== SEGMENTATION COMPLETE ===" << endl;

    } catch (const exception& e) {
        result.errorMessage = string("Segmentation failed: ") + e.what();
        cout << "Error: " << result.errorMessage << endl;
    }

    return result;
}

vector<int> PairwiseHarmonicsSegmentation::computeFPSSamples() {
    return mesh->farthestPointSampling(params.numFPSSamples);
}

vector<PairwiseHarmonicsSegmentation::SkeletalSegment>
PairwiseHarmonicsSegmentation::generateSkeletalSegments(const vector<int>& fpsPoints) {

    vector<SkeletalSegment> segments;
    int totalPairs = fpsPoints.size() * (fpsPoints.size() - 1) / 2;
    int processedPairs = 0;

    cout << "Computing pairwise harmonic fields for " << totalPairs << " point pairs..." << endl;

    for (int i = 0; i < fpsPoints.size(); i++) {
        for (int j = i + 1; j < fpsPoints.size(); j++) {
            processedPairs++;            // Reduce verbose output - show only sample of progress
            if (processedPairs % 20 == 0 || processedPairs <= 5 || processedPairs >= totalPairs - 2) {
                cout << "  Pair " << processedPairs << "/" << totalPairs
                     << " (vertices " << fpsPoints[i] << ", " << fpsPoints[j] << ")" << endl;
            }

            // Compute harmonic field between FPS points
            Eigen::VectorXd harmonicField = harmonics->computePairwiseHarmonic(fpsPoints[i], fpsPoints[j]);

            if (harmonicField.size() != mesh->verts.size()) {
                continue; // Skip failed computation
            }

            // Extract isocurve centroids to form skeletal nodes
            vector<Eigen::Vector3d> skeletalNodes = extractIsocurveCentroids(harmonicField, fpsPoints[i], fpsPoints[j]);

            // Create segment if we have enough nodes
            if (skeletalNodes.size() >= params.minSkeletalNodes) {
                SkeletalSegment segment;
                segment.nodes = skeletalNodes;
                segment.sourceIdx = fpsPoints[i];
                segment.targetIdx = fpsPoints[j];

                // Compute segment length
                segment.length = 0;
                for (int k = 1; k < skeletalNodes.size(); k++) {
                    segment.length += (skeletalNodes[k] - skeletalNodes[k-1]).norm();
                }

                // Filter based on length relative to direct endpoint distance
                Eigen::Vector3d endpointDist(
                    mesh->verts[fpsPoints[i]]->coords[0] - mesh->verts[fpsPoints[j]]->coords[0],
                    mesh->verts[fpsPoints[i]]->coords[1] - mesh->verts[fpsPoints[j]]->coords[1],
                    mesh->verts[fpsPoints[i]]->coords[2] - mesh->verts[fpsPoints[j]]->coords[2]
                );
                double endpointDistance = endpointDist.norm();

                if (segment.length > endpointDistance * params.minSegmentLength) {
                    segments.push_back(segment);
                }
            }
        }
    }

    return segments;
}

vector<Eigen::Vector3d> PairwiseHarmonicsSegmentation::extractIsocurveCentroids(
    const Eigen::VectorXd& harmonicField, int sourceIdx, int targetIdx) {

    vector<Eigen::Vector3d> centroids;

    // Extract isocurves at regular intervals
    for (int k = 1; k < params.numIsocurves; k++) {
        double isoValue = double(k) / params.numIsocurves;

        // Find vertices near this isocurve level
        vector<int> braceletVertices;
        for (int v = 0; v < mesh->verts.size(); v++) {
            if (abs(harmonicField(v) - isoValue) < params.isoTolerance) {
                braceletVertices.push_back(v);
            }
        }

        // Compute centroid if we have enough vertices
        if (braceletVertices.size() >= 3) {
            Eigen::Vector3d centroid(0, 0, 0);
            for (int v : braceletVertices) {
                centroid += Eigen::Vector3d(
                    mesh->verts[v]->coords[0],
                    mesh->verts[v]->coords[1],
                    mesh->verts[v]->coords[2]
                );
            }
            centroid /= braceletVertices.size();
            centroids.push_back(centroid);
        }
    }

    return centroids;
}

void PairwiseHarmonicsSegmentation::computeRigidityAnalysis(vector<SkeletalSegment>& segments) {
    cout << "Computing rigidity for " << segments.size() << " segments..." << endl;

    for (auto& segment : segments) {
        segment.nodeRigidities.clear();

        // Compute rigidity for each node using PCA
        for (int nodeIdx = 0; nodeIdx < segment.nodes.size(); nodeIdx++) {
            // Define local neighborhood
            vector<Eigen::Vector3d> neighborhood;
            int windowSize = min(5, (int)segment.nodes.size());
            int start = max(0, nodeIdx - windowSize/2);
            int end = min((int)segment.nodes.size(), start + windowSize);

            for (int k = start; k < end; k++) {
                neighborhood.push_back(segment.nodes[k]);
            }

            double rigidity = computeNodeRigidity(neighborhood);
            segment.nodeRigidities.push_back(rigidity);
        }

        // Compute average rigidity for the segment
        double totalRigidity = 0;
        for (double r : segment.nodeRigidities) {
            totalRigidity += r;
        }
        segment.avgRigidity = totalRigidity / segment.nodeRigidities.size();
        segment.quality = segment.avgRigidity * segment.length;
    }
}

double PairwiseHarmonicsSegmentation::computeNodeRigidity(const vector<Eigen::Vector3d>& neighborhood) {
    if (neighborhood.size() < 3) return 0.5; // Default value

    // Compute covariance matrix
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();

    for (const auto& point : neighborhood) {
        mean += point;
    }
    mean /= neighborhood.size();

    for (const auto& point : neighborhood) {
        Eigen::Vector3d diff = point - mean;
        covariance += diff * diff.transpose();
    }
    covariance /= (neighborhood.size() - 1);

    // Compute eigenvalues for PCA-based rigidity
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    Eigen::Vector3d eigenvalues = solver.eigenvalues();

    // Sort in descending order
    sort(eigenvalues.data(), eigenvalues.data() + 3, greater<double>());

    double lambda1 = eigenvalues(0);
    double lambda2 = eigenvalues(1);
    double lambda3 = eigenvalues(2);
    double sumLambda = lambda1 + lambda2 + lambda3;

    // Rigidity formula: ξ = λ₁ / (λ₁ + λ₂ + λ₃)
    if (sumLambda > 1e-10) {
        return lambda1 / sumLambda;
    }
    return 0.33; // Default for degenerate case
}

vector<PairwiseHarmonicsSegmentation::SkeletalSegment>
PairwiseHarmonicsSegmentation::selectPartialSkeleton(const vector<SkeletalSegment>& segments) {

    // Sort segments by quality (rigidity * length)
    vector<SkeletalSegment> sortedSegments = segments;
    sort(sortedSegments.begin(), sortedSegments.end(),
         [](const SkeletalSegment& a, const SkeletalSegment& b) {
             return a.quality > b.quality;
         });

    // Greedy selection to build connected skeleton
    vector<SkeletalSegment> skeleton;
    set<int> usedFPSPoints;

    // Start with highest quality segment
    if (!sortedSegments.empty()) {
        skeleton.push_back(sortedSegments[0]);
        usedFPSPoints.insert(sortedSegments[0].sourceIdx);
        usedFPSPoints.insert(sortedSegments[0].targetIdx);

        // Add connecting segments
        for (int iter = 0; iter < 10 && skeleton.size() < params.maxSkeletonSegments; iter++) {
            double bestQuality = 0;
            int bestIdx = -1;

            for (int i = 1; i < sortedSegments.size(); i++) {
                const auto& seg = sortedSegments[i];

                // Check if connects to existing skeleton
                bool connects = (usedFPSPoints.count(seg.sourceIdx) > 0) ||
                              (usedFPSPoints.count(seg.targetIdx) > 0);

                // Avoid cycles
                bool createsCycle = (usedFPSPoints.count(seg.sourceIdx) > 0) &&
                                  (usedFPSPoints.count(seg.targetIdx) > 0);

                if (connects && !createsCycle && seg.quality > bestQuality) {
                    bestQuality = seg.quality;
                    bestIdx = i;
                }
            }

            if (bestIdx >= 0) {
                skeleton.push_back(sortedSegments[bestIdx]);
                usedFPSPoints.insert(sortedSegments[bestIdx].sourceIdx);
                usedFPSPoints.insert(sortedSegments[bestIdx].targetIdx);
                sortedSegments.erase(sortedSegments.begin() + bestIdx);
            } else {
                break;
            }
        }
    }

    return skeleton;
}

vector<PairwiseHarmonicsSegmentation::MeshComponent>
PairwiseHarmonicsSegmentation::createInitialSegmentation(const vector<SkeletalSegment>& skeleton) {

    vector<MeshComponent> components;
    vector<int> vertexToComponent(mesh->verts.size(), -1);

    // Create components based on harmonic fields of skeleton segments
    for (int segIdx = 0; segIdx < skeleton.size(); segIdx++) {
        const auto& segment = skeleton[segIdx];

        Eigen::VectorXd harmonicField = harmonics->computePairwiseHarmonic(segment.sourceIdx, segment.targetIdx);

        if (harmonicField.size() != mesh->verts.size()) continue;

        MeshComponent component;
        component.skeletonSegmentId = segIdx;

        // Assign vertices based on harmonic field values
        double minVal = harmonicField.minCoeff();
        double maxVal = harmonicField.maxCoeff();
        double range = maxVal - minVal;

        if (range > 1e-6) {
            for (int vIdx = 0; vIdx < mesh->verts.size(); vIdx++) {
                double normalizedVal = (harmonicField(vIdx) - minVal) / range;

                // Assign vertices in the middle range to this component
                if (normalizedVal >= 0.1 && normalizedVal <= 0.9 && vertexToComponent[vIdx] == -1) {
                    component.vertexIndices.push_back(vIdx);
                    vertexToComponent[vIdx] = components.size();
                }
            }
        }

        if (!component.vertexIndices.empty()) {
            components.push_back(component);
        }
    }

    // Assign remaining vertices to nearest component
    for (int vIdx = 0; vIdx < mesh->verts.size(); vIdx++) {
        if (vertexToComponent[vIdx] == -1) {
            // Find nearest assigned vertex
            double minDist = DBL_MAX;
            int nearestComponent = 0;

            for (int otherIdx = 0; otherIdx < mesh->verts.size(); otherIdx++) {
                if (vertexToComponent[otherIdx] != -1) {
                    Eigen::Vector3d v1(mesh->verts[vIdx]->coords[0], mesh->verts[vIdx]->coords[1], mesh->verts[vIdx]->coords[2]);
                    Eigen::Vector3d v2(mesh->verts[otherIdx]->coords[0], mesh->verts[otherIdx]->coords[1], mesh->verts[otherIdx]->coords[2]);
                    double dist = (v1 - v2).norm();
                    if (dist < minDist) {
                        minDist = dist;
                        nearestComponent = vertexToComponent[otherIdx];
                    }
                }
            }

            if (nearestComponent < components.size()) {
                components[nearestComponent].vertexIndices.push_back(vIdx);
                vertexToComponent[vIdx] = nearestComponent;
            }
        }
    }

    return components;
}

void PairwiseHarmonicsSegmentation::completeSkeleton(
    vector<MeshComponent>& components, vector<Eigen::Vector3d>& skeletonNodes) {

    // Compute component centers
    for (auto& component : components) {
        if (!component.vertexIndices.empty()) {
            Eigen::Vector3d center(0, 0, 0);
            for (int vIdx : component.vertexIndices) {
                center += Eigen::Vector3d(
                    mesh->verts[vIdx]->coords[0],
                    mesh->verts[vIdx]->coords[1],
                    mesh->verts[vIdx]->coords[2]
                );
            }
            center /= component.vertexIndices.size();
            component.center = center;
            skeletonNodes.push_back(center);
        }
    }
}

void PairwiseHarmonicsSegmentation::refineSegmentation(
    vector<MeshComponent>& components, const vector<Eigen::Vector3d>& skeleton) {
    // Future enhancement: refine segmentation based on completed skeleton
    // This could involve re-assigning vertices based on distance to skeleton nodes
}

void PairwiseHarmonicsSegmentation::visualizeResults(
    const SegmentationResult& result,
    SoSeparator* root,
    Painter* painter,
    SoWinExaminerViewer* viewer,
    Mesh* mesh) {

    if (!result.success) {
        cout << "Cannot visualize failed segmentation: " << result.errorMessage << endl;
        return;
    }

    // Visualize partial skeleton segments
    for (int i = 0; i < result.partialSkeleton.size(); i++) {
        const auto& segment = result.partialSkeleton[i];

        SoSeparator* segSep = new SoSeparator();
        SoMaterial* mat = new SoMaterial();

        // Color by rigidity: red = high rigidity, blue = low rigidity
        float rigidity = static_cast<float>(segment.avgRigidity);
        mat->diffuseColor.setValue(rigidity, 0.2f, 1.0f - rigidity);
        segSep->addChild(mat);

        // Create simplified skeletal path
        SoCoordinate3* coords = new SoCoordinate3();
        coords->point.setNum(segment.nodes.size());
        for (int j = 0; j < segment.nodes.size(); j++) {
            coords->point.set1Value(j,
                segment.nodes[j][0],
                segment.nodes[j][1],
                segment.nodes[j][2]
            );
        }
        segSep->addChild(coords);

        SoIndexedLineSet* lineSet = new SoIndexedLineSet();
        for (int j = 0; j < segment.nodes.size() - 1; j++) {
            lineSet->coordIndex.set1Value(j * 3, j);
            lineSet->coordIndex.set1Value(j * 3 + 1, j + 1);
            lineSet->coordIndex.set1Value(j * 3 + 2, -1);
        }
        segSep->addChild(lineSet);
        root->addChild(segSep);    }

    // Visualize mesh components with different colors
    if (!result.meshComponents.empty()) {
        cout << "Adding mesh component visualization..." << endl;

        // Generate distinct colors for each component
        vector<Eigen::Vector3f> componentColors;
        for (int i = 0; i < result.meshComponents.size(); i++) {
            float hue = float(i) / float(result.meshComponents.size()) * 360.0f;
            // Convert HSV to RGB for distinct colors
            float r, g, b;
            if (hue < 60) {
                r = 1.0f; g = hue / 60.0f; b = 0.0f;
            } else if (hue < 120) {
                r = (120 - hue) / 60.0f; g = 1.0f; b = 0.0f;
            } else if (hue < 180) {
                r = 0.0f; g = 1.0f; b = (hue - 120) / 60.0f;
            } else if (hue < 240) {
                r = 0.0f; g = (240 - hue) / 60.0f; b = 1.0f;
            } else if (hue < 300) {
                r = (hue - 240) / 60.0f; g = 0.0f; b = 1.0f;
            } else {
                r = 1.0f; g = 0.0f; b = (360 - hue) / 60.0f;
            }
            componentColors.push_back(Eigen::Vector3f(r * 0.8f, g * 0.8f, b * 0.8f)); // Slightly dimmed
        }

        // Visualize each component with its color
        for (int compIdx = 0; compIdx < result.meshComponents.size(); compIdx++) {
            const auto& component = result.meshComponents[compIdx];
            if (component.vertexIndices.empty()) continue;

            SoSeparator* compSep = new SoSeparator();
            SoMaterial* compMat = new SoMaterial();
            const Eigen::Vector3f& color = componentColors[compIdx];
            compMat->diffuseColor.setValue(color[0], color[1], color[2]);
            compSep->addChild(compMat);

            // Create point set for component vertices
            SoCoordinate3* compCoords = new SoCoordinate3();
            compCoords->point.setNum(component.vertexIndices.size());            for (int i = 0; i < component.vertexIndices.size(); i++) {
                int vIdx = component.vertexIndices[i];
                if (vIdx >= 0 && vIdx < mesh->verts.size()) {
                    Vertex* v = mesh->verts[vIdx];
                    compCoords->point.set1Value(i, v->coords[0], v->coords[1], v->coords[2]);
                }
            }
            compSep->addChild(compCoords);

            SoPointSet* pointSet = new SoPointSet();
            pointSet->numPoints = component.vertexIndices.size();
            compSep->addChild(pointSet);

            root->addChild(compSep);
        }
    }

    // Visualize component centers
    SoSeparator* centersSep = new SoSeparator();
    SoMaterial* centerMat = new SoMaterial();
    centerMat->diffuseColor.setValue(1.0f, 0.0f, 1.0f); // Magenta
    centersSep->addChild(centerMat);

    for (const auto& center : result.skeletonNodes) {
        // Create sphere at each center (simplified visualization)
        SoSeparator* sphereSep = new SoSeparator();
        // Add sphere creation code here...
        centersSep->addChild(sphereSep);
    }
    root->addChild(centersSep);

    viewer->scheduleRedraw();
    viewer->viewAll();

    cout << "\nVisualization complete:" << endl;
    cout << "  - " << result.partialSkeleton.size() << " skeletal segments (colored by rigidity)" << endl;
    cout << "  - " << result.meshComponents.size() << " mesh components" << endl;
    cout << "  - " << result.skeletonNodes.size() << " skeleton nodes (magenta points)" << endl;
}
