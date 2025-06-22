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
    : mesh(m), harmonics(h), rigidityComputed(false) {
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

    try {        // =============================================================================
        // STAGE 1: Initial Point Sampling and Global Rigidity Analysis
        // =============================================================================
        cout << "\n--- STAGE 1: FARTHEST POINT SAMPLING ---" << endl;
        result.fpsPoints = computeFPSSamples();
        cout << "Generated " << result.fpsPoints.size() << " FPS samples" << endl;

        cout << "\n--- STAGE 1.5: GLOBAL RIGIDITY ANALYSIS ---" << endl;
        computeGlobalVertexRigidity();  // Compute global rigidity map
        cout << "Global rigidity analysis complete" << endl;

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
        cout << "Selected " << result.partialSkeleton.size() << " segments for partial skeleton" << endl;        // =============================================================================
        // STAGE 4: Initial Segmentation (IMPROVED APPROACH)
        // =============================================================================
        cout << "\n--- STAGE 4: MESH SEGMENTATION ---" << endl;
        result.meshComponents = createImprovedSegmentation(result.partialSkeleton);
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
            }            // Compute harmonic field between FPS points
            Eigen::VectorXd harmonicField = harmonics->computePairwiseHarmonic(fpsPoints[i], fpsPoints[j]);

            if (harmonicField.size() != mesh->verts.size()) {
                continue; // Skip failed computation
            }

            // Extract segments with rigidity-based cutting (NEW APPROACH)
            vector<SkeletalSegment> pairSegments = extractRigidityBasedSegments(harmonicField, fpsPoints[i], fpsPoints[j]);

            // Add all valid segments from this point pair
            for (const auto& segment : pairSegments) {
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

    // CRITICAL FIX: Always include the actual FPS endpoints first!
    // This ensures skeletal paths reach the limb extremities

    // Add source FPS point (should correspond to harmonic value ~0.0)
    Eigen::Vector3d sourcePoint(mesh->verts[sourceIdx]->coords[0],
                               mesh->verts[sourceIdx]->coords[1],
                               mesh->verts[sourceIdx]->coords[2]);
    centroids.push_back(sourcePoint);

    // Extract isocurves at regular intervals (excluding endpoints k=0 and k=numIsocurves)
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

    // Add target FPS point (should correspond to harmonic value ~1.0)
    Eigen::Vector3d targetPoint(mesh->verts[targetIdx]->coords[0],
                               mesh->verts[targetIdx]->coords[1],
                               mesh->verts[targetIdx]->coords[2]);
    centroids.push_back(targetPoint);

    cout << "    Extracted " << centroids.size() << " centroids including FPS endpoints" << endl;
    cout << "      Source FPS " << sourceIdx << " at: (" << sourcePoint[0] << ", " << sourcePoint[1] << ", " << sourcePoint[2] << ")" << endl;
    cout << "      Target FPS " << targetIdx << " at: (" << targetPoint[0] << ", " << targetPoint[1] << ", " << targetPoint[2] << ")" << endl;

    return centroids;
}

/**
 * Extract skeletal segments with rigidity-based cutting
 * IMPROVED VERSION: Uses global mesh rigidity to identify true junction areas
 */
vector<PairwiseHarmonicsSegmentation::SkeletalSegment>
PairwiseHarmonicsSegmentation::extractRigidityBasedSegments(
    const Eigen::VectorXd& harmonicField, int sourceIdx, int targetIdx) {

    vector<SkeletalSegment> segments;

    // Extract isocurve centroids first
    vector<Eigen::Vector3d> allCentroids = extractIsocurveCentroids(harmonicField, sourceIdx, targetIdx);

    if (allCentroids.size() < 3) {
        return segments; // Need at least 3 points for rigidity analysis
    }

    // Ensure global rigidity is computed
    if (!rigidityComputed) {
        computeGlobalVertexRigidity();
    }

    // IMPROVED APPROACH: Map skeletal centroids to mesh vertices and use global rigidity
    vector<double> rigidities;

    cout << "    Mapping " << allCentroids.size() << " centroids to mesh rigidity..." << endl;    for (int i = 0; i < allCentroids.size(); i++) {
        const Eigen::Vector3d& centroid = allCentroids[i];
        double centroidRigidity;

        // Special handling for FPS endpoints (first and last centroids)
        if (i == 0) {
            // First centroid is source FPS point
            centroidRigidity = globalVertexRigidity[sourceIdx];
            cout << "      Centroid " << i << " (SOURCE FPS " << sourceIdx << "): rigidity = " << centroidRigidity << endl;
        }
        else if (i == allCentroids.size() - 1) {
            // Last centroid is target FPS point
            centroidRigidity = globalVertexRigidity[targetIdx];
            cout << "      Centroid " << i << " (TARGET FPS " << targetIdx << "): rigidity = " << centroidRigidity << endl;
        }
        else {
            // Interior centroid - find closest mesh vertex
            double minDistance = 1e10;
            int closestVertex = 0;

            for (int v = 0; v < mesh->verts.size(); v++) {
                Eigen::Vector3d vertexPos(mesh->verts[v]->coords[0],
                                         mesh->verts[v]->coords[1],
                                         mesh->verts[v]->coords[2]);

                double distance = (vertexPos - centroid).norm();
                if (distance < minDistance) {
                    minDistance = distance;
                    closestVertex = v;
                }
            }

            // Use the global rigidity of the closest mesh vertex
            centroidRigidity = globalVertexRigidity[closestVertex];

            // Also sample nearby vertices for more robust analysis
            vector<double> nearbyRigidities;
            double searchRadius = 0.05; // Small radius for nearby sampling

            for (int v = 0; v < mesh->verts.size(); v++) {
                Eigen::Vector3d vertexPos(mesh->verts[v]->coords[0],
                                         mesh->verts[v]->coords[1],
                                         mesh->verts[v]->coords[2]);

                double distance = (vertexPos - centroid).norm();
                if (distance < searchRadius) {
                    nearbyRigidities.push_back(globalVertexRigidity[v]);
                }
            }

            // Use average of nearby rigidities if we have enough samples
            if (nearbyRigidities.size() >= 3) {
                double avgRigidity = 0;
                for (double r : nearbyRigidities) avgRigidity += r;
                centroidRigidity = avgRigidity / nearbyRigidities.size();
            }

            if (i % 10 == 0) { // Show progress every 10th point
                cout << "      Centroid " << i << ": rigidity = " << centroidRigidity
                     << " (from vertex " << closestVertex << ", " << nearbyRigidities.size() << " nearby)" << endl;
            }
        }

        rigidities.push_back(centroidRigidity);
    }

    // Find statistics to adaptively set thresholds
    double minRig = *min_element(rigidities.begin(), rigidities.end());
    double maxRig = *max_element(rigidities.begin(), rigidities.end());
    double meanRig = 0;
    for (double r : rigidities) meanRig += r;
    meanRig /= rigidities.size();    cout << "    Rigidity statistics: min=" << minRig << ", max=" << maxRig << ", mean=" << meanRig << endl;

    // REVISED APPROACH: Based on research papers and data analysis
    // Skeletal segments should be preserved as complete limb paths
    // Only cut at true anatomical junctions (very low rigidity OR major transitions)

    // Analysis of your data shows:
    // - Global rigidity: [0.40569, 0.999857]
    // - Skeletal paths: ~0.55-0.83 (these ARE the limb centers, should be preserved!)
    // - True junctions: likely < 0.6 (main body/torso connections)

    double junctionThreshold = 0.6;    // Based on global analysis - true body junctions
    double transitionThreshold = 0.65; // Moderate transitions within limbs

    // Override with global context if available
    if (minRig > 0.5) {
        // High-rigidity skeletal path - very conservative cutting
        junctionThreshold = minRig + 0.1 * (maxRig - minRig);  // Only bottom 10%
        transitionThreshold = minRig + 0.2 * (maxRig - minRig);
        cout << "    High-rigidity skeletal path detected - using conservative junction detection" << endl;
    }

    cout << "    Junction detection: junction=" << junctionThreshold << ", transition=" << transitionThreshold << endl;// Cut segments using adaptive thresholding based on actual data distribution
    cout << "    Cutting segments at low-rigidity junctions using adaptive thresholds..." << endl;

    vector<int> cutPoints; // Indices where to cut
    cutPoints.push_back(0); // Always start with first point    // Find junction points using conservative thresholding
    // Goal: Preserve complete limb segments, only cut at true anatomical junctions
    // IMPORTANT: Never cut near endpoints (first/last 3 points) to preserve limb extremities
    for (int i = 3; i < rigidities.size() - 3; i++) { // Skip first and last 3 points
        bool isJunction = false;

        // Primary check: Very low rigidity (definite anatomical junction like torso connection)
        if (rigidities[i] < junctionThreshold) {
            isJunction = true;
            cout << "      ANATOMICAL junction at centroid " << i << " (rigidity: " << rigidities[i] << " < " << junctionThreshold << ")" << endl;
        }
        // Secondary check: Significant local minimum AND below transition threshold
        else if (rigidities[i] < transitionThreshold) {
            // Check if this is a very significant local minimum (not just noise)
            bool isSignificantMin = true;
            int windowSize = 6; // Larger window for more stability
            double minLocalValue = rigidities[i];

            // Check if this is actually the minimum in a substantial neighborhood
            for (int j = max(3, i - windowSize); j <= min((int)rigidities.size() - 4, i + windowSize); j++) {
                if (j != i && rigidities[j] <= minLocalValue + 0.03) // Very small tolerance
                    isSignificantMin = false;
            }

            // Additional check: must be substantially lower than neighbors
            double avgNeighbors = 0;
            int neighborCount = 0;
            for (int j = max(3, i - 3); j <= min((int)rigidities.size() - 4, i + 3); j++) {
                if (j != i) {
                    avgNeighbors += rigidities[j];
                    neighborCount++;
                }
            }
            if (neighborCount > 0) {
                avgNeighbors /= neighborCount;

                bool isSubstantialDrop = (avgNeighbors - rigidities[i]) > 0.05; // 5% drop required

                if (isSignificantMin && isSubstantialDrop) {
                    isJunction = true;
                    cout << "      Significant transition at centroid " << i << " (rigidity: " << rigidities[i]
                         << ", drop: " << (avgNeighbors - rigidities[i]) << ")" << endl;
                }
            }
        }if (isJunction) {
            cutPoints.push_back(i);
        }
    }
    cutPoints.push_back(allCentroids.size() - 1); // Always end with last point

    // If no junctions found, preserve the entire skeletal path as one segment
    if (cutPoints.size() == 2) { // Only start and end points
        cout << "    No significant junctions detected - preserving complete skeletal path" << endl;
    }

    cout << "    Found " << (cutPoints.size() - 1) << " segments from " << cutPoints.size() << " cut points" << endl;

    // Create segments between cut points
    for (int cutIdx = 0; cutIdx < cutPoints.size() - 1; cutIdx++) {
        int startIdx = cutPoints[cutIdx];
        int endIdx = cutPoints[cutIdx + 1];

        if (endIdx - startIdx >= 2) { // Need at least 2 points for a segment
            SkeletalSegment segment;

            // Copy nodes for this segment
            for (int i = startIdx; i <= endIdx; i++) {
                segment.nodes.push_back(allCentroids[i]);
                if (i < rigidities.size()) {
                    segment.nodeRigidities.push_back(rigidities[i]);
                }
            }

            segment.sourceIdx = sourceIdx;
            segment.targetIdx = targetIdx;

            // Compute segment length
            segment.length = 0;
            for (int i = 1; i < segment.nodes.size(); i++) {
                segment.length += (segment.nodes[i] - segment.nodes[i-1]).norm();
            }

            // Compute average rigidity
            double totalRigidity = 0;
            for (double r : segment.nodeRigidities) {
                totalRigidity += r;
            }
            segment.avgRigidity = totalRigidity / segment.nodeRigidities.size();
            segment.quality = segment.avgRigidity * segment.length;

            segments.push_back(segment);

            cout << "      Created segment " << segments.size() - 1 << ": "
                 << segment.nodes.size() << " nodes, length: " << segment.length
                 << ", avg rigidity: " << segment.avgRigidity << endl;
        }
    }

    return segments;
}

void PairwiseHarmonicsSegmentation::computeRigidityAnalysis(vector<SkeletalSegment>& segments) {
    cout << "Rigidity analysis complete for " << segments.size() << " segments" << endl;
    cout << "(Using global vertex rigidity computed in extractRigidityBasedSegments)" << endl;

    // Print summary statistics
    if (!segments.empty()) {
        double minAvgRig = 1.0, maxAvgRig = 0.0, totalAvgRig = 0.0;

        for (const auto& segment : segments) {
            if (segment.avgRigidity < minAvgRig) minAvgRig = segment.avgRigidity;
            if (segment.avgRigidity > maxAvgRig) maxAvgRig = segment.avgRigidity;
            totalAvgRig += segment.avgRigidity;
        }

        cout << "Segment rigidity summary:" << endl;
        cout << "  Min avg rigidity: " << minAvgRig << endl;
        cout << "  Max avg rigidity: " << maxAvgRig << endl;
        cout << "  Overall avg rigidity: " << (totalAvgRig / segments.size()) << endl;
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

    cout << "  Selecting diverse skeletal segments for anatomical coverage..." << endl;
    cout << "  Input: " << segments.size() << " candidate segments" << endl;

    // STRATEGY: Enforce spatial diversity from the start
    // Group segments by their source FPS point, then select the best from each group
    map<int, vector<SkeletalSegment>> segmentsBySource;

    for (const auto& seg : segments) {
        segmentsBySource[seg.sourceIdx].push_back(seg);
    }

    cout << "  Segments grouped by " << segmentsBySource.size() << " source FPS points" << endl;

    // Sort segments within each group by quality
    for (auto& group : segmentsBySource) {
        sort(group.second.begin(), group.second.end(),
             [](const SkeletalSegment& a, const SkeletalSegment& b) {
                 return a.quality > b.quality;
             });
    }

    // Phase 1: Select the single best segment from each source point
    // This ensures maximum spatial diversity
    vector<SkeletalSegment> diverseSegments;

    cout << "  Phase 1: Selecting best segment from each source point..." << endl;
    for (const auto& group : segmentsBySource) {
        if (!group.second.empty()) {
            const SkeletalSegment& bestFromGroup = group.second[0];
            diverseSegments.push_back(bestFromGroup);

            cout << "    From FPS " << group.first << ": selected "
                 << bestFromGroup.sourceIdx << " -> " << bestFromGroup.targetIdx
                 << " (length: " << bestFromGroup.length
                 << ", quality: " << bestFromGroup.quality << ")" << endl;
        }
    }

    // Sort the diverse segments by quality for final selection
    sort(diverseSegments.begin(), diverseSegments.end(),
         [](const SkeletalSegment& a, const SkeletalSegment& b) {
             return a.quality > b.quality;
         });

    // Phase 2: Select top segments up to the limit, ensuring no FPS point is heavily reused
    vector<SkeletalSegment> skeleton;
    set<int> usedSources;
    set<int> usedTargets;

    cout << "  Phase 2: Final selection with diversity constraints..." << endl;

    int maxSegments = min(params.maxSkeletonSegments, (int)diverseSegments.size());
    for (int i = 0; i < maxSegments; i++) {
        const auto& seg = diverseSegments[i];

        // Prefer segments that don't reuse FPS points
        bool reuseSource = usedSources.count(seg.sourceIdx) > 0;
        bool reuseTarget = usedTargets.count(seg.targetIdx) > 0;

        // Allow some reuse if we have fewer segments than desired, but prefer new points
        bool acceptSegment = true;
        if (skeleton.size() >= maxSegments / 2) { // Be stricter in later selections
            acceptSegment = !reuseSource && !reuseTarget;
        }

        if (acceptSegment) {
            skeleton.push_back(seg);
            usedSources.insert(seg.sourceIdx);
            usedTargets.insert(seg.targetIdx);

            cout << "    Selected segment " << skeleton.size() << ": "
                 << seg.sourceIdx << " -> " << seg.targetIdx
                 << " (length: " << seg.length << ", quality: " << seg.quality << ")" << endl;
        } else {
            cout << "    Skipped segment " << seg.sourceIdx << " -> " << seg.targetIdx
                 << " (FPS point reuse)" << endl;
        }

        if (skeleton.size() >= params.maxSkeletonSegments) break;
    }

    cout << "  Final selection: " << skeleton.size() << " diverse skeletal segments" << endl;
    cout << "  Used " << usedSources.size() << " unique source FPS points" << endl;
    cout << "  Used " << usedTargets.size() << " unique target FPS points" << endl;

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

/**
 * Create improved mesh segmentation that properly handles torso/junction areas
 * Based on the paper's approach: assign vertices to limb segments, then group remainder as torso
 */
vector<PairwiseHarmonicsSegmentation::MeshComponent>
PairwiseHarmonicsSegmentation::createImprovedSegmentation(const vector<SkeletalSegment>& skeleton) {

    vector<MeshComponent> components;
    vector<int> vertexToComponent(mesh->verts.size(), -1);
    vector<double> vertexConfidence(mesh->verts.size(), 0.0);

    cout << "Creating improved segmentation with " << skeleton.size() << " skeletal segments..." << endl;

    // Stage 1: Assign vertices to skeletal segments with confidence scoring
    for (int segIdx = 0; segIdx < skeleton.size(); segIdx++) {
        const auto& segment = skeleton[segIdx];

        cout << "  Processing segment " << segIdx << " (source: " << segment.sourceIdx
             << ", target: " << segment.targetIdx << ")" << endl;

        Eigen::VectorXd harmonicField = harmonics->computePairwiseHarmonic(segment.sourceIdx, segment.targetIdx);
        if (harmonicField.size() != mesh->verts.size()) continue;

        MeshComponent component;
        component.skeletonSegmentId = segIdx;

        // Find optimal threshold for this harmonic field
        double minVal = harmonicField.minCoeff();
        double maxVal = harmonicField.maxCoeff();
        double midVal = (minVal + maxVal) / 2.0;
        double range = maxVal - minVal;

        if (range < 1e-6) continue; // Skip degenerate fields

        // Use isocurve-based assignment: vertices near isocurves belong to this segment
        for (int v = 0; v < mesh->verts.size(); v++) {
            double harmonicValue = harmonicField(v);

            // Compute distance to skeletal path to determine association strength
            Eigen::Vector3d vertexPos(mesh->verts[v]->coords[0],
                                     mesh->verts[v]->coords[1],
                                     mesh->verts[v]->coords[2]);

            double minDistToSkeleton = 1e10;
            for (const auto& node : segment.nodes) {
                double dist = (vertexPos - node).norm();
                minDistToSkeleton = min(minDistToSkeleton, dist);
            }

            // Compute confidence based on harmonic field gradient and distance to skeleton
            double harmonicConfidence = 1.0 - abs(harmonicValue - midVal) / (range / 2.0);
            double spatialConfidence = exp(-minDistToSkeleton * 5.0); // Exponential falloff
            double totalConfidence = harmonicConfidence * spatialConfidence;

            // Assign vertex if this is the best segment so far
            if (totalConfidence > vertexConfidence[v] && totalConfidence > 0.3) {
                vertexToComponent[v] = segIdx;
                vertexConfidence[v] = totalConfidence;
                component.vertexIndices.push_back(v);
            }
        }

        if (!component.vertexIndices.empty()) {
            // Compute component center
            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for (int vIdx : component.vertexIndices) {
                center += Eigen::Vector3d(mesh->verts[vIdx]->coords[0],
                                         mesh->verts[vIdx]->coords[1],
                                         mesh->verts[vIdx]->coords[2]);
            }
            component.center = center / component.vertexIndices.size();

            components.push_back(component);
            cout << "    Created limb component " << segIdx << " with " << component.vertexIndices.size() << " vertices" << endl;
        }
    }

    // Stage 2: Handle unassigned vertices (torso/junction areas)
    vector<int> unassignedVertices;
    for (int v = 0; v < mesh->verts.size(); v++) {
        if (vertexToComponent[v] == -1) {
            unassignedVertices.push_back(v);
        }
    }

    cout << "  Found " << unassignedVertices.size() << " unassigned vertices for torso/junction components" << endl;

    if (!unassignedVertices.empty()) {
        // Group unassigned vertices using connected components analysis
        vector<bool> visited(mesh->verts.size(), false);

        for (int v : unassignedVertices) {
            if (visited[v]) continue;

            // Start new torso component with connected component search
            MeshComponent torsoComponent;
            torsoComponent.skeletonSegmentId = -1; // Mark as torso/junction
            vector<int> componentVertices;
              // BFS to find connected unassigned vertices using triangle adjacency
            queue<int> toVisit;
            toVisit.push(v);
            visited[v] = true;

            while (!toVisit.empty()) {
                int currentV = toVisit.front();
                toVisit.pop();
                componentVertices.push_back(currentV);

                // Check neighbors through triangle connectivity
                for (int triIdx = 0; triIdx < mesh->tris.size(); triIdx++) {
                    Triangle* tri = mesh->tris[triIdx];
                    vector<int> triVertices = {tri->v1i, tri->v2i, tri->v3i};

                    // Check if current vertex is in this triangle
                    bool containsCurrent = false;
                    for (int vIdx : triVertices) {
                        if (vIdx == currentV) {
                            containsCurrent = true;
                            break;
                        }
                    }

                    if (containsCurrent) {
                        // Add unvisited unassigned neighbors from this triangle
                        for (int neighborV : triVertices) {
                            if (neighborV != currentV && !visited[neighborV] &&
                                vertexToComponent[neighborV] == -1) {
                                visited[neighborV] = true;
                                toVisit.push(neighborV);
                            }
                        }
                    }
                }
            }

            if (componentVertices.size() > 10) { // Only keep substantial components
                torsoComponent.vertexIndices = componentVertices;

                // Compute component center
                Eigen::Vector3d center = Eigen::Vector3d::Zero();
                for (int vIdx : componentVertices) {
                    center += Eigen::Vector3d(mesh->verts[vIdx]->coords[0],
                                             mesh->verts[vIdx]->coords[1],
                                             mesh->verts[vIdx]->coords[2]);
                }
                torsoComponent.center = center / componentVertices.size();

                components.push_back(torsoComponent);
                cout << "    Created torso/junction component with " << componentVertices.size() << " vertices" << endl;
            }
        }
    }

    cout << "Improved segmentation complete: " << components.size() << " total components" << endl;
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
        SoMaterial* mat = new SoMaterial();        // Color by rigidity: green = high rigidity (skeletal centers), red = low rigidity (junctions)
        // This matches the paper convention and other modules
        float rigidity = static_cast<float>(segment.avgRigidity);
        mat->diffuseColor.setValue(1.0f - rigidity, rigidity, 0.2f); // RGB: red decreases, green increases with rigidity
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

void PairwiseHarmonicsSegmentation::computeGlobalVertexRigidity() {
    if (rigidityComputed) return; // Already computed

    cout << "Computing global vertex rigidity for " << mesh->verts.size() << " vertices..." << endl;
    globalVertexRigidity.clear();
    globalVertexRigidity.resize(mesh->verts.size());

    // For each vertex, compute rigidity based on its local neighborhood
    for (int v = 0; v < mesh->verts.size(); v++) {
        if (v % 500 == 0) {
            cout << "  Processing vertex " << v << "/" << mesh->verts.size() << endl;
        }

        // Find neighboring vertices using triangle connectivity
        vector<Eigen::Vector3d> neighborhood;
        Eigen::Vector3d centerVertex(mesh->verts[v]->coords[0],
                                    mesh->verts[v]->coords[1],
                                    mesh->verts[v]->coords[2]);
        neighborhood.push_back(centerVertex);

        // Find neighbors through triangles
        set<int> neighborIndices;
        for (int triIdx = 0; triIdx < mesh->tris.size(); triIdx++) {
            Triangle* tri = mesh->tris[triIdx];
            vector<int> triVertices = {tri->v1i, tri->v2i, tri->v3i};

            // If this triangle contains vertex v, add its other vertices as neighbors
            bool containsV = false;
            for (int vIdx : triVertices) {
                if (vIdx == v) {
                    containsV = true;
                    break;
                }
            }

            if (containsV) {
                for (int neighborV : triVertices) {
                    if (neighborV != v) {
                        neighborIndices.insert(neighborV);
                    }
                }
            }
        }

        // Add neighbor positions to analysis
        for (int neighborIdx : neighborIndices) {
            if (neighborIdx >= 0 && neighborIdx < mesh->verts.size()) {
                neighborhood.push_back(Eigen::Vector3d(
                    mesh->verts[neighborIdx]->coords[0],
                    mesh->verts[neighborIdx]->coords[1],
                    mesh->verts[neighborIdx]->coords[2]
                ));
            }
        }

        // Compute rigidity for this vertex
        if (neighborhood.size() >= 3) {
            globalVertexRigidity[v] = computeNodeRigidity(neighborhood);
        } else {
            globalVertexRigidity[v] = 0.5; // Default for isolated vertices
        }
    }

    // Print statistics
    double minRig = *min_element(globalVertexRigidity.begin(), globalVertexRigidity.end());
    double maxRig = *max_element(globalVertexRigidity.begin(), globalVertexRigidity.end());
    double avgRig = 0;
    for (double r : globalVertexRigidity) avgRig += r;
    avgRig /= globalVertexRigidity.size();

    cout << "Global vertex rigidity computed:" << endl;
    cout << "  Range: [" << minRig << ", " << maxRig << "]" << endl;
    cout << "  Average: " << avgRig << endl;

    rigidityComputed = true;
}
