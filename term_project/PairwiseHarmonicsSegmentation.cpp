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
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoPointSet.h>
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
        // STAGE 3: Rigidity Analysis and Partial Skeleton Construction        // =============================================================================
        // STAGE 3: RIGIDITY ANALYSIS ---
        cout << "\n--- STAGE 3: RIGIDITY ANALYSIS ---" << endl;
        computeRigidityAnalysis(rawSegments);
        result.partialSkeleton = selectPartialSkeleton(rawSegments);
        cout << "Selected " << result.partialSkeleton.size() << " segments for partial skeleton" << endl;

        // =============================================================================
        // STAGE 3.5: TORSO SKELETON GENERATION ---
        // =============================================================================
        cout << "\n--- STAGE 3.5: TORSO SKELETON GENERATION ---" << endl;
        vector<SkeletalSegment> torsoSegments = generateTorsoSegments();

        // Add torso segments to the partial skeleton
        for (const auto& torsoSeg : torsoSegments) {
            result.partialSkeleton.push_back(torsoSeg);
        }
        cout << "Added " << torsoSegments.size() << " torso segments to skeleton" << endl;
        cout << "Total skeleton segments: " << result.partialSkeleton.size() << endl;

        // =============================================================================
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
            }            // Extract complete limb segments (PAPER-CORRECT APPROACH)
            vector<SkeletalSegment> pairSegments = extractCompleteLimbSegments(harmonicField, fpsPoints[i], fpsPoints[j]);

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

    // IMPROVED: Use proper triangle marching for robust isocurve extraction
    // This follows the paper's methodology more closely
    int numLevels = std::min(params.numIsocurves, 15); // Cap for performance

    for (int k = 1; k < numLevels; k++) {
        double isoValue = double(k) / numLevels;

        // Use triangle marching to extract proper isocurves
        auto isocurveSegments = harmonics->extractIsoCurveSegments(harmonicField, isoValue);

        if (!isocurveSegments.empty()) {
            // Compute centroid of all isocurve points at this level
            Eigen::Vector3d levelCentroid = Eigen::Vector3d::Zero();
            int pointCount = 0;

            for (const auto& segment : isocurveSegments) {
                levelCentroid += segment.first;
                levelCentroid += segment.second;
                pointCount += 2;
            }

            if (pointCount > 0) {
                levelCentroid /= pointCount;
                centroids.push_back(levelCentroid);
            }
        } else {
            // Fallback: simple tolerance-based approach if triangle marching fails
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
    }

    // Add target FPS point (should correspond to harmonic value ~1.0)
    Eigen::Vector3d targetPoint(mesh->verts[targetIdx]->coords[0],
                               mesh->verts[targetIdx]->coords[1],
                               mesh->verts[targetIdx]->coords[2]);
    centroids.push_back(targetPoint);

    cout << "    Extracted " << centroids.size() << " robust centroids including FPS endpoints" << endl;
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
    }    // PAPER-CORRECT APPROACH: Compute rigidity along the skeletal path
    // Key insight: Rigidity should be computed for nodes on the skeletal segment itself,
    // not mapped from mesh vertices. This follows the paper's algorithm exactly.

    vector<double> skeletalRigidities;
    cout << "    Computing rigidity along skeletal path with " << allCentroids.size() << " nodes..." << endl;

    // Compute rigidity for each node along the skeletal path
    for (int i = 0; i < allCentroids.size(); i++) {
        double nodeRigidity;

        if (allCentroids.size() < 3) {
            nodeRigidity = 0.5; // Default for very short paths
        } else {
            // Define neighborhood around this node on the skeletal path
            vector<Eigen::Vector3d> skeletalNeighborhood;

            // Use a sliding window approach: include nearby nodes on the same path
            int windowSize = min(7, (int)allCentroids.size()); // Adaptive window size
            int halfWindow = windowSize / 2;

            int startIdx = max(0, i - halfWindow);
            int endIdx = min((int)allCentroids.size() - 1, i + halfWindow);

            // Collect neighborhood nodes from the skeletal path
            for (int j = startIdx; j <= endIdx; j++) {
                skeletalNeighborhood.push_back(allCentroids[j]);
            }

            // Compute PCA-based rigidity on skeletal neighborhood
            nodeRigidity = computeNodeRigidity(skeletalNeighborhood);
        }

        skeletalRigidities.push_back(nodeRigidity);

        if (i % 10 == 0) { // Show progress every 10th node
            cout << "      Skeletal node " << i << ": rigidity = " << nodeRigidity << endl;
        }    }

    vector<double> rigidities = skeletalRigidities; // Use skeletal rigidities instead of mesh-mapped ones

    // Find statistics to adaptively set thresholds
    double minRig = *std::min_element(rigidities.begin(), rigidities.end());
    double maxRig = *std::max_element(rigidities.begin(), rigidities.end());
    double meanRig = 0;
    for (double r : rigidities) meanRig += r;
    meanRig /= rigidities.size();cout << "    Rigidity statistics: min=" << minRig << ", max=" << maxRig << ", mean=" << meanRig << endl;    // NEW APPROACH: Terminate segments at anatomical boundaries
    // Key insight: Segments should STOP when entering low-rigidity areas (torso)
    // rather than forcing connections across the entire body

    // Define anatomical boundary based on actual rigidity distribution
    double anatomicalBoundary = 0.6;  // True anatomical junctions

    // For paths with generally high rigidity, use adaptive threshold
    if (meanRig > 0.65) {
        anatomicalBoundary = meanRig - 0.1;  // Stop before entering low-rigidity areas
        cout << "    High-rigidity path detected - using adaptive boundary: " << anatomicalBoundary << endl;
    }

    cout << "    Anatomical boundary threshold: " << anatomicalBoundary << endl;    // PAPER-BASED APPROACH: Create individual limb segments that terminate at anatomical boundaries
    // Key insight: Each segment should represent ONE limb (arm/leg) from extremity to junction

    // Strategy: Find the best limb segment by identifying the longest high-rigidity continuous path
    // that starts or ends at an extremity (FPS point) and terminates at an anatomical boundary

    vector<int> segmentBoundaries; // Indices for limb segment boundaries

    // STEP 1: Identify extremity endpoints (should have high rigidity)
    bool sourceIsExtremity = (rigidities[0] >= anatomicalBoundary);
    bool targetIsExtremity = (rigidities.back() >= anatomicalBoundary);

    cout << "    Source extremity (FPS " << sourceIdx << "): rigidity = " << rigidities[0]
         << (sourceIsExtremity ? " (LIMB TIP)" : " (LOW-RIGIDITY)") << endl;
    cout << "    Target extremity (FPS " << targetIdx << "): rigidity = " << rigidities.back()
         << (targetIsExtremity ? " (LIMB TIP)" : " (LOW-RIGIDITY)") << endl;

    // STEP 2: Find limb segments that start from extremities and terminate at anatomical boundaries
    if (sourceIsExtremity) {
        // Create a limb segment starting from source extremity
        int limbEndIdx = 0;

        // Find where the limb terminates (first significant drop in rigidity)
        for (int i = 1; i < rigidities.size(); i++) {
            if (rigidities[i] < anatomicalBoundary) {
                limbEndIdx = i - 1;  // Stop before entering low-rigidity area
                cout << "      Source limb terminates at centroid " << limbEndIdx
                     << " (before low-rigidity zone: " << rigidities[i] << ")" << endl;
                break;
            }
            limbEndIdx = i;
        }

        // Only create segment if it has reasonable length
        if (limbEndIdx > 2) {  // At least 3 points for a meaningful limb
            segmentBoundaries.push_back(0);
            segmentBoundaries.push_back(limbEndIdx);
        }
    }

    if (targetIsExtremity) {
        // Create a limb segment ending at target extremity
        int limbStartIdx = rigidities.size() - 1;

        // Find where the limb starts (working backwards from target)
        for (int i = rigidities.size() - 2; i >= 0; i--) {
            if (rigidities[i] < anatomicalBoundary) {
                limbStartIdx = i + 1;  // Start after leaving low-rigidity area
                cout << "      Target limb starts at centroid " << limbStartIdx
                     << " (after low-rigidity zone: " << rigidities[i] << ")" << endl;
                break;
            }
            limbStartIdx = i;
        }

        // Only create segment if it has reasonable length and doesn't overlap with source segment
        if ((rigidities.size() - 1 - limbStartIdx) > 2 &&
            (segmentBoundaries.empty() || limbStartIdx > segmentBoundaries.back())) {
            segmentBoundaries.push_back(limbStartIdx);
            segmentBoundaries.push_back(rigidities.size() - 1);
        }
    }    cout << "    Found " << (segmentBoundaries.size() / 2) << " individual limb segments" << endl;

    // STEP 3: Create actual limb segments from identified boundaries
    for (int boundaryIdx = 0; boundaryIdx < segmentBoundaries.size(); boundaryIdx += 2) {
        if (boundaryIdx + 1 < segmentBoundaries.size()) {
            int startIdx = segmentBoundaries[boundaryIdx];
            int endIdx = segmentBoundaries[boundaryIdx + 1];

            // Verify this represents a valid limb segment
            if (endIdx - startIdx >= 2) {
                // Check rigidity composition - should be mostly high-rigidity for limb segments
                int highRigidityCount = 0;
                for (int i = startIdx; i <= endIdx; i++) {
                    if (i < rigidities.size() && rigidities[i] >= anatomicalBoundary) {
                        highRigidityCount++;
                    }
                }

                // Only create segment if it's predominantly high-rigidity (actual limb)
                double highRigidityRatio = double(highRigidityCount) / (endIdx - startIdx + 1);
                if (highRigidityRatio >= 0.7) {  // Stricter requirement: 70% high-rigidity
                    SkeletalSegment segment;

                    // Copy nodes for this limb segment
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

                    // Only add limb segments with reasonable length and high quality
                    if (segment.length > 0.15 && segment.avgRigidity >= anatomicalBoundary) {
                        segments.push_back(segment);
                        cout << "      Created individual limb segment " << (segments.size()-1) << ": "
                             << segment.nodes.size() << " nodes, length: " << segment.length
                             << ", avg rigidity: " << segment.avgRigidity << endl;
                    }
                    else {
                        cout << "      Skipped short/low-quality limb segment (length: " << segment.length
                             << ", rigidity: " << segment.avgRigidity << ")" << endl;
                    }
                }
                else {
                    cout << "      Skipped mixed-rigidity segment (likely crosses torso): high-rigidity ratio = "
                         << highRigidityRatio << endl;
                }
            }
        }
    }

    return segments;
}

/**
 * Extract complete anatomical limb segments according to the paper
 * Key insight: Generate complete paths from extremities to torso junctions,
 * not fragmented segments cut at every rigidity change
 */
vector<PairwiseHarmonicsSegmentation::SkeletalSegment>
PairwiseHarmonicsSegmentation::extractCompleteLimbSegments(
    const Eigen::VectorXd& harmonicField, int sourceIdx, int targetIdx) {

    vector<SkeletalSegment> segments;

    // Extract isocurve centroids as the skeletal path
    vector<Eigen::Vector3d> skeletalPath = extractIsocurveCentroids(harmonicField, sourceIdx, targetIdx);

    if (skeletalPath.size() < 3) {
        return segments; // Need minimum nodes for a meaningful segment
    }

    // Compute rigidity along the entire path
    vector<double> pathRigidities;
    for (int i = 0; i < skeletalPath.size(); i++) {
        vector<Eigen::Vector3d> neighborhood;

        // Create neighborhood on the skeletal path
        int windowSize = min(7, (int)skeletalPath.size());
        int halfWindow = windowSize / 2;
        int startIdx = max(0, i - halfWindow);
        int endIdx = min((int)skeletalPath.size() - 1, i + halfWindow);

        for (int j = startIdx; j <= endIdx; j++) {
            neighborhood.push_back(skeletalPath[j]);
        }

        double rigidity = computeNodeRigidity(neighborhood);
        pathRigidities.push_back(rigidity);
    }

    // Find rigidity statistics for adaptive processing
    double minRig = *min_element(pathRigidities.begin(), pathRigidities.end());
    double maxRig = *max_element(pathRigidities.begin(), pathRigidities.end());
    double meanRig = 0;
    for (double r : pathRigidities) meanRig += r;
    meanRig /= pathRigidities.size();

    cout << "    Path rigidity: min=" << minRig << ", max=" << maxRig << ", mean=" << meanRig << endl;

    // PAPER-CORRECT APPROACH: Create ONE complete segment for the entire path
    // Only divide if there are clear anatomical boundaries (very low rigidity zones)

    // Check if this path represents a complete limb (high rigidity at endpoints)
    bool sourceIsLimbTip = pathRigidities.front() > 0.65;
    bool targetIsLimbTip = pathRigidities.back() > 0.65;

    cout << "    Source rigidity: " << pathRigidities.front() << (sourceIsLimbTip ? " (LIMB TIP)" : " (JUNCTION)") << endl;
    cout << "    Target rigidity: " << pathRigidities.back() << (targetIsLimbTip ? " (LIMB TIP)" : " (JUNCTION)") << endl;

    // Strategy 1: If both endpoints are limb tips, create complete path
    if (sourceIsLimbTip && targetIsLimbTip) {
        SkeletalSegment completePath;
        completePath.nodes = skeletalPath;
        completePath.nodeRigidities = pathRigidities;
        completePath.sourceIdx = sourceIdx;
        completePath.targetIdx = targetIdx;

        // Compute segment properties
        completePath.length = 0;
        for (int i = 1; i < skeletalPath.size(); i++) {
            completePath.length += (skeletalPath[i] - skeletalPath[i-1]).norm();
        }

        completePath.avgRigidity = meanRig;
        completePath.quality = completePath.avgRigidity * completePath.length;

        segments.push_back(completePath);
        cout << "    Created complete limb-to-limb path: length=" << completePath.length << ", quality=" << completePath.quality << endl;
    }

    // Strategy 2: If one endpoint is limb tip, create limb segment from tip to junction
    else if (sourceIsLimbTip || targetIsLimbTip) {
        // Find where the limb connects to the torso (major rigidity drop)
        double junctionThreshold = 0.55; // Conservative threshold for true anatomical junctions

        if (sourceIsLimbTip) {
            // Create limb segment from source tip to torso junction
            int junctionIdx = pathRigidities.size() - 1; // Default to end

            // Find first significant rigidity drop
            for (int i = 1; i < pathRigidities.size(); i++) {
                if (pathRigidities[i] < junctionThreshold) {
                    junctionIdx = i;
                    break;
                }
            }

            // Only create segment if it has reasonable length
            if (junctionIdx > 2) {
                SkeletalSegment limbSegment;
                limbSegment.nodes = vector<Eigen::Vector3d>(skeletalPath.begin(), skeletalPath.begin() + junctionIdx + 1);
                limbSegment.nodeRigidities = vector<double>(pathRigidities.begin(), pathRigidities.begin() + junctionIdx + 1);
                limbSegment.sourceIdx = sourceIdx;
                limbSegment.targetIdx = targetIdx;

                limbSegment.length = 0;
                for (int i = 1; i < limbSegment.nodes.size(); i++) {
                    limbSegment.length += (limbSegment.nodes[i] - limbSegment.nodes[i-1]).norm();
                }

                double rigSum = 0;
                for (double r : limbSegment.nodeRigidities) rigSum += r;
                limbSegment.avgRigidity = rigSum / limbSegment.nodeRigidities.size();
                limbSegment.quality = limbSegment.avgRigidity * limbSegment.length;

                segments.push_back(limbSegment);
                cout << "    Created source limb segment: length=" << limbSegment.length << ", nodes=" << limbSegment.nodes.size() << endl;
            }
        }

        if (targetIsLimbTip) {
            // Create limb segment from torso junction to target tip
            int junctionIdx = 0; // Default to start

            // Find last significant rigidity drop (working backwards)
            for (int i = pathRigidities.size() - 2; i >= 0; i--) {
                if (pathRigidities[i] < junctionThreshold) {
                    junctionIdx = i + 1;
                    break;
                }
            }

            // Only create segment if it has reasonable length and doesn't overlap with source segment
            if ((pathRigidities.size() - 1 - junctionIdx) > 2) {
                SkeletalSegment limbSegment;
                limbSegment.nodes = vector<Eigen::Vector3d>(skeletalPath.begin() + junctionIdx, skeletalPath.end());
                limbSegment.nodeRigidities = vector<double>(pathRigidities.begin() + junctionIdx, pathRigidities.end());
                limbSegment.sourceIdx = sourceIdx;
                limbSegment.targetIdx = targetIdx;

                limbSegment.length = 0;
                for (int i = 1; i < limbSegment.nodes.size(); i++) {
                    limbSegment.length += (limbSegment.nodes[i] - limbSegment.nodes[i-1]).norm();
                }

                double rigSum = 0;
                for (double r : limbSegment.nodeRigidities) rigSum += r;
                limbSegment.avgRigidity = rigSum / limbSegment.nodeRigidities.size();
                limbSegment.quality = limbSegment.avgRigidity * limbSegment.length;

                segments.push_back(limbSegment);
                cout << "    Created target limb segment: length=" << limbSegment.length << ", nodes=" << limbSegment.nodes.size() << endl;
            }
        }
    }

    // Strategy 3: If neither endpoint is a limb tip, this might be a torso connection
    else {
        // Create a single torso connection segment only if it has decent rigidity overall
        if (meanRig > 0.5) {
            SkeletalSegment torsoConnection;
            torsoConnection.nodes = skeletalPath;
            torsoConnection.nodeRigidities = pathRigidities;
            torsoConnection.sourceIdx = sourceIdx;
            torsoConnection.targetIdx = targetIdx;

            torsoConnection.length = 0;
            for (int i = 1; i < skeletalPath.size(); i++) {
                torsoConnection.length += (skeletalPath[i] - skeletalPath[i-1]).norm();
            }

            torsoConnection.avgRigidity = meanRig;
            torsoConnection.quality = torsoConnection.avgRigidity * torsoConnection.length * 0.5; // Lower priority for torso connections

            segments.push_back(torsoConnection);
            cout << "    Created torso connection: length=" << torsoConnection.length << ", quality=" << torsoConnection.quality << endl;
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

    cout << "\n=== GREEDY SKELETON SELECTION (Paper Algorithm) ===" << endl;
    cout << "Input pool: " << segments.size() << " candidate skeletal segments" << endl;

    vector<SkeletalSegment> candidatePool;
    vector<SkeletalSegment> finalSkeleton;

    // STEP 0: Pre-filter by quality and length (CRITICAL IMPROVEMENT)
    cout << "Applying quality filters:" << endl;
    cout << "  Min quality: " << params.minQualityThreshold << endl;
    cout << "  Min length: " << params.minSegmentLength << endl;
    cout << "  Min nodes: " << params.minSkeletalNodes << endl;

    for (const auto& segment : segments) {



        bool meetsCriteria = (segment.quality >= params.minQualityThreshold &&
                             segment.length >= params.minSegmentLength &&
                             segment.nodes.size() >= params.minSkeletalNodes);

        if (meetsCriteria) {
            candidatePool.push_back(segment);
        }
    }

    cout << "After filtering: " << candidatePool.size() << " candidates (from "
         << segments.size() << " total)" << endl;

    if (candidatePool.empty()) {
        cout << "ERROR: No segments meet minimum quality criteria!" << endl;
        return finalSkeleton;
    }

    // Sort the filtered pool by quality (descending)
    sort(candidatePool.begin(), candidatePool.end(),
         [](const SkeletalSegment& a, const SkeletalSegment& b) {
             return a.quality > b.quality;
         });

    cout << "\nTop candidates after filtering:" << endl;
    for (int i = 0; i < min(8, (int)candidatePool.size()); i++) {
        cout << "  Rank " << (i+1) << ": FPS " << candidatePool[i].sourceIdx
             << " -> " << candidatePool[i].targetIdx
             << " (Q: " << candidatePool[i].quality
             << ", L: " << candidatePool[i].length
             << ", R: " << candidatePool[i].avgRigidity << ")" << endl;
    }

    cout << "\n=== GREEDY SELECTION PROCESS ===" << endl;

    // GREEDY ALGORITHM: Iteratively select best non-overlapping segments
    while (!candidatePool.empty() && finalSkeleton.size() < params.maxSkeletonSegments) {

        // STEP 1: Find the highest-scoring segment in remaining pool
        SkeletalSegment bestSegment = candidatePool[0]; // Already sorted by quality
        finalSkeleton.push_back(bestSegment);

        cout << "Selected segment " << finalSkeleton.size() << ": FPS "
             << bestSegment.sourceIdx << " -> " << bestSegment.targetIdx
             << " (quality: " << bestSegment.quality
             << ", length: " << bestSegment.length
             << ", avg rigidity: " << bestSegment.avgRigidity << ")" << endl;

        // STEP 2: Remove this segment from the pool
        candidatePool.erase(candidatePool.begin());

        // STEP 3: Remove all segments that overlap too much with the selected one
        vector<SkeletalSegment> nonOverlapping;
        int removedCount = 0;

        for (const auto& candidate : candidatePool) {
            bool hasOverlap = checkSegmentOverlap(bestSegment, candidate);

            if (!hasOverlap) {
                nonOverlapping.push_back(candidate);
            } else {
                removedCount++;
            }
        }

        cout << "  Removed " << removedCount << " overlapping segments" << endl;
        cout << "  Remaining candidates: " << nonOverlapping.size() << endl;

        // Update candidate pool
        candidatePool = nonOverlapping;
    }

    cout << "\n=== GREEDY SELECTION COMPLETE ===" << endl;
    cout << "Final skeleton: " << finalSkeleton.size() << " non-overlapping segments" << endl;

    // Print final skeleton summary
    for (int i = 0; i < finalSkeleton.size(); i++) {
        const auto& seg = finalSkeleton[i];
        cout << "  Segment " << (i+1) << ": FPS " << seg.sourceIdx << " -> " << seg.targetIdx
             << " (quality: " << seg.quality << ", length: " << seg.length << ")" << endl;
    }

    return finalSkeleton;
}

/**
 * Check if two skeletal segments overlap spatially
 * IMPROVED VERSION: More aggressive overlap detection to prevent spaghetti skeleton
 */
bool PairwiseHarmonicsSegmentation::checkSegmentOverlap(
    const SkeletalSegment& seg1, const SkeletalSegment& seg2) {

    // Method 1: Check FPS endpoint sharing (CRITICAL OVERLAP)
    // This is the most reliable sign of overlap.
    bool shareEndpoints = (seg1.sourceIdx == seg2.sourceIdx) ||
                         (seg1.sourceIdx == seg2.targetIdx) ||
                         (seg1.targetIdx == seg2.sourceIdx) ||
                         (seg1.targetIdx == seg2.targetIdx);

    if (shareEndpoints) {
        return true; // Definite overlap, no further checks needed.
    }

    // --- START OF REPLACEMENT ---
    // REVISED, LESS AGGRESSIVE OVERLAP LOGIC:
    // Two segments are considered overlapping only if they are very similar
    // in BOTH position AND orientation. This prevents the removal of distinct
    // anatomical parts (like a leg) that might share a path with a longer
    // segment (like a back-to-leg path) but have a different overall direction.

    // Step 1: Check for significant spatial proximity.
    double overlapThreshold = params.overlapSpatialThreshold;
    int overlapCount = 0;
    // Use a fixed number of samples for consistent comparison.
    int totalSamples = 10;

    for (int i = 0; i < totalSamples; i++) {
        double ratio = (i + 0.5) / totalSamples; // Sample from middle of sub-segments

        int idx1 = min(int(ratio * (seg1.nodes.size() - 1)), (int)seg1.nodes.size() - 1);
        Eigen::Vector3d pos1 = seg1.nodes[idx1];

        // Find the closest point on the other segment to this sample point.
        double minDistance = 1e10;
        for (const auto& node2 : seg2.nodes) {
            minDistance = min(minDistance, (pos1 - node2).norm());
        }

        if (minDistance < overlapThreshold) {
            overlapCount++;
        }
    }
    double overlapRatio = double(overlapCount) / double(totalSamples);

    // Step 2: Check for parallel orientation.
    bool isParallel = false;
    if (seg1.nodes.size() >= 2 && seg2.nodes.size() >= 2) {
        Eigen::Vector3d dir1 = (seg1.nodes.back() - seg1.nodes.front()).normalized();
        Eigen::Vector3d dir2 = (seg2.nodes.back() - seg2.nodes.front()).normalized();
        double dotProduct = abs(dir1.dot(dir2));

        // Consider them parallel if their end-to-end vectors are closely aligned.
        if (dotProduct > 0.85) { // Stricter threshold: >85% parallel
            isParallel = true;
        }
    }

    // FINAL DECISION: Overlap only if they are both spatially close AND parallel.
    // This correctly identifies duplicate paths while preserving distinct components.
    return (overlapRatio > 0.7 && isParallel);
    // --- END OF REPLACEMENT ---
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

    cout << "Creating improved segmentation with " << skeleton.size() << " skeletal segments..." << endl;    // Stage 1: Assign vertices to skeletal segments with confidence scoring
    for (int segIdx = 0; segIdx < skeleton.size(); segIdx++) {
        const auto& segment = skeleton[segIdx];

        cout << "  Processing segment " << segIdx << " (source: " << segment.sourceIdx
             << ", target: " << segment.targetIdx << ")" << endl;

        // Special handling for torso segments (sourceIdx = -1, targetIdx = -1)
        if (segment.sourceIdx == -1 || segment.targetIdx == -1) {
            cout << "    Skipping torso segment (no harmonic field computation)" << endl;
            continue;
        }

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
    }    // Stage 2: Handle torso segments and unassigned vertices
    vector<int> unassignedVertices;
    for (int v = 0; v < mesh->verts.size(); v++) {
        if (vertexToComponent[v] == -1) {
            unassignedVertices.push_back(v);
        }
    }

    cout << "  Found " << unassignedVertices.size() << " unassigned vertices for torso/junction components" << endl;

    // Stage 2a: Assign vertices to torso segments using spatial proximity
    for (int segIdx = 0; segIdx < skeleton.size(); segIdx++) {
        const auto& segment = skeleton[segIdx];

        // Process only torso segments (sourceIdx = -1, targetIdx = -1)
        if (segment.sourceIdx != -1 || segment.targetIdx != -1) continue;

        cout << "  Processing torso segment " << segIdx << " with " << segment.nodes.size() << " nodes" << endl;

        MeshComponent torsoComponent;
        torsoComponent.skeletonSegmentId = segIdx;

        // Assign unassigned vertices based on proximity to torso segment nodes
        vector<int> assignedToTorso;
        for (int v : unassignedVertices) {
            if (vertexToComponent[v] != -1) continue; // Already assigned

            Eigen::Vector3d vertexPos(mesh->verts[v]->coords[0],
                                     mesh->verts[v]->coords[1],
                                     mesh->verts[v]->coords[2]);

            // Find minimum distance to any node in this torso segment
            double minDistToSegment = 1e10;
            for (const auto& node : segment.nodes) {
                double dist = (vertexPos - node).norm();
                minDistToSegment = min(minDistToSegment, dist);
            }

            // Assign vertex if it's close enough to this torso segment
            double maxTorsoDistance = 0.3; // Reasonable distance threshold for torso assignment
            if (minDistToSegment < maxTorsoDistance) {
                assignedToTorso.push_back(v);
                vertexToComponent[v] = components.size(); // Will be assigned to this component
            }
        }

        // Only create torso component if we have enough vertices
        if (assignedToTorso.size() > 50) { // Substantial torso component
            torsoComponent.vertexIndices = assignedToTorso;

            // Compute component center
            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for (int vIdx : assignedToTorso) {
                center += Eigen::Vector3d(mesh->verts[vIdx]->coords[0],
                                         mesh->verts[vIdx]->coords[1],
                                         mesh->verts[vIdx]->coords[2]);
            }
            torsoComponent.center = center / assignedToTorso.size();

            components.push_back(torsoComponent);
            cout << "    Created torso component " << segIdx << " with " << assignedToTorso.size() << " vertices" << endl;
        } else {
            cout << "    Skipped small torso segment (only " << assignedToTorso.size() << " vertices)" << endl;
            // Reset vertex assignments for small segments
            for (int v : assignedToTorso) {
                vertexToComponent[v] = -1;
            }
        }
    }

    // Stage 2b: Handle remaining unassigned vertices with connected components
    // Update unassigned list
    unassignedVertices.clear();
    for (int v = 0; v < mesh->verts.size(); v++) {
        if (vertexToComponent[v] == -1) {
            unassignedVertices.push_back(v);
        }
    }

    cout << "  Found " << unassignedVertices.size() << " remaining unassigned vertices" << endl;

    if (!unassignedVertices.empty()) {
        // Group remaining unassigned vertices using connected components analysis
        vector<bool> visited(mesh->verts.size(), false);

        for (int v : unassignedVertices) {
            if (visited[v]) continue;

            // Start new junction component with connected component search
            MeshComponent junctionComponent;
            junctionComponent.skeletonSegmentId = -1; // Mark as junction/remainder
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
            }            if (componentVertices.size() > 10) { // Only keep substantial components
                junctionComponent.vertexIndices = componentVertices;

                // Compute component center
                Eigen::Vector3d center = Eigen::Vector3d::Zero();
                for (int vIdx : componentVertices) {
                    center += Eigen::Vector3d(mesh->verts[vIdx]->coords[0],
                                             mesh->verts[vIdx]->coords[1],
                                             mesh->verts[vIdx]->coords[2]);
                }
                junctionComponent.center = center / componentVertices.size();

                components.push_back(junctionComponent);
                cout << "    Created junction component with " << componentVertices.size() << " vertices" << endl;
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
    }    // Visualize partial skeleton segments
    cout << "Visualizing " << result.partialSkeleton.size() << " skeletal segments..." << endl;

    int validSegments = 0;
    for (int i = 0; i < result.partialSkeleton.size(); i++) {
        const auto& segment = result.partialSkeleton[i];

        // Apply comprehensive filters to remove problematic segments
        bool skipSegment = false;
        string skipReason;

        validSegments++;
        cout << "  Rendering segment " << i << ": " << segment.nodes.size() << " nodes, length=" << segment.length << endl;

        SoSeparator* segSep = new SoSeparator();

        // Set material properties for better visibility
        SoMaterial* mat = new SoMaterial();

        // Color by rigidity: green = high rigidity (skeletal centers), red = low rigidity (junctions)
        // Ensure colors are bright and visible
        float rigidity = static_cast<float>(segment.avgRigidity);
        if (rigidity < 0.1f) rigidity = 0.1f;  // Ensure minimum visibility
        if (rigidity > 0.9f) rigidity = 0.9f;  // Ensure contrast

        mat->diffuseColor.setValue(1.0f - rigidity, rigidity, 0.2f); // RGB: red decreases, green increases with rigidity
        mat->emissiveColor.setValue(0.2f * (1.0f - rigidity), 0.2f * rigidity, 0.1f); // Add emissive for visibility
        segSep->addChild(mat);

        // Set line width for better visibility
        SoDrawStyle* drawStyle = new SoDrawStyle();
        drawStyle->lineWidth = 3.0f;  // Make lines thicker
        drawStyle->pointSize = 5.0f;  // Make points visible too
        segSep->addChild(drawStyle);

        // Create skeletal path coordinates
        SoCoordinate3* coords = new SoCoordinate3();
        coords->point.setNum(segment.nodes.size());
        for (int j = 0; j < segment.nodes.size(); j++) {
            coords->point.set1Value(j,
                segment.nodes[j][0],
                segment.nodes[j][1],
                segment.nodes[j][2]
            );
            cout << "    Node " << j << ": (" << segment.nodes[j][0] << ", "
                 << segment.nodes[j][1] << ", " << segment.nodes[j][2] << ")" << endl;
        }
        segSep->addChild(coords);

        // Create line segments connecting nodes
        SoIndexedLineSet* lineSet = new SoIndexedLineSet();
        for (int j = 0; j < segment.nodes.size() - 1; j++) {
            lineSet->coordIndex.set1Value(j * 3, j);
            lineSet->coordIndex.set1Value(j * 3 + 1, j + 1);
            lineSet->coordIndex.set1Value(j * 3 + 2, -1);
        }
        segSep->addChild(lineSet);

        // Also add points at nodes for visibility
        SoPointSet* pointSet = new SoPointSet();
        pointSet->numPoints = segment.nodes.size();
        segSep->addChild(pointSet);

        root->addChild(segSep);
    }

    cout << "Rendered " << validSegments << " valid segments (filtered out " << (result.partialSkeleton.size() - validSegments) << ")" << endl;

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

/**
 * Generate skeletal segments from low-rigidity (torso) areas
 * IMPROVED VERSION: Creates meaningful skeletal structure within the torso using junction analysis
 */
vector<PairwiseHarmonicsSegmentation::SkeletalSegment>
PairwiseHarmonicsSegmentation::generateTorsoSegments() {
    vector<SkeletalSegment> torsoSegments;

    if (!rigidityComputed) {
        computeGlobalVertexRigidity();
    }

    cout << "Generating improved torso skeleton from low-rigidity areas..." << endl;

    // Identify low-rigidity vertices (torso/junction areas)
    double torsoThreshold = 0.6; // Same threshold used for anatomical boundaries
    vector<int> torsoVertices;

    for (int v = 0; v < mesh->verts.size(); v++) {
        if (globalVertexRigidity[v] < torsoThreshold) {
            torsoVertices.push_back(v);
        }
    }

    cout << "  Found " << torsoVertices.size() << " low-rigidity vertices for torso analysis" << endl;

    if (torsoVertices.size() < 20) {
        cout << "  Insufficient torso vertices for segment generation" << endl;
        return torsoSegments;
    }

    // STEP 1: Find torso center and main axis
    Eigen::Vector3d torsoCenter = Eigen::Vector3d::Zero();
    for (int v : torsoVertices) {
        torsoCenter += Eigen::Vector3d(mesh->verts[v]->coords[0],
                                      mesh->verts[v]->coords[1],
                                      mesh->verts[v]->coords[2]);
    }
    torsoCenter /= torsoVertices.size();

    // STEP 2: Find principal direction of torso using PCA
    Eigen::MatrixXd torsoPoints(torsoVertices.size(), 3);
    for (int i = 0; i < torsoVertices.size(); i++) {
        int v = torsoVertices[i];
        torsoPoints(i, 0) = mesh->verts[v]->coords[0] - torsoCenter[0];
        torsoPoints(i, 1) = mesh->verts[v]->coords[1] - torsoCenter[1];
        torsoPoints(i, 2) = mesh->verts[v]->coords[2] - torsoCenter[2];
    }

    Eigen::Matrix3d covariance = torsoPoints.transpose() * torsoPoints / torsoVertices.size();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    Eigen::Vector3d mainAxis = solver.eigenvectors().col(2);  // Principal component

    cout << "  Torso center: (" << torsoCenter[0] << ", " << torsoCenter[1] << ", " << torsoCenter[2] << ")" << endl;
    cout << "  Main axis: (" << mainAxis[0] << ", " << mainAxis[1] << ", " << mainAxis[2] << ")" << endl;

    // STEP 3: Create skeletal segments along main torso axes
    // Generate segments that represent the main torso structure
    vector<Eigen::Vector3d> torsoSkeletonNodes;

    // Find extremes along the main axis
    double minProjection = 1e10, maxProjection = -1e10;
    Eigen::Vector3d minPoint, maxPoint;

    for (int v : torsoVertices) {
        Eigen::Vector3d vertexPos(mesh->verts[v]->coords[0],
                                 mesh->verts[v]->coords[1],
                                 mesh->verts[v]->coords[2]);

        double projection = (vertexPos - torsoCenter).dot(mainAxis);

        if (projection < minProjection) {
            minProjection = projection;
            minPoint = vertexPos;
        }
        if (projection > maxProjection) {
            maxProjection = projection;
            maxPoint = vertexPos;
        }
    }

    // Create main torso segment along principal axis
    SkeletalSegment mainTorsoSegment;
    mainTorsoSegment.nodes.push_back(minPoint);
    mainTorsoSegment.nodes.push_back(torsoCenter);
    mainTorsoSegment.nodes.push_back(maxPoint);    // Set rigidity values for torso nodes (low rigidity)
    mainTorsoSegment.nodeRigidities.push_back(torsoThreshold * 0.8);
    mainTorsoSegment.nodeRigidities.push_back(torsoThreshold * 0.7);
    mainTorsoSegment.nodeRigidities.push_back(torsoThreshold * 0.8);

    mainTorsoSegment.sourceIdx = -1; // Special marker for torso segments
    mainTorsoSegment.targetIdx = -1;

    mainTorsoSegment.length = (maxPoint - minPoint).norm();
    mainTorsoSegment.avgRigidity = torsoThreshold * 0.75;
    mainTorsoSegment.quality = mainTorsoSegment.avgRigidity * mainTorsoSegment.length;

    if (mainTorsoSegment.length > 0.1) {
        torsoSegments.push_back(mainTorsoSegment);
        cout << "  Created main torso segment: length = " << mainTorsoSegment.length << endl;
    }

    // STEP 4: Create secondary segments for junction connections
    // Find points that are potential junction connection points (connection to limb attachment areas)
    vector<Eigen::Vector3d> secondaryAxes;
    secondaryAxes.push_back(solver.eigenvectors().col(1));  // Second principal component
    secondaryAxes.push_back(solver.eigenvectors().col(0));  // Third principal component

    for (int axisIdx = 0; axisIdx < secondaryAxes.size(); axisIdx++) {
        Eigen::Vector3d axis = secondaryAxes[axisIdx];

        // Find extremes along this secondary axis
        double minProj = 1e10, maxProj = -1e10;
        Eigen::Vector3d minPt, maxPt;

        for (int v : torsoVertices) {
            Eigen::Vector3d vertexPos(mesh->verts[v]->coords[0],
                                     mesh->verts[v]->coords[1],
                                     mesh->verts[v]->coords[2]);

            double proj = (vertexPos - torsoCenter).dot(axis);

            if (proj < minProj) {
                minProj = proj;
                minPt = vertexPos;
            }
            if (proj > maxProj) {
                maxProj = proj;
                maxPt = vertexPos;
            }
        }

        // Create secondary torso segment
        SkeletalSegment secondarySegment;
        secondarySegment.nodes.push_back(minPt);
        secondarySegment.nodes.push_back(torsoCenter);
        secondarySegment.nodes.push_back(maxPt);

        secondarySegment.nodeRigidities.push_back(torsoThreshold * 0.6);
        secondarySegment.nodeRigidities.push_back(torsoThreshold * 0.5);
        secondarySegment.nodeRigidities.push_back(torsoThreshold * 0.6);

        secondarySegment.sourceIdx = -1;
        secondarySegment.targetIdx = -1;

        secondarySegment.length = (maxPt - minPt).norm();
        secondarySegment.avgRigidity = torsoThreshold * 0.6;
        secondarySegment.quality = secondarySegment.avgRigidity * secondarySegment.length;

        if (secondarySegment.length > 0.08) {  // Smaller threshold for secondary segments
            torsoSegments.push_back(secondarySegment);
            cout << "  Created secondary torso segment " << axisIdx + 1 << ": length = "
                 << secondarySegment.length << endl;
        }
    }    cout << "  Generated " << torsoSegments.size() << " total torso segments" << endl;

    return torsoSegments;
}
