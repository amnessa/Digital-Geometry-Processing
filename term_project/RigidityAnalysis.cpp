#include "pch.h"
#include "RigidityAnalysis.h"
#include "IsocurveAnalysis.h"
#include "helpers.h"  // For PairwiseHarmonics class
#include <iostream>
#include <algorithm>
#include <cmath>

// Coin3D headers
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>

// Project headers
#include "helpers.h"

using namespace std;

void RigidityAnalysis::enhancedRigidityAnalysis(
    Mesh* mesh,
    PairwiseHarmonics* harmonics,
    const std::vector<int>& fps_samples,
    std::vector<std::vector<Eigen::Vector3d>>& skeletal_segments,
    std::vector<double>& rigidity_scores,
    std::vector<bool>& nonrigid_nodes) {

    if (!mesh || !harmonics || fps_samples.size() < 2) {
        cout << "Error: Please load mesh and compute FPS samples first!" << endl;
        return;
    }

    cout << "\n=== ENHANCED RIGIDITY ANALYSIS ===" << endl;
    cout << "Generating skeletal segments from multiple FPS point pairs..." << endl;

    rigidity_scores.clear();
    nonrigid_nodes.clear();
    skeletal_segments.clear();

    // Generate skeletal segments from MULTIPLE point pairs (not just first 2)
    int maxPairs = min(10, (int)(fps_samples.size() * (fps_samples.size() - 1) / 2)); // Limit to 10 pairs for efficiency
    int pairCount = 0;

    for (int i = 0; i < fps_samples.size() && pairCount < maxPairs; i++) {
        for (int j = i + 1; j < fps_samples.size() && pairCount < maxPairs; j++) {
            int p_idx = fps_samples[i];
            int q_idx = fps_samples[j];

            cout << "  Processing pair " << (pairCount + 1) << "/" << maxPairs
                 << ": vertices " << p_idx << " <-> " << q_idx << endl;

            // Compute harmonic field for this pair
            Eigen::VectorXd harmonicField = harmonics->computePairwiseHarmonic(p_idx, q_idx);
            if (harmonicField.size() != mesh->verts.size()) {
                cout << "    [WARNING] Failed to compute harmonic field for this pair, skipping..." << endl;
                continue;
            }

            // Extract isocurves using proper triangle marching algorithm
            vector<Eigen::Vector3d> skeletal_nodes;
            int numIsocurves = 30; // Use moderate number for efficiency

            for (int k = 1; k < numIsocurves; k++) {
                double isoValue = double(k) / numIsocurves;

                // Extract isocurves using triangle marching
                vector<vector<Eigen::Vector3d>> isocurves = IsocurveAnalysis::extractIsocurvesTriangleMarching(mesh, harmonicField, isoValue);

                // For each isocurve, compute its centroid as a skeletal node
                for (const auto& curve : isocurves) {
                    if (curve.size() >= 3) {  // Only use curves with sufficient points
                        Eigen::Vector3d centroid(0, 0, 0);
                        for (const auto& point : curve) {
                            centroid += point;
                        }
                        centroid /= curve.size();
                        skeletal_nodes.push_back(centroid);
                    }
                }
            }

            if (skeletal_nodes.size() >= 3) {
                skeletal_segments.push_back(skeletal_nodes);
                cout << "    Generated skeletal segment with " << skeletal_nodes.size() << " nodes" << endl;
            }

            pairCount++;
        }
    }

    cout << "Generated " << skeletal_segments.size() << " skeletal segments from " << pairCount << " point pairs" << endl;

    if (skeletal_segments.empty()) {
        cout << "ERROR: No valid skeletal segments generated!" << endl;
        return;
    }

    // Now compute rigidity for ALL nodes from ALL segments
    cout << "Computing rigidity scores for ALL skeletal nodes..." << endl;

    vector<double> allNodeRigidities;
    vector<Eigen::Vector3d> allNodes;

    for (int segIdx = 0; segIdx < skeletal_segments.size(); segIdx++) {
        const auto& nodes = skeletal_segments[segIdx];
        cout << "Analyzing segment " << segIdx + 1 << " with " << nodes.size() << " nodes..." << endl;

        for (int nodeIdx = 0; nodeIdx < nodes.size(); nodeIdx++) {
            double rigidity = computeNodeRigidity(nodes, nodeIdx);
            allNodeRigidities.push_back(rigidity);
            allNodes.push_back(nodes[nodeIdx]);

            cout << "    Node " << nodeIdx << ": rigidity = " << rigidity << endl;
        }
    }

    // Store all rigidity scores
    rigidity_scores = allNodeRigidities;

    // Compute statistics
    if (!allNodeRigidities.empty()) {
        double minRig = *std::min_element(allNodeRigidities.begin(), allNodeRigidities.end());
        double maxRig = *std::max_element(allNodeRigidities.begin(), allNodeRigidities.end());
        double avgRig = 0;
        for (double r : allNodeRigidities) avgRig += r;
        avgRig /= allNodeRigidities.size();

        cout << "\nRigidity Statistics:" << endl;
        cout << "  Total nodes analyzed: " << allNodeRigidities.size() << endl;
        cout << "  Rigidity range: [" << minRig << ", " << maxRig << "]" << endl;
        cout << "  Average rigidity: " << avgRig << endl;
        cout << "  Standard deviation: ";

        double variance = 0;
        for (double r : allNodeRigidities) {
            variance += (r - avgRig) * (r - avgRig);
        }
        variance /= allNodeRigidities.size();
        double stddev = sqrt(variance);
        cout << stddev << endl;

        // Show distribution at different thresholds
        cout << "\nRigidity Distribution:" << endl;
        for (double threshold = 0.5; threshold <= 1.0; threshold += 0.1) {
            int count = 0;
            for (double r : allNodeRigidities) {
                if (r >= threshold) count++;
            }
            double percentage = 100.0 * count / allNodeRigidities.size();
            cout << "    >= " << threshold << ": " << count << " nodes (" << percentage << "%)" << endl;
        }
    }

    cout << "\nEnhanced rigidity analysis complete!" << endl;
}

void RigidityAnalysis::visualizeAllRigidityPoints(
    const Mesh* mesh,
    const std::vector<std::vector<Eigen::Vector3d>>& skeletal_segments,
    const std::vector<double>& rigidity_scores,
    SoSeparator* root,
    SoWinExaminerViewer* viewer) {

    if (rigidity_scores.empty() || skeletal_segments.empty()) {
        cout << "Error: Please run enhanced rigidity analysis first!" << endl;
        return;
    }

    cout << "\n=== VISUALIZING ALL RIGIDITY POINTS ===" << endl;
    cout << "Showing continuous rigidity distribution: Green = Low Rigidity (Junctions), Red = High Rigidity (Limbs)" << endl;

    clearVisualization(root);

    // Get all skeletal nodes
    vector<Eigen::Vector3d> allNodes;
    for (const auto& segment : skeletal_segments) {
        for (const auto& node : segment) {
            allNodes.push_back(node);
        }
    }

    if (allNodes.size() != rigidity_scores.size()) {
        cout << "Error: Mismatch between nodes and rigidity scores!" << endl;
        return;
    }

    // Find rigidity range for color mapping
    double minRig = *std::min_element(rigidity_scores.begin(), rigidity_scores.end());
    double maxRig = *std::max_element(rigidity_scores.begin(), rigidity_scores.end());

    cout << "Rigidity range: [" << minRig << ", " << maxRig << "]" << endl;
    cout << "Color mapping: Green (≤" << minRig + 0.3 * (maxRig - minRig) << ") -> Yellow -> Red (≥" << maxRig - 0.1 * (maxRig - minRig) << ")" << endl;

    // Create visualization for each node
    for (int i = 0; i < allNodes.size(); i++) {
        double rigidity = rigidity_scores[i];
        const Eigen::Vector3d& node = allNodes[i];

        // Compute color based on rigidity (green to red gradient)
        double normalizedRig = (maxRig - minRig) > 1e-10 ? (rigidity - minRig) / (maxRig - minRig) : 0.5;

        // Color mapping: Green (low rigidity/junctions) to Red (high rigidity/limbs)
        float r = std::min(1.0f, (float)(2.0 * normalizedRig));           // 0→1 as rigidity increases
        float g = std::min(1.0f, (float)(2.0 * (1.0 - normalizedRig)));   // 1→0 as rigidity increases
        float b = 0.1f;  // Small amount of blue for better visibility

        Eigen::Vector3f color(r, g, b);
        SoSeparator* pointViz = createColoredSphere(node, color, 0.008f);
        root->addChild(pointViz);
    }

    // Add a legend by creating sample spheres at known positions
    cout << "Creating color legend..." << endl;

    // Find mesh bounding box for legend placement
    Eigen::Vector3d minBounds(1e10, 1e10, 1e10);
    Eigen::Vector3d maxBounds(-1e10, -1e10, -1e10);

    for (int i = 0; i < mesh->verts.size(); i++) {
        Vertex* v = mesh->verts[i];
        minBounds[0] = std::min(minBounds[0], (double)v->coords[0]);
        minBounds[1] = std::min(minBounds[1], (double)v->coords[1]);
        minBounds[2] = std::min(minBounds[2], (double)v->coords[2]);
        maxBounds[0] = std::max(maxBounds[0], (double)v->coords[0]);
        maxBounds[1] = std::max(maxBounds[1], (double)v->coords[1]);
        maxBounds[2] = std::max(maxBounds[2], (double)v->coords[2]);
    }

    // Create legend spheres
    double legendX = maxBounds[0] + 0.2 * (maxBounds[0] - minBounds[0]);
    double legendStartY = maxBounds[1];
    double legendSpacing = 0.1 * (maxBounds[1] - minBounds[1]);

    for (int i = 0; i < 5; i++) {
        double legendRig = double(i) / 4.0; // 0 to 1

        float r = std::min(1.0f, (float)(2.0 * legendRig));
        float g = std::min(1.0f, (float)(2.0 * (1.0 - legendRig)));
        float b = 0.1f;

        Eigen::Vector3d legendPos(legendX, legendStartY - i * legendSpacing, (maxBounds[2] + minBounds[2]) / 2);
        Eigen::Vector3f legendColor(r, g, b);
        SoSeparator* legendSphere = createColoredSphere(legendPos, legendColor, 0.015f);
        root->addChild(legendSphere);
    }

    // Count nodes in different rigidity ranges for feedback
    int lowRig = 0, medRig = 0, highRig = 0;
    double lowThresh = minRig + 0.33 * (maxRig - minRig);
    double highThresh = minRig + 0.67 * (maxRig - minRig);

    for (double rig : rigidity_scores) {
        if (rig < lowThresh) lowRig++;
        else if (rig < highThresh) medRig++;
        else highRig++;
    }

    cout << "\nVisualization complete!" << endl;
    cout << "Node distribution:" << endl;
    cout << "  Low rigidity (Green, potential junctions): " << lowRig << " nodes" << endl;
    cout << "  Medium rigidity (Yellow): " << medRig << " nodes" << endl;
    cout << "  High rigidity (Red, limb centers): " << highRig << " nodes" << endl;
    cout << "\nLegend spheres placed on the right side of the model" << endl;

    viewer->scheduleRedraw();
    viewer->viewAll();
}

void RigidityAnalysis::interactiveRigidityThresholdTesting(
    const std::vector<std::vector<Eigen::Vector3d>>& skeletal_segments,
    const std::vector<double>& rigidity_scores,
    SoSeparator* root,
    SoWinExaminerViewer* viewer) {

    if (rigidity_scores.empty() || skeletal_segments.empty()) {
        cout << "Error: Please run enhanced rigidity analysis first!" << endl;
        return;
    }

    cout << "\n=== INTERACTIVE RIGIDITY THRESHOLD TESTING ===" << endl;
    cout << "This will help you find the optimal threshold for detecting junction points" << endl;
    cout << "Paper recommendation: Use threshold 0.9 (range 0.85-0.95 works well)" << endl;

    // Show current rigidity statistics
    double minRig = *std::min_element(rigidity_scores.begin(), rigidity_scores.end());
    double maxRig = *std::max_element(rigidity_scores.begin(), rigidity_scores.end());

    cout << "\nCurrent rigidity range: [" << minRig << ", " << maxRig << "]" << endl;
    cout << "Total skeletal nodes: " << rigidity_scores.size() << endl;

    while (true) {
        cout << "\n--- THRESHOLD ANALYSIS ---" << endl;
        cout << "Enter a rigidity threshold to test (0.4-1.0, or 'q' to quit): ";

        string input;
        cin >> input;

        if (input == "q" || input == "Q") {
            break;
        }

        double threshold;
        try {
            threshold = std::stod(input);
        } catch (...) {
            cout << "Invalid input! Please enter a number between 0.4 and 1.0" << endl;
            continue;
        }

        if (threshold < 0.4 || threshold > 1.0) {
            cout << "Threshold out of range! Please enter a value between 0.4 and 1.0" << endl;
            cout << "(Paper recommends ~0.9, your data range is [" << minRig << ", " << maxRig << "])" << endl;
            cout << "(Values ~0.33 = junctions, ~1.0 = rigid limbs)" << endl;
            continue;
        }

        // Count nodes at this threshold
        int rigidNodes = 0;
        int nonRigidNodes = 0;
        vector<Eigen::Vector3d> allNodes;

        // Get all skeletal nodes
        for (const auto& segment : skeletal_segments) {
            for (const auto& node : segment) {
                allNodes.push_back(node);
            }
        }

        // Classify nodes based on threshold
        for (int i = 0; i < rigidity_scores.size(); i++) {
            if (rigidity_scores[i] >= threshold) {
                rigidNodes++;
            } else {
                nonRigidNodes++;
            }
        }

        cout << "\nResults for threshold = " << threshold << ":" << endl;
        cout << "  Rigid nodes (skeletal): " << rigidNodes << " (" << (100.0 * rigidNodes / rigidity_scores.size()) << "%)" << endl;
        cout << "  Non-rigid nodes (junctions): " << nonRigidNodes << " (" << (100.0 * nonRigidNodes / rigidity_scores.size()) << "%)" << endl;

        // Visualize nodes with this threshold
        clearVisualization(root);

        for (int i = 0; i < allNodes.size() && i < rigidity_scores.size(); i++) {
            const Eigen::Vector3d& node = allNodes[i];
            double rigidity = rigidity_scores[i];

            // Color based on threshold classification
            float r, g, b;
            float radius;
            if (rigidity >= threshold) {
                // Rigid node - use blue to red gradient based on how rigid it is
                double normalizedRig = (rigidity - threshold) / (maxRig - threshold + 1e-10);
                r = std::min(1.0f, (float)normalizedRig);
                g = 0.2f;
                b = 1.0f - std::min(1.0f, (float)normalizedRig);
                radius = 0.008f;
            } else {
                // Non-rigid node - use green (potential junction)
                r = 0.0f;
                g = 1.0f;
                b = 0.2f;
                radius = 0.012f; // Larger spheres for junction points
            }

            Eigen::Vector3f color(r, g, b);
            SoSeparator* pointViz = createColoredSphere(node, color, radius);
            root->addChild(pointViz);
        }

        // Show interpretation guide
        cout << "\nVisualization Guide:" << endl;
        cout << "  GREEN spheres (large): Non-rigid nodes = Junction candidates" << endl;
        cout << "  BLUE-to-RED spheres (small): Rigid nodes = Skeletal centers" << endl;
        cout << "  -> For human models, you want ~4-8 green junction points" << endl;
        cout << "  -> (shoulders, hips, neck, wrists, ankles, etc.)" << endl;

        viewer->scheduleRedraw();
        viewer->viewAll();
    }

    cout << "Threshold testing complete!" << endl;
}

double RigidityAnalysis::computeNodeRigidity(
    const std::vector<Eigen::Vector3d>& nodes,
    int nodeIdx,
    int windowSize) {

    // Define local neighborhood around this node
    vector<Eigen::Vector3d> neighborhood;
    windowSize = min(windowSize, (int)nodes.size()); // Use up to windowSize neighbors for better PCA

    int start = max(0, nodeIdx - windowSize/2);
    int end = min((int)nodes.size(), start + windowSize);

    for (int k = start; k < end; k++) {
        neighborhood.push_back(nodes[k]);
    }

    // Compute PCA-based rigidity
    double rigidity = 1.0; // Default for edge cases

    if (neighborhood.size() >= 3) {
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

        // Compute PCA-based rigidity using CORRECT formula from paper
        // Paper formula: ξ = λ₁ / (λ₁ + λ₂ + λ₃) where λ₁ ≥ λ₂ ≥ λ₃
        // This gives values in [0,1]: ~1.0 = rigid (linear), ~0.33 = non-rigid (junction)

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        Eigen::Vector3d eigenvalues = solver.eigenvalues();

        // Sort in descending order (λ₁ ≥ λ₂ ≥ λ₃)
        std::sort(eigenvalues.data(), eigenvalues.data() + 3, std::greater<double>());

        double lambda1 = eigenvalues(0);
        double lambda2 = eigenvalues(1);
        double lambda3 = eigenvalues(2);
        double sumLambda = lambda1 + lambda2 + lambda3;

        // CORRECTED rigidity formula from paper
        if (sumLambda > 1e-10) {
            rigidity = lambda1 / sumLambda;  // Correct formula: ξ = λ₁ / (λ₁ + λ₂ + λ₃)
        } else {
            rigidity = 0.33; // Default for degenerate case (junction-like)
        }
    }

    return rigidity;
}

void RigidityAnalysis::clearVisualization(SoSeparator* root) {
    // Remove all children except the first one (which might be the mesh)
    while (root->getNumChildren() > 1) {
        root->removeChild(root->getNumChildren() - 1);
    }
}

SoSeparator* RigidityAnalysis::createColoredSphere(
    const Eigen::Vector3d& position,
    const Eigen::Vector3f& color,
    float radius) {

    SoSeparator* pointSep = new SoSeparator();

    // Material
    SoMaterial* mat = new SoMaterial();
    mat->diffuseColor.setValue(color[0], color[1], color[2]);
    mat->emissiveColor.setValue(color[0] * 0.3f, color[1] * 0.3f, color[2] * 0.3f);
    pointSep->addChild(mat);

    // Transform to position
    SoTransform* transform = new SoTransform();
    transform->translation.setValue(position[0], position[1], position[2]);
    pointSep->addChild(transform);

    // Sphere representing the point
    SoSphere* sphere = new SoSphere();
    sphere->radius = radius;
    pointSep->addChild(sphere);

    return pointSep;
}
