#include "pch.h"  // Include precompiled header first

#include <stdio.h>
#include <cstdlib>
#include <time.h>
#include <stdint.h>
#include <cfloat>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <set>
#include <queue>

// Standard C++ headers and Eigen headers BEFORE Windows.h and Coin3D headers
#include <Eigen/Dense>

// Windows headers with macro protection
#define NOMINMAX
#include <windows.h>

// Coin3D/SoWin headers
#include <Inventor/Win/SoWin.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoFaceSet.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/events/SoKeyboardEvent.h>
#include <Inventor/events/SoMouseButtonEvent.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/SoPickedPoint.h>
#include <Inventor/actions/SoRayPickAction.h>
#include <Inventor/actions/SoHandleEventAction.h>
#include <Inventor/SbVec3f.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoPointSet.h>

// Project headers
#include "Mesh.h"
#include "Painter.h"
#include "helpers.h"

using namespace std;

// Forward declarations
int getRandVertexPoint(Mesh* mesh);
float getDistance(float* refCoord, float* targetCoord);
float* Dijkstra(float** M, int src, int targ, int N, int* prev);
int minDistance(float* dist, bool* marked, int N);
DWORD WINAPI ConsoleInputThread(LPVOID lpParam);

// Global variables to store mesh and painter
Mesh* g_mesh = nullptr;
Painter* g_painter = nullptr;

// Global variables for visualization
SoSeparator* g_root = nullptr;
SoWinExaminerViewer* g_viewer = nullptr;

// Forward declarations
void visualizeGeodesicPath(int source, int target);
void visualizeFarthestPointSampling(int numSamples);
void loadNewMesh(const string& meshPath);
void computeAndVisualizePatches(const string& meshPath);
void testPairwiseHarmonics();
SoSeparator* computeAndVisualizeSymmetry(Mesh* mesh, Painter* painter);

/**
 * Load a new mesh and prepare it for visualization
 * @param meshPath Path to the mesh file (.off format)
 */
void loadNewMesh(const string& meshPath) {
	// Clear previous visualization
	while (g_root->getNumChildren() > 0) {
		g_root->removeChild(0);
	}
	
	// Delete old mesh if it exists
	if (g_mesh) {
		delete g_mesh;
	}
	
	// Create new mesh
	g_mesh = new Mesh();
	
	// Load the mesh
	cout << "Loading mesh from " << meshPath << "..." << endl;
	g_mesh->loadOff(const_cast<char*>(meshPath.c_str()));
	g_mesh->normalizeCoordinates();
	
	cout << "Mesh loaded with " << g_mesh->verts.size() << " vertices." << endl;
	cout << "Vertex indices range from 0 to " << g_mesh->verts.size() - 1 << endl;

	// Add the base mesh visualization
	g_root->addChild(g_painter->getShapeSep(g_mesh));
	
	// Update viewer
	g_viewer->viewAll();
}

/**
 * Compute geodesic path between two vertices and return the path vertices
 * @param source Source vertex index
 * @param target Target vertex index
 * @return Vector of vertex indices forming the path
 */
vector<int> computeGeodesicPath(int source, int target) {
	vector<int> path;
	int N = g_mesh->verts.size();
	
	// Validate input
	if (source < 0 || source >= N || target < 0 || target >= N || source == target) {
		return path;
	}
	
	// Compute shortest path
	int* prev = g_mesh->findShortestPath(source, N);
	
	if (prev) {
		// Reconstruct path
		int at = target;
		while (at != source && at != -1) {
			path.push_back(at);
			at = prev[at];
		}
		
		if (at == source) {
			path.push_back(source);
		}
		
		// Reverse to get from source to target
		reverse(path.begin(), path.end());
		
		delete[] prev;
	}
	
	return path;
}

/**
 * Find intersection between two geodesic paths
 * @param path1 First path as vector of vertex indices
 * @param path2 Second path as vector of vertex indices
 * @param intersectionPoint Output parameter for the intersection point
 * @return True if an intersection exists, false otherwise
 */
bool findIntersection(const vector<int>& path1, const vector<int>& path2, int& intersectionPoint) {
	for (int v1 : path1) {
		for (int v2 : path2) {
			if (v1 == v2) {
				intersectionPoint = v1;
				return true;
			}
		}
	}
	return false;
}

/**
 * Compute and visualize the geodesic path between two vertices
 * @param source Source vertex index
 * @param target Target vertex index
 */
void visualizeGeodesicPath(int source, int target) {
	// Clear previous visualization (except base mesh)
	while (g_root->getNumChildren() > 1) {
		g_root->removeChild(1);
	}
	
	int N = g_mesh->verts.size();
	
	// Validate input
	if (source < 0 || source >= N || target < 0 || target >= N) {
		cout << "Error: Invalid vertex indices!" << endl;
		return;
	}
	
	if (source == target) {
		cout << "Error: Source and target are the same vertex!" << endl;
		return;
	}
	
	// Compute and visualize path
	cout << "Computing geodesic path from vertex " << source << " to vertex " << target << "..." << endl;
	int* prev = g_mesh->findShortestPath(source, N);
	
	if (prev) {
		g_root->addChild(g_painter->DrawLines(g_mesh, source, target, N, prev));
		
		// Highlight source and target with different colors
		SoSeparator* pointsSep = new SoSeparator();
		
		// Source (green)
		pointsSep->addChild(g_painter->get1PointSep(g_mesh, source, 0, 0, 0, 1.0f, false));
		
		// Target (blue)
		SoSeparator* targetSep = g_painter->get1PointSep(g_mesh, target, 0, 0, 0, 1.0f, false);
		SoMaterial* targetMat = new SoMaterial();
		targetMat->diffuseColor.setValue(0, 0, 1); // Blue
		targetSep->addChild(targetMat);
		pointsSep->addChild(targetSep);
	
		g_root->addChild(pointsSep);
		
		delete[] prev;
		cout << "Path visualization complete." << endl;
	} else {
		cout << "Failed to compute path!" << endl;
	}
	
	// Update viewer
	g_viewer->viewAll();
}

/**
 * Visualize the farthest point sampling algorithm on the mesh
 * @param numSamples Number of sample points to compute
 */
void visualizeFarthestPointSampling(int numSamples) {
	// Clear previous visualization (except base mesh)
	while (g_root->getNumChildren() > 1) {
		g_root->removeChild(1);
	}
	
	int N = g_mesh->verts.size();
	
	// Validate input
	if (numSamples < 2 || numSamples > N) {
		cout << "Error: Number of samples must be between 2 and " << N << endl;
		numSamples = min(max(numSamples, 2), N);
		cout << "Using " << numSamples << " samples instead." << endl;
	}
	
	// Compute samples
	cout << "Computing farthest point sampling with " << numSamples << " points..." << endl;
	vector<int> samples = g_mesh->farthestPointSampling(numSamples);
	
	if (samples.size() != numSamples) {
		cout << "Warning: Requested " << numSamples << " samples, but got " << samples.size() << endl;
	}
	
	// Highlight sample points
	SoSeparator* pointsSep = new SoSeparator();
	for (int sample : samples) {
		pointsSep->addChild(g_painter->get1PointSep(g_mesh, sample, 0, 1, 0, 1.0f, false));
	}
	g_root->addChild(pointsSep);
	
	// Draw paths between sample points
	for (size_t i = 0; i < samples.size(); i++) {
		for (size_t j = i + 1; j < samples.size(); j++) {
			int source = samples[i];
			int target = samples[j];
			
			int* prev = g_mesh->findShortestPath(source, N);
			g_root->addChild(g_painter->DrawLines(g_mesh, source, target, N, prev));
			delete[] prev;
		}
	}
	
	cout << "Sample visualization complete." << endl;
	
	// Update viewer
	g_viewer->viewAll();
}

/**
 * Find the optimal order for vertices to form a loop with minimal unwanted intersections
 * @param fpsPoints Vector of farthest point sampling vertices
 * @param mesh Pointer to the mesh
 * @return Vector of vertices in optimal order
 */
vector<int> findOptimalLoopOrder(vector<int>& fpsPoints, Mesh* mesh) {
	// Keep first point fixed and try permutations of others
	vector<int> bestOrder = {fpsPoints[0], fpsPoints[1], fpsPoints[2], fpsPoints[3]};
	int minIntersections = INT_MAX;
	
	// Store all possible permutations (keeping first point fixed)
	vector<vector<int>> permutations;
	vector<int> remaining = {fpsPoints[1], fpsPoints[2], fpsPoints[3]};
	
	// Generate all permutations
	do {
		vector<int> order = {fpsPoints[0]}; // Keep first point fixed
		order.insert(order.end(), remaining.begin(), remaining.end());
		permutations.push_back(order);
	} while (next_permutation(remaining.begin(), remaining.end()));
	
	// Check each permutation for unwanted intersections
	for (const vector<int>& order : permutations) {
		// Compute paths between consecutive points (including last to first)
		vector<vector<int>> loopPaths;
		for (int i = 0; i < order.size(); i++) {
			int source = order[i];
			int target = order[(i + 1) % order.size()]; // Wrap around to first
			
			vector<int> path = computeGeodesicPath(source, target);
			if (!path.empty()) {
				loopPaths.push_back(path);
			}
		}
		
		// Count unwanted intersections (shared vertices between non-adjacent paths)
		int intersections = 0;
		for (int i = 0; i < loopPaths.size(); i++) {
			for (int j = i + 2; j < loopPaths.size(); j++) {
				// Skip adjacent paths (i and i+1)
				if ((i == 0 && j == loopPaths.size() - 1)) {
					// First and last are adjacent in the loop
					continue;
				}
				
				// Check for shared vertices (except FPS points)
				for (int v1 : loopPaths[i]) {
					if (find(fpsPoints.begin(), fpsPoints.end(), v1) != fpsPoints.end()) {
						continue; // Skip FPS points
					}
					
					for (int v2 : loopPaths[j]) {
						if (v1 == v2) {
							intersections++;
							break; // Found one intersection point between these paths
						}
					}
				}
			}
		}
		
		// Update best order if this has fewer intersections
		if (intersections < minIntersections) {
			minIntersections = intersections;
			bestOrder = order;
			
			// If we found a perfect loop (no unwanted intersections), return it immediately
			if (intersections == 0) {
				cout << "Found perfect loop with no unwanted intersections!" << endl;
				return bestOrder;
			}
		}
	}
	
	cout << "Best loop order has " << minIntersections << " unwanted intersections." << endl;
	return bestOrder;
}

/**
 * Compute and visualize two types of patch maps:
 * 1. MAP 1: Star-like paths from geodesic center to FPS points
 * 2. MAP 2: Loop paths between FPS points in optimal order
 * 
 * @param meshPath Path to the mesh file (.off format)
 */
void computeAndVisualizePatches(const string& meshPath) {
	// Load mesh
	loadNewMesh(meshPath);
	
	// Clear previous visualization
	while (g_root->getNumChildren() > 1) {
		g_root->removeChild(1);
	}
	
	cout << "Computing geodesic paths and intersections..." << endl;
	
	// Get farthest point samples
	int numFPS = 4;
	vector<int> samples = g_mesh->farthestPointSampling(numFPS);
	
	if (samples.size() != numFPS) {
		cout << "Error: Failed to compute " << numFPS << " sample points." << endl;
		return;
	}
	
	int N = g_mesh->verts.size();
	
	// Find the optimal loop order
	vector<int> optimalOrder = findOptimalLoopOrder(samples, g_mesh);
	
	cout << "Optimal loop order: ";
	for (int idx : optimalOrder) {
		cout << idx << " ";
	}
	cout << endl;
	
	// Use the optimal order for MAP 2 instead of the hardcoded order
	cout << "Creating MAP 2: Loop paths between FPS points in optimal order..." << endl;
	
	// Create a separator for the loop paths
	SoSeparator* map2Sep = new SoSeparator();
	SoMaterial* map2Mat = new SoMaterial();
	map2Mat->diffuseColor.setValue(0, 0, 1); // Blue for loop paths
	map2Sep->addChild(map2Mat);
	
	// Vector to collect path vertices for patch
	vector<vector<int>> paths;
	
	for (int i = 0; i < numFPS; i++) {
		int source = optimalOrder[i];
		int target = optimalOrder[(i + 1) % numFPS]; // Connect to next in optimal order
		
		// First, get path vertices for patch boundary
		vector<int> path = computeGeodesicPath(source, target);
		if (!path.empty()) {
			paths.push_back(path);
		}
		
		// Find shortest path and visualize
		int* prev = g_mesh->findShortestPath(source, N);
		if (prev) {
			// Create and add visualization of path
			map2Sep->addChild(g_painter->DrawLines(g_mesh, source, target, N, prev));
			delete[] prev;
			cout << "Added path from " << source << " to " << target << endl;
		} else {
			cout << "Failed to compute path from " << source << " to " << target << "!" << endl;
		}
	}
	
	// Add the loop paths separator to the scene
	g_root->addChild(map2Sep);
	
	// Create patch boundaries using the corner points
	vector<vector<int>> patchBoundaries;
	
	// If we have 4 valid paths forming a closed loop
	if (paths.size() == numFPS) {
		vector<int> quadCorners = optimalOrder;
		
		// Make sure the quadrilateral is properly oriented (try to prevent fold-overs)
		SbVec3f centroid(0, 0, 0);
		for (int idx : quadCorners) {
			centroid += SbVec3f(g_mesh->verts[idx]->coords);
		}
		centroid /= quadCorners.size();
		
		// Calculate vectors from centroid to corners for orientation check
		vector<SbVec3f> cornerVecs;
		for (int idx : quadCorners) {
			SbVec3f vec = SbVec3f(g_mesh->verts[idx]->coords) - centroid;
			vec.normalize();
			cornerVecs.push_back(vec);
		}
		
		// Check if we need to reorder based on orientation
		float orientation = (cornerVecs[0].cross(cornerVecs[1])).dot(cornerVecs[2].cross(cornerVecs[3]));
		if (orientation < 0) {
			// Swap points 2 and 3 to improve orientation
			swap(quadCorners[2], quadCorners[3]);
			cout << "Adjusted corner ordering for better patch orientation" << endl;
		}
		
		patchBoundaries.push_back(quadCorners);
		
		// Visualize the interpolated patches
		SoSeparator* patchesSep = g_painter->visualizePatches(g_mesh, patchBoundaries);
		g_root->addChild(patchesSep);
		cout << "Added interpolated patch visualization" << endl;
	} else {
		cout << "Warning: Could not create a complete closed loop with 4 paths." << endl;
		cout << "Found " << paths.size() << " valid paths out of " << numFPS << " needed." << endl;
	}
	
	// MAP 1: Create a separator for the center-to-FPS geodesic paths (star pattern)
	SoSeparator* map1Sep = new SoSeparator();
	SoMaterial* map1Mat = new SoMaterial();
	map1Mat->diffuseColor.setValue(0, 1, 0); // Green for map1 (star) paths
	map1Sep->addChild(map1Mat);
	
	// Collect all vertices in all paths
	vector<int> allPathVertices;
	
	// MAP 1: Create star paths from geodesic center to each FPS point
	cout << "Creating MAP 1: Star paths from geodesic center to FPS points..." << endl;
	for (int i = 0; i < numFPS; i++) {
		int target = samples[i];
		
		// Compute geodesic path from center to FPS point
		vector<int> path = computeGeodesicPath(samples[0], target);
		
		if (!path.empty()) {
			// Add path vertices to overall collection
			allPathVertices.insert(allPathVertices.end(), path.begin(), path.end());
			
			// Visualize this path segment
			int* prev = g_mesh->findShortestPath(samples[0], N);
			if (prev) {
				map1Sep->addChild(g_painter->DrawLines(g_mesh, samples[0], target, N, prev));
				delete[] prev;
				cout << "Added MAP 1 path from geodesic center to FPS " << i 
					 << " with " << path.size() << " vertices" << endl;
			}
		} else {
			cout << "Failed to compute path from center to " << target << "!" << endl;
		}
	}
	
	// Add both map separators to scene
	g_root->addChild(map1Sep);
	
	// Highlight all vertices along paths with small green points
	SoSeparator* allPointsSep = new SoSeparator();
	SoMaterial* pointsMat = new SoMaterial();
	pointsMat->diffuseColor.setValue(0, 1, 0); // Green
	allPointsSep->addChild(pointsMat);
	
	// Add individual points for every vertex in all geodesic paths
	for (int vertIdx : allPathVertices) {
		allPointsSep->addChild(g_painter->get1PointSep(g_mesh, vertIdx, 0, 1, 0, 3.0f, false));
	}
	g_root->addChild(allPointsSep);
	
	// Highlight geodesic center in purple
	SoSeparator* centerSep = new SoSeparator();
	SoMaterial* centerMat = new SoMaterial();
	centerMat->diffuseColor.setValue(1, 0, 1); // Purple for center
	centerSep->addChild(centerMat);
	centerSep->addChild(g_painter->get1PointSep(g_mesh, samples[0], 1, 0, 1, 10.0f, false));
	g_root->addChild(centerSep);
	cout << "Highlighted geodesic center (purple)" << endl;
	
	// Highlight FPS points with distinct larger points
	SoSeparator* fpsSep = new SoSeparator();
	for (int i = 0; i < samples.size(); i++) {
		// Different color for each FPS point
		float r = (i == 0) ? 1.0f : 0.0f;
		float g = (i == 1) ? 1.0f : 0.0f;
		float b = (i == 2) ? 1.0f : 0.0f;
		
		if (i == 3) r = g = 1.0f; // Yellow for last point
		
		fpsSep->addChild(g_painter->get1PointSep(g_mesh, samples[i], r, g, b, 8.0f, false));
		cout << "FPS " << i << " is vertex " << samples[i] << endl;
	}
	g_root->addChild(fpsSep);
	
	cout << "Complete patch visualization with:" << endl;
	cout << " - MAP 1: Green star paths from center to FPS points" << endl;
	cout << " - MAP 2: Blue loop paths between FPS points in optimal order" << endl;
	cout << " - Total vertices: " << allPathVertices.size() << endl;
	
	// Update viewer
	g_viewer->viewAll();
}

/**
 * Test and visualize pairwise harmonic functions for intrinsic symmetry analysis
 * Allows the user to select two vertices and visualizes the resulting harmonic field
 */
void testPairwiseHarmonics() {
    // Clear previous visualization (except base mesh)
    while (g_root->getNumChildren() > 1) {
        g_root->removeChild(1);
    }
    
    int nVerts = g_mesh->verts.size();
    
    // Ask user for two vertices to compute the pairwise harmonic between
    int p_idx, q_idx;
    cout << "Enter first vertex index (p) - value 0 (0-" << nVerts - 1 << "): ";
    cin >> p_idx;
    cout << "Enter second vertex index (q) - value 1 (0-" << nVerts - 1 << "): ";
    cin >> q_idx;
    
    // Validate input
    if (p_idx < 0 || p_idx >= nVerts || q_idx < 0 || q_idx >= nVerts || p_idx == q_idx) {
        cout << "Error: Invalid vertex indices or same vertex selected!" << endl;
        return;
    }
    
    cout << "Computing pairwise harmonic function between vertices " << p_idx << " and " << q_idx << "..." << endl;
    
    // Compute and visualize the pairwise harmonic
    SoSeparator* harmonicVisualization = testPairwiseHarmonic(g_mesh, p_idx, q_idx, g_painter);
    g_root->addChild(harmonicVisualization);
    
    cout << "Pairwise harmonic visualization complete." << endl;
    cout << "Blue areas are close to vertex " << p_idx << " (value 0)" << endl;
    cout << "Red areas are close to vertex " << q_idx << " (value 1)" << endl;
    
    // Update viewer
    g_viewer->viewAll();
}

SoSeparator* computeAndVisualizeSymmetry(Mesh* mesh, Painter* painter){
    SoSeparator* symmetryResult = new SoSeparator();

    // Add mesh visualization
    symmetryResult->addChild(painter->getShapeSep(mesh));

    int numFpsSamples = 20; // Choose number of points for FPS
    std::cout << "Running Farthest Point Sampling with " << numFpsSamples << " samples..." << std::endl;
    std::vector<int> fps_samples = mesh->farthestPointSampling(numFpsSamples);
    std::cout << "FPS completed. Got " << fps_samples.size() << " samples." << std::endl;

    if (fps_samples.size() < 2) {
        std::cerr << "Need at least 2 FPS samples to form pairs." << std::endl;
        return nullptr;
    }

    // Ensure numFpsSamples reflects the actual number obtained
    numFpsSamples = fps_samples.size();

    PairwiseHarmonics ph(mesh);
    std::cout << "Computing Cotangent Laplacian..." << std::endl;
    if(!ph.computeCotangentLaplacian()){
        std::cerr << "Failed to compute Laplacian!" << std::endl;
        return nullptr;
    }
    std::cout << "Laplacian computed." << std::endl;

    int numSamplesK = 10; // K for R/D descriptors
    double pis_threshold = 0.7; // PIS threshold for voting
    double sigma_gaussian = pis_threshold / 2.0; // Sigma for vote weighting

    // --- Calculate Geodesic Distance Threshold for Filtering ---
    float bbox_diagonal = mesh->getBoundingBoxDiagonal();
    float estimated_max_geo_dist = bbox_diagonal * 0.5f;
    float dmin_threshold = estimated_max_geo_dist / 5.0f;
    std::cout << "Bounding Box Diagonal: " << bbox_diagonal << std::endl;
    std::cout << "Estimated Max Geodesic Distance: " << estimated_max_geo_dist << std::endl;
    std::cout << "Geodesic Distance Filter Threshold (d_min): " << dmin_threshold << std::endl;

    // --- Precompute Geodesic Distances from all FPS samples ---
    std::cout << "Precomputing geodesic distances from " << numFpsSamples << " FPS points..." << std::endl;
    std::vector<float*> all_fps_distances(numFpsSamples, nullptr);
    int N = mesh->verts.size();
    bool precomputation_ok = true;
    for (int s = 0; s < numFpsSamples; ++s) {
        int source_vertex = fps_samples[s];
        all_fps_distances[s] = mesh->computeGeodesicDistances(source_vertex, N);
        if (!all_fps_distances[s]) {
            std::cerr << "Error: Failed to compute geodesic distances from FPS sample " << source_vertex << std::endl;
            precomputation_ok = false;
            // Clean up already allocated arrays before breaking
            for(int k=0; k<s; ++k) delete[] all_fps_distances[k];
            all_fps_distances.clear();
            break;
        }
    }
    if (!precomputation_ok) {
         std::cerr << "Aborting symmetry computation due to geodesic precomputation failure." << std::endl;
         return nullptr;
    }
    std::cout << "Geodesic precomputation complete." << std::endl;
    // --- End Precomputation ---


    std::vector<double> face_votes(mesh->tris.size(), 0.0);
    int pair_count = 0;
    int pairs_filtered = 0;

    std::cout << "Processing " << (numFpsSamples * (numFpsSamples - 1) / 2) << " candidate pairs..." << std::endl;

    // Iterate through unique pairs from FPS samples
    for (size_t i = 0; i < numFpsSamples; ++i){
        for(size_t j = i + 1; j < numFpsSamples; ++j){
            int p = fps_samples[i];
            int q = fps_samples[j];

            pair_count++;

            if (pair_count % 10 == 0) {
                std::cout << "Processing pair " << pair_count << " (" << p << ", " << q << ")" << std::endl;
            }

            // --- Use Precomputed Distances for Filtering ---
            float* dist_p_raw = all_fps_distances[i]; // Get precomputed distances from p
            // float* dist_q_raw = all_fps_distances[j]; // Get precomputed distances from q (needed later for D_qp)

            // Should not be null if precomputation check passed, but check anyway
            if (!dist_p_raw) {
                 std::cerr << "Internal Error: Missing precomputed distance for " << p << ". Skipping pair." << std::endl;
                 continue;
            }

            float geo_dist_pq = dist_p_raw[q]; // Look up distance p->q

            // --- Remove the redundant Dijkstra call and delete[] ---
            // float* dist_p_raw = mesh->computeGeodesicDistances(p, N);
            // if (!dist_p_raw) { ... }
            // float geo_dist_pq = dist_p_raw[q];
            // delete[] dist_p_raw; // REMOVED
            // --- End Removal ---


            if (geo_dist_pq == FLT_MAX) {
                 std::cout << "  Skipping pair (" << p << ", " << q << ") - vertices unreachable." << std::endl;
                 pairs_filtered++;
                 continue;
            }

            if (geo_dist_pq < dmin_threshold) {
                 std::cout << "  Skipping pair (" << p << ", " << q << ") - Geodesic distance " << geo_dist_pq << " < " << dmin_threshold << std::endl;
                 pairs_filtered++;
                 continue;
            }
            // --- End Filtering ---

            // 1. Compute fields f_pq and f_qp
            Eigen::VectorXd f_pq = ph.computePairwiseHarmonic(p, q);
            Eigen::VectorXd f_qp = ph.computePairwiseHarmonic(q, p);

            if (f_pq.size() == 0 || f_qp.size() == 0) {
                std::cerr << "Skipping pair (" << p << ", " << q << ") due to harmonic computation failure." << std::endl;
                continue;
            }

            // 2. Compute R and D descriptors
            // Get the other distance array needed
            float* dist_q_raw = all_fps_distances[j];
             if (!dist_q_raw) { // Safety check
                 std::cerr << "Internal Error: Missing precomputed distance for " << q << ". Skipping pair." << std::endl;
                 continue;
             }

            Eigen::VectorXd R_pq = ph.computeRDescriptor(f_pq, numSamplesK);
            // Pass precomputed distances to D descriptor function
            Eigen::VectorXd D_pq = ph.computeDDescriptor(p, q, f_pq, numSamplesK, dist_p_raw, dist_q_raw);

            Eigen::VectorXd R_qp = ph.computeRDescriptor(f_qp, numSamplesK);
            // Pass precomputed distances (swapped order) to D descriptor function
            Eigen::VectorXd D_qp = ph.computeDDescriptor(q, p, f_qp, numSamplesK, dist_q_raw, dist_p_raw);


            // Check if descriptors are valid
            if (R_pq.size() != numSamplesK || D_pq.size() != numSamplesK ||
                R_qp.size() != numSamplesK || D_qp.size() != numSamplesK) {
                std::cerr << " Skipping pair (" << p << ", " << q << ") due to descriptor computation failure." << std::endl;
                continue;
            }

            // 3. Compute PIS score
            double pis_score = ph.computePIS(R_pq, D_pq, R_qp, D_qp);

            // 4. Check threshold and vote
            if (pis_score > pis_threshold){
                // ... (voting logic remains the same) ...
                std::cout << "  Pair (" << p << ", " << q << ") has high PIS score: " << pis_score << ". Voting..." << std::endl;
                double vote_weight = std::exp(-std::pow(pis_score - 1.0, 2) / (2.0 * sigma_gaussian * sigma_gaussian));
                double mid_isoValue = 0.5;
                std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> mid_segments = ph.extractIsoCurveSegments(f_pq, mid_isoValue);
                for (const auto& segment : mid_segments) {
                    Eigen::Vector3d segment_midpoint = (segment.first + segment.second) * 0.5;
                    int closest_face_idx = mesh->findClosestFace(segment_midpoint);
                    if (closest_face_idx != -1) {
                        face_votes[closest_face_idx] += vote_weight;
                    }
                }
            }
        } // End inner loop (j)
    } // End outer loop (i)

    std::cout << "Finished processing pairs. Filtered out " << pairs_filtered << " pairs based on geodesic distance." << std::endl;

    // --- Clean up precomputed distance arrays ---
    std::cout << "Cleaning up precomputed distances..." << std::endl;
    for (float* dist_array : all_fps_distances) {
        if (dist_array) {
            delete[] dist_array;
        }
    }
    all_fps_distances.clear();
    std::cout << "Cleanup complete." << std::endl;
    // --- End Cleanup ---


    // --- Extract and Visualize Symmetry Axis/Regions ---
    // ... (visualization logic remains the same) ...
    std::cout << "Analyzing votes..." << std::endl;
    double max_vote = 0.0;
    for (double vote : face_votes) {
        if (vote > max_vote) {
            max_vote = vote;
        }
    }
    std::cout << "Max vote: " << max_vote << std::endl;

    if (max_vote > 1e-6) {
        SoSeparator* symmetryAxisSep = new SoSeparator();
        SoMaterial* axisMat = new SoMaterial();
        axisMat->diffuseColor.setValue(0.0f, 1.0f, 0.0f); // Green
        symmetryAxisSep->addChild(axisMat);
        SoCoordinate3* axisCoords = new SoCoordinate3();
        SoIndexedFaceSet* axisFaces = new SoIndexedFaceSet();
        std::vector<int> vertex_map(mesh->verts.size(), -1);
        int axis_coord_idx = 0;
        int axis_face_vtx_idx = 0;
        double vote_display_threshold = max_vote * 0.5;

        for (size_t face_idx = 0; face_idx < face_votes.size(); ++face_idx) {
            if (face_votes[face_idx] >= vote_display_threshold) {
                Triangle* tri = mesh->tris[face_idx];
                int v_indices[] = {tri->v1i, tri->v2i, tri->v3i};
                for (int v_idx : v_indices) {
                    if (vertex_map[v_idx] == -1) {
                        vertex_map[v_idx] = axis_coord_idx;
                        axisCoords->point.set1Value(axis_coord_idx++, mesh->verts[v_idx]->coords);
                    }
                    axisFaces->coordIndex.set1Value(axis_face_vtx_idx++, vertex_map[v_idx]);
                }
                axisFaces->coordIndex.set1Value(axis_face_vtx_idx++, -1);
            }
        }
        symmetryAxisSep->addChild(axisCoords);
        symmetryAxisSep->addChild(axisFaces);
        symmetryResult->addChild(symmetryAxisSep);
        std::cout << "Added symmetry axis visualization." << std::endl;
    } else {
         std::cout << "No significant symmetry votes found." << std::endl;
    }


    return symmetryResult;
}

/**
 * Display the application menu
 */
void displayMenu() {
    cout << "\n=== Mesh Processing Menu ===" << endl;
    cout << "1. Compute geodesic path between two vertices" << endl;
    cout << "2. Visualize farthest point sampling" << endl;
    cout << "3. Compute and visualize patches (using mesh 249.off)" << endl;
    cout << "4. Compute and visualize patches (using mesh Armadillo.off)" << endl;
    cout << "5. Test pairwise harmonics (Phase 1 & 2 Test)" << endl;
    cout << "6. Compute and visualize intrinsic symmetry (Phase 3)" << endl; // New option
    cout << "7. Exit" << endl; // Renumber Exit
    cout << "Enter your choice (1-7): "; // Update range
}

/**
 * Main application entry point
 */
int main(int, char** argv)
{
	srand(time(NULL));

	// Coin3D Initializations
	HWND window = SoWin::init(argv[0]);
	if (window == NULL)
		exit(1);
	
	g_root = new SoSeparator;
	g_root->ref();

	// Create the mesh and painter
	g_mesh = new Mesh();
	g_painter = new Painter();

	// Load the mesh
	string meshPath = "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/hw2/man0.off";
	cout << "Loading mesh from " << meshPath << "..." << endl;
	g_mesh->loadOff(const_cast<char*>(meshPath.c_str()));
	g_mesh->normalizeCoordinates();
	
	cout << "Mesh loaded with " << g_mesh->verts.size() << " vertices." << endl;
	cout << "Vertex indices range from 0 to " << g_mesh->verts.size() - 1 << endl;

	// Add the base mesh visualization
	g_root->addChild(g_painter->getShapeSep(g_mesh));

	// Set up viewer
	g_viewer = new SoWinExaminerViewer(window);
	g_viewer->setSceneGraph(g_root);
	g_viewer->setTitle("Assignment 1 - Mesh Processing");
	g_viewer->setSize(SbVec2s(800, 600));
	g_viewer->show();
	g_viewer->viewAll();

	// Display initial menu
	displayMenu();
	
	// Create a separate thread for handling console input
	HANDLE consoleThread = CreateThread(
		NULL,                   // default security attributes
		0,                      // use default stack size  
		ConsoleInputThread,     // thread function name
		NULL,                   // argument to thread function 
		0,                      // use default creation flags 
		NULL);                  // returns the thread identifier 

	if (consoleThread == NULL) {
		cout << "Error creating console thread!" << endl;
	}
	
	// Start the Coin3D event loop in the main thread
	SoWin::show(window);
	SoWin::mainLoop();
	
	// Clean up - this will be reached when the GUI window is closed
	CloseHandle(consoleThread);
	delete g_viewer;
	delete g_mesh;
	delete g_painter;
	g_root->unref();

	return 0;
}

/**
 * Thread function to handle console input
 * Runs separately from the main rendering thread
 */
DWORD WINAPI ConsoleInputThread(LPVOID lpParam)
{
    int choice = 0;

    while (choice != 7) {  // Update exit condition to 7
        cin >> choice;

        // Clear input buffer after reading choice
        cin.ignore(numeric_limits<streamsize>::max(), '\n');


        switch (choice) {
            case 1: { // Geodesic path
                int source, target;
                cout << "Enter source vertex (0-" << g_mesh->verts.size() - 1 << "): ";
                cin >> source;
                cout << "Enter target vertex (0-" << g_mesh->verts.size() - 1 << "): ";
                cin >> target;
                // Clear input buffer after reading numbers
                cin.ignore(numeric_limits<streamsize>::max(), '\n');

                visualizeGeodesicPath(source, target);
                break;
            }

            case 2: { // Farthest point sampling
                int numSamples;
                cout << "Enter number of samples (2-" << g_mesh->verts.size() << "): ";
                cin >> numSamples;
                cin.ignore(numeric_limits<streamsize>::max(), '\n');

                visualizeFarthestPointSampling(numSamples);
                break;
            }

            case 3: { // Patching with 249.off
                string meshPath = "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/hw2/249.off";
                computeAndVisualizePatches(meshPath);
                break;
            }

            case 4: { // Patching with Armadillo.off
                string meshPath = "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/hw2/Armadillo.off";
                computeAndVisualizePatches(meshPath);
                break;
            }

            case 5: { // Test pairwise harmonics
                testPairwiseHarmonics();
                break;
            }

            case 6: { // Compute and visualize symmetry
                if (g_mesh && g_painter) {
                     // Clear previous visualization (except base mesh)
                     while (g_root->getNumChildren() > 1) {
                         g_root->removeChild(1);
                     }
                     // Call the function and add the result to the main scene graph
                     SoSeparator* symmetryViz = computeAndVisualizeSymmetry(g_mesh, g_painter);
                     if (symmetryViz) { // Check if it returned something
                        g_root->addChild(symmetryViz); // Add to the global root
                     }
                     g_viewer->viewAll(); // Update viewer after computation
                } else {
                    std::cout << "Error: Mesh or painter not initialized." << std::endl;
                }
                break;
            }

            case 7: // Exit
                cout << "Exiting program." << endl;
                SoWin::exitMainLoop(); // Request exit from the main Coin3D loop
                break;

            default:
                cout << "Invalid choice. Please try again." << endl;
                break;
        }

        if (choice != 7) {  // Update condition to 7
            displayMenu(); // Show the menu again for next input
        }
    }

    return 0;
}

/**
 * Find the vertex with the minimum distance that hasn't been marked yet
 * @param dist Array of distances
 * @param marked Array indicating whether vertices have been processed
 * @param N Number of vertices
 * @return Index of the unprocessed vertex with minimum distance
 */
int minDistance(float* dist, bool* marked, int N)
{
	float min = INT_MAX, min_index;

	for (int v = 0; v < N; v++)
		if (marked[v] == false && dist[v] <= min)
			min = dist[v], min_index = v;

	return min_index;
}

/**
 * Dijkstra's algorithm to find shortest paths in a graph
 * @param M Adjacency matrix with edge weights
 * @param src Source vertex index
 * @param targ Target vertex index
 * @param N Number of vertices
 * @param prev Output array to store predecessors
 * @return Array of distances from source to all vertices
 */
float* Dijkstra(float** M, int src, int targ, int N, int* prev)
{
	float* dist = new float[N];
	bool* marked = new bool[N];
	for (int i = 0; i < N; i++)
	{
		prev[i] = -1; // Initialize predecessor
		dist[i] = FLT_MAX; // Initialize distance to infinity
		marked[i] = false; // Initialize all vertices as unprocessed
	}
	dist[src] = 0; // Distance from source to itself is 0

	for (int count = 0; count < N - 1; count++)
	{
		int u = minDistance(dist, marked, N);

		marked[u] = true;

		// Update distances to neighbors
		for (int v = 0; v < N; v++) {
			if (!marked[v] && M[u][v] && (dist[u] + M[u][v] < dist[v]))
			{
				prev[v] = u;
				dist[v] = dist[u] + M[u][v];
			}
		}
	}
	delete[] marked;
	return dist;
}

/**
 * Get a random vertex from the mesh
 * @param mesh Pointer to the mesh
 * @return Random vertex index
 */
int getRandVertexPoint(Mesh* mesh) {
	int randV1 = rand() % (mesh->verts.size());
	return randV1;
}

/**
 * Calculate Euclidean distance between two 3D points
 * @param refCoord First point coordinates
 * @param targetCoord Second point coordinates
 * @return Euclidean distance
 */
float getDistance(float* refCoord, float* targetCoord) {
	float distance = (
		pow(refCoord[0] - targetCoord[0], 2) +
		pow(refCoord[1] - targetCoord[1], 2) +
		pow(refCoord[2] - targetCoord[2], 2));

	return sqrt(distance);
}