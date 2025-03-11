#include <stdio.h>
#include <cstdlib>
#include <time.h>
#include <stdint.h>
#include <cfloat>
#include <iostream>
#include <string>
#include <windows.h>
#include <vector>
#include <algorithm>
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
#include "Mesh.h"
#include "Painter.h"
#include <set>
#include <queue>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoPointSet.h>

using namespace std;

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
void computeAndVisualizePatches();

// Function to load a new mesh
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

// Function to compute geodesic path and return the path vertices
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

// Function to detect intersection between two geodesic paths
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

// Function to update the visualization with geodesic path
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

// Function to visualize farthest point sampling
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
	
	// Draw paths between sample points (optional)
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

// Function to find the optimal loop order with minimal unwanted intersections
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

// Function to compute and visualize patches
void computeAndVisualizePatches() {
	// Load mesh
	string meshPath = "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/hw1src/348.off";
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
		
		// Use findShortestPath correctly - here's the fix:
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
		// (This is a simple heuristic - could be improved)
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

// Function to display the menu
void displayMenu() {
	cout << "\n=== Mesh Processing Menu ===" << endl;
	cout << "1. Compute geodesic path between two vertices" << endl;
	cout << "2. Visualize farthest point sampling" << endl;
	cout << "3. Compute and visualize patches (using mesh from segeval)" << endl;
	cout << "4. Exit" << endl;
	cout << "Enter your choice (1-4): ";
}

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
	string meshPath = "C:/Users/cagopa/Desktop/Digital-Geometry-Processing/hw1src/man0.off";
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

// Console Input Thread Function
DWORD WINAPI ConsoleInputThread(LPVOID lpParam) 
{
	int choice = 0;
	
	while (choice != 4) {
		cin >> choice;
		
		switch (choice) {
			case 1: { // Geodesic path
				int source, target;
				cout << "Enter source vertex (0-" << g_mesh->verts.size() - 1 << "): ";
				cin >> source;
				cout << "Enter target vertex (0-" << g_mesh->verts.size() - 1 << "): ";
				cin >> target;
				
				visualizeGeodesicPath(source, target);
				break;
			}
			
			case 2: { // Farthest point sampling
				int numSamples;
				cout << "Enter number of samples (2-" << g_mesh->verts.size() << "): ";
				cin >> numSamples;
				
				visualizeFarthestPointSampling(numSamples);
				break;
			}
			
			case 3: { // Patching
				computeAndVisualizePatches();
				break;
			}
			
			case 4: // Exit
				cout << "Exiting program." << endl;
				SoWin::exitMainLoop(); // Request exit from the main Coin3D loop
				break;
				
			default:
				cout << "Invalid choice. Please try again." << endl;
				break;
		}
		
		if (choice != 4) {
			displayMenu(); // Show the menu again for next input
		}
	}
	
	return 0;
}

int minDistance(float* dist, bool* marked, int N)
{
	// Initialize min value
	float min = INT_MAX, min_index;

	for (int v = 0; v < N; v++)
		if (marked[v] == false && dist[v] <= min)
			min = dist[v], min_index = v;

	return min_index;
}

//Dijkstra algorithm
float* Dijkstra(float** M, int src,int targ, int N, int* prev )
{
	float* dist = new float[N];
	bool* marked = new bool[N];
	for (int i = 0; i < N; i++)
	{
		prev[i] = -1; //source node
		dist[i] = FLT_MAX;
		marked[i] = false;
	}
	dist[src] = 0;

	for (int count = 0; count < N - 1; count++)
	{
		int u = minDistance(dist, marked, N);

		marked[u] = true;

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

int getRandVertexPoint(Mesh* mesh) {

	int randV1 = rand() % (mesh->verts.size());
	return randV1;
}

//returns distance between two 3D points
float getDistance(float* refCoord, float* targetCoord) {
	
	float distance = (
		pow(refCoord[0] - targetCoord[0], 2) +
		pow(refCoord[1] - targetCoord[1], 2) +
		pow(refCoord[2] - targetCoord[2], 2));

	return sqrt(distance);


}