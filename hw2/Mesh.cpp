#include "pch.h"  // Include precompiled header first

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <limits>
#include <cmath>
#include <Eigen/Dense>
#include <limits> // Required for std::numeric_limits
#include <cmath> // For std::sqrt

// Project headers
#include "Mesh.h"
#include "helpers.h"

using namespace std;
/**
 * Load a mesh from an OFF file
 * @param name Path to the .off file
 */
void Mesh::loadOff(char* name)
{
	FILE* fPtr = fopen(name, "r");
	if (!fPtr){
		printf("Error opening file %s\n", name);
		exit(1);
	}
	
	char str[334];

	fscanf(fPtr, "%s", str);

	int nVerts, nTris, n, i = 0;
	float x, y, z;

	fscanf(fPtr, "%d %d %d\n", &nVerts, &nTris, &n);
	while (i++ < nVerts)
	{
		fscanf(fPtr, "%f %f %f", &x, &y, &z);
		addVertex(x, y, z);
	}

	while (fscanf(fPtr, "%d", &i) != EOF)
	{
		fscanf(fPtr, "%f %f %f", &x, &y, &z);
		addTriangle((int) x, (int) y, (int) z);
		makeVertsNeighbor(x, y);
		makeVertsNeighbor(y, z);
		makeVertsNeighbor(x, z);
	}

	fclose(fPtr);
}

/**
 * Create a cube mesh with the specified side length
 * @param sideLen Length of the cube side
 */
void Mesh::createCube(float sideLen)
{
	// Create cube corners
	float flbc[3] = {0, 0, 0}, deltaX = 0, deltaY = 0, deltaZ = 0;
	for (int v = 0; v < 8; v++)
	{
		switch (v)
		{
			case 1:
				deltaX = sideLen;
				break;
			case 2:
				deltaZ = -sideLen;
				break;
			case 3:
				deltaX = 0;
				break;
			case 4:
				deltaZ = 0;
				deltaY = sideLen;
				break;
			case 5:
				deltaX = sideLen;
				break;
			case 6:
				deltaZ = -sideLen;
				break;
			default:
				deltaX = 0;;
				break;
		}
		addVertex(flbc[0] + deltaX, flbc[1] + deltaY, flbc[2] + deltaZ);
	}

	// Create cube faces
	addTriangle(0, 2, 1);
	addTriangle(0, 3, 2);

	addTriangle(1, 2, 5);
	addTriangle(2, 6, 5);

	addTriangle(2, 3, 6);
	addTriangle(3, 7, 6);

	addTriangle(3, 4, 7);
	addTriangle(3, 0, 4);

	addTriangle(4, 5, 6);
	addTriangle(4, 6, 7);

	addTriangle(0, 1, 5);
	addTriangle(0, 5, 4);
}

/**
 * Add a triangle to the mesh and update connectivity information
 * @param v1 First vertex index
 * @param v2 Second vertex index
 * @param v3 Third vertex index
 */
void Mesh::addTriangle(int v1, int v2, int v3)
{
	int idx = tris.size();
	tris.push_back( new Triangle(idx, v1, v2, v3) );

	// Update vertex-triangle connectivity
	verts[v1]->triList.push_back(idx);
	verts[v2]->triList.push_back(idx);
	verts[v3]->triList.push_back(idx);

	// Create or update connectivity between vertices
	if (! makeVertsNeighbor(v1, v2) )
		addEdge(v1, v2);

	if (! makeVertsNeighbor(v1, v3) )
		addEdge(v1, v3);

	if (! makeVertsNeighbor(v2, v3) )
		addEdge(v2, v3);
}

/**
 * Make two vertices neighbors by updating their adjacency lists
 * @param v1i First vertex index
 * @param v2i Second vertex index
 * @return True if the vertices were already neighbors, false otherwise
 */
bool Mesh::makeVertsNeighbor(int v1i, int v2i)
{
	// Check if already neighbors
	for (int i = 0; i < verts[v1i]->vertList.size(); i++)
		if (verts[v1i]->vertList[i] == v2i)
			return true;

	// Add to each other's neighbor lists
	verts[v1i]->vertList.push_back(v2i);
	verts[v2i]->vertList.push_back(v1i);
	return false;
}

/**
 * Add a vertex to the mesh
 * @param x X coordinate
 * @param y Y coordinate
 * @param z Z coordinate
 */
void Mesh::addVertex(float x, float y, float z)
{
	int idx = verts.size();
	float* c = new float[3];
	c[0] = x;
	c[1] = y;
	c[2] = z;

	verts.push_back( new Vertex(idx, c) );
}

/**
 * Add an edge to the mesh
 * @param v1 First vertex index
 * @param v2 Second vertex index
 */
void Mesh::addEdge(int v1, int v2)
{
	int idx = edges.size();

	edges.push_back( new Edge(idx, v1, v2) );

	verts[v1]->edgeList.push_back(idx);
	verts[v2]->edgeList.push_back(idx);
}

/**
 * Compute geodesic distances from a source vertex to all other vertices
 * @param source Source vertex index
 * @param N Output parameter for number of vertices
 * @return Array of distances from source to all vertices
 */
float* Mesh::computeGeodesicDistances(int source, int& N) {
    N = verts.size();
    float* dist = new float[N];
    bool* visited = new bool[N];
    
    // Initialize distances
    for (int i = 0; i < N; i++) {
        dist[i] = FLT_MAX;
        visited[i] = false;
    }
    
    dist[source] = 0;
    
    // Dijkstra's algorithm implementation
    for (int count = 0; count < N-1; count++) {
        // Find vertex with minimum distance
        int u = -1;
        float minDist = FLT_MAX;
        for (int i = 0; i < N; i++) {
            if (!visited[i] && dist[i] < minDist) {
                minDist = dist[i];
                u = i;
            }
        }
        
        if (u == -1) break;
        
        visited[u] = true;
        
        // Update distances to neighbors
        for (int i = 0; i < verts[u]->vertList.size(); i++) {
            int v = verts[u]->vertList[i];
            
            // Compute actual edge length for more accurate geodesics
            float edgeLength = 0.0f;
            for (int j = 0; j < edges.size(); j++) {
                if ((edges[j]->v1i == u && edges[j]->v2i == v) || 
                    (edges[j]->v1i == v && edges[j]->v2i == u)) {
                    edgeLength = edges[j]->length;
                    break;
                }
            }
            
            if (edgeLength == 0.0f) {
                // Compute Euclidean distance if edge length isn't pre-computed
                float dx = verts[u]->coords[0] - verts[v]->coords[0];
                float dy = verts[u]->coords[1] - verts[v]->coords[1];
                float dz = verts[u]->coords[2] - verts[v]->coords[2];
                edgeLength = sqrt(dx*dx + dy*dy + dz*dz);
            }
            
            if (!visited[v] && dist[u] != FLT_MAX && dist[u] + edgeLength < dist[v]) {
                dist[v] = dist[u] + edgeLength;
            }
        }
    }
    
    delete[] visited;
    return dist;
}

/**
 * Find the shortest path from a source vertex to all other vertices
 * Uses Dijkstra's algorithm to compute paths
 * 
 * @param source Source vertex index
 * @param N Output parameter for number of vertices
 * @return Array of predecessors for path reconstruction, must be freed by caller
 */
int* Mesh::findShortestPath(int source, int& N) {
    N = verts.size();
    float* dist = new float[N];
    int* prev = new int[N];
    bool* visited = new bool[N];
    
    // Initialize data structures
    for (int i = 0; i < N; i++) {
        dist[i] = FLT_MAX;
        prev[i] = -1;
        visited[i] = false;
    }
    
    dist[source] = 0;
    
    // Dijkstra's algorithm
    for (int count = 0; count < N; count++) {
        // Find unvisited vertex with minimum distance
        int u = -1;
        float minDist = FLT_MAX;
        for (int i = 0; i < N; i++) {
            if (!visited[i] && dist[i] < minDist) {
                minDist = dist[i];
                u = i;
            }
        }
        
        if (u == -1) break; // No more reachable vertices
        
        visited[u] = true;
        
        // Process neighbors with bounds checking
        for (size_t i = 0; i < verts[u]->vertList.size(); i++) {
            int v = verts[u]->vertList[i];
            
            // Verify v is a valid index
            if (v < 0 || v >= N) continue;
            
            // Calculate edge length (Euclidean distance)
            float dx = verts[u]->coords[0] - verts[v]->coords[0];
            float dy = verts[u]->coords[1] - verts[v]->coords[1];
            float dz = verts[u]->coords[2] - verts[v]->coords[2];
            float edgeLength = sqrt(dx*dx + dy*dy + dz*dz);
            
            // Update distance if shorter path found
            if (!visited[v] && dist[u] != FLT_MAX && dist[u] + edgeLength < dist[v]) {
                dist[v] = dist[u] + edgeLength;
                prev[v] = u;
            }
        }
    }
    
    // Clean up memory
    delete[] dist;
    delete[] visited;
    
    return prev; // Remember to free this in the calling function!
}

/**
 * Compute the geodesic distance matrix and save to file
 * Contains distances between all pairs of vertices
 * 
 * @param outputFile Path to output file
 */
void Mesh::computeGeodesicDistanceMatrix(const char* outputFile) {
    int N = verts.size();
    float** distMatrix = new float*[N];
    
    // Compute distances from each vertex to all others
    for (int i = 0; i < N; i++) {
        distMatrix[i] = new float[N];
        int dummy;
        float* distances = computeGeodesicDistances(i, dummy);
        for (int j = 0; j < N; j++) {
            distMatrix[i][j] = distances[j];
        }
        delete[] distances;
    }
    
    // Write to file
    FILE* file = fopen(outputFile, "w");
    if (file) {
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                fprintf(file, "%f ", distMatrix[i][j]);
            }
            fprintf(file, "\n");
        }
        fclose(file);
    }
    
    // Clean up
    for (int i = 0; i < N; i++) {
        delete[] distMatrix[i];
    }
    delete[] distMatrix;
}

/**
 * Farthest Point Sampling algorithm
 * Selects points that are geodesically far from each other
 * 
 * @param numSamples Number of sample points to compute
 * @param seedVertex Index of the initial seed vertex (default: 0)
 * @return Vector of vertex indices of the sampled points
 */
vector<int> Mesh::farthestPointSampling(int numSamples, int seedVertex) {
    int N = verts.size();
    vector<int> samples;
    
    if (numSamples <= 0 || numSamples > N)
        return samples;
    
    // Initialize with the seed vertex
    samples.push_back(seedVertex);
    
    // Array to track minimum distance from each vertex to any sample
    float* minDistToSamples = new float[N];
    for (int i = 0; i < N; i++) {
        minDistToSamples[i] = FLT_MAX;
    }
    
    // Update distances from the seed
    int dummy;
    float* distances = computeGeodesicDistances(seedVertex, dummy);
    for (int i = 0; i < N; i++) {
        minDistToSamples[i] = distances[i];
    }
    delete[] distances;
    
    // Main FPS algorithm loop
    for (int i = 1; i < numSamples; i++) {
        // Find the farthest vertex from all current samples
        int farthestVertex = -1;
        float maxDist = -1.0f;
        
        for (int v = 0; v < N; v++) {
            if (minDistToSamples[v] > maxDist) {
                maxDist = minDistToSamples[v];
                farthestVertex = v;
            }
        }
        
        if (farthestVertex == -1) break;
        
        // Add new sample
        samples.push_back(farthestVertex);
        
        // Update minimum distances to include new sample
        distances = computeGeodesicDistances(farthestVertex, dummy);
        for (int v = 0; v < N; v++) {
            minDistToSamples[v] = min(minDistToSamples[v], distances[v]);
        }
        delete[] distances;
    }
    
    delete[] minDistToSamples;
    return samples;
}

/**
 * Find intersections between two geodesic paths
 * @param path1 First path as vector of vertex indices
 * @param path2 Second path as vector of vertex indices
 * @return Vector of intersection information
 */
vector<Mesh::PathIntersection> Mesh::findPathIntersections(vector<int>& path1, vector<int>& path2) {
    vector<PathIntersection> intersections;
    
    // For each segment in path1
    for (int i = 0; i < path1.size() - 1; i++) {
        int v1 = path1[i];
        int v2 = path1[i+1];
        
        // Line segment from v1 to v2
        float* p1 = verts[v1]->coords;
        float* p2 = verts[v2]->coords;
        
        // For each segment in path2
        for (int j = 0; j < path2.size() - 1; j++) {
            int v3 = path2[j];
            int v4 = path2[j+1];
            
            // Line segment from v3 to v4
            float* p3 = verts[v3]->coords;
            float* p4 = verts[v4]->coords;
            
            // Note: Full 3D line segment intersection code would be implemented here
            // Currently this is a placeholder for future implementation
        }
    }
    
    return intersections;
}

/**
 * Utility function to convert from prev array to path vector
 * @param prev Array of predecessors
 * @param source Source vertex index
 * @param target Target vertex index
 * @param N Number of vertices
 * @return Vector of vertex indices forming the path
 */
vector<int> getPathVector(int* prev, int source, int target, int N) {
    vector<int> path;
    
    // Safety check 
    if (!prev || target < 0 || target >= N) {
        return path;
    }
    
    // Build path with safety limit to prevent infinite loops
    int at = target;
    int safety = 0;
    int maxSteps = N; // Maximum number of steps should not exceed number of vertices
    
    while (at != -1 && at != source && safety < maxSteps) {
        path.push_back(at);
        at = prev[at];
        safety++;
    }
    
    // Add source if found
    if (at == source) {
        path.push_back(source);
    }
    
    // Reverse to get path from source to target
    reverse(path.begin(), path.end());
    return path;
}

/**
 * Create patches from a set of sampled vertices
 * @param sampledVertices Vector of sampled vertex indices
 */
void Mesh::createPatches(vector<int>& sampledVertices) {
    // For each quadruple of sample points
    for (int i = 0; i < sampledVertices.size(); i++) {
        for (int j = i + 1; j < sampledVertices.size(); j++) {
            for (int k = j + 1; k < sampledVertices.size(); k++) {
                for (int l = k + 1; l < sampledVertices.size(); l++) {
                    int s1 = sampledVertices[i];
                    int s2 = sampledVertices[j];
                    int s3 = sampledVertices[k];
                    int s4 = sampledVertices[l];
                    
                    // Compute the 6 paths between these 4 points
                    int N;
                    int* prev1_2 = findShortestPath(s1, N);
                    int* prev1_3 = findShortestPath(s1, N);
                    int* prev1_4 = findShortestPath(s1, N);
                    int* prev2_3 = findShortestPath(s2, N);
                    int* prev2_4 = findShortestPath(s2, N);
                    int* prev3_4 = findShortestPath(s3, N);
                    
                    // Convert to path vectors
                    vector<int> path1_2 = getPathVector(prev1_2, s1, s2, verts.size());
                    vector<int> path1_3 = getPathVector(prev1_3, s1, s3, verts.size());
                    vector<int> path1_4 = getPathVector(prev1_4, s1, s4, verts.size());
                    vector<int> path2_3 = getPathVector(prev2_3, s2, s3, verts.size());
                    vector<int> path2_4 = getPathVector(prev2_4, s2, s4, verts.size());
                    vector<int> path3_4 = getPathVector(prev3_4, s3, s4, verts.size());
                    
                    // Note: Path intersection analysis would be implemented here
                    
                    // Clean up
                    delete[] prev1_2;
                    delete[] prev1_3;
                    delete[] prev1_4;
                    delete[] prev2_3;
                    delete[] prev2_4;
                    delete[] prev3_4;
                }
            }
        }
    }
}

/**
 * Normalize mesh coordinates to fit within a standard bounding box
 * Centers the mesh and scales it to fit within [-1,1] range
 */
void Mesh::normalizeCoordinates() {
    // Find bounding box
    float minX = FLT_MAX, minY = FLT_MAX, minZ = FLT_MAX;
    float maxX = -FLT_MAX, maxY = -FLT_MAX, maxZ = -FLT_MAX;
    
    for (unsigned int i = 0; i < verts.size(); i++) {
        minX = min(minX, verts[i]->coords[0]);
        minY = min(minY, verts[i]->coords[1]);
        minZ = min(minZ, verts[i]->coords[2]);
        
        maxX = max(maxX, verts[i]->coords[0]);
        maxY = max(maxY, verts[i]->coords[1]);
        maxZ = max(maxZ, verts[i]->coords[2]);
    }
    
    // Calculate center and scale
    float centerX = (minX + maxX) / 2.0f;
    float centerY = (minY + maxY) / 2.0f;
    float centerZ = (minZ + maxZ) / 2.0f;
    
    float maxDim = max(max(maxX - minX, maxY - minY), maxZ - minZ);
    float scale = 2.0f / maxDim;  // Scale to fit in [-1,1] range
    
    // Apply transformation: center and scale
    for (unsigned int i = 0; i < verts.size(); i++) {
        verts[i]->coords[0] = (verts[i]->coords[0] - centerX) * scale;
        verts[i]->coords[1] = (verts[i]->coords[1] - centerY) * scale;
        verts[i]->coords[2] = (verts[i]->coords[2] - centerZ) * scale;
    }
    
    printf("Mesh normalized: Scale=%f, Center=(%f,%f,%f)\n", 
           scale, centerX, centerY, centerZ);
}
double Mesh::calculateAngleAtVertex(Triangle* tri, int vertexIdx) {
    int v1i = tri->v1i;
    int v2i = tri->v2i;
    int v3i = tri->v3i;

    Vertex *v1 = verts[v1i];
    Vertex *v2 = verts[v2i];
    Vertex *v3 = verts[v3i];

    Eigen::Vector3d p1(v1->coords[0], v1->coords[1], v1->coords[2]);
    Eigen::Vector3d p2(v2->coords[0], v2->coords[1], v2->coords[2]);
    Eigen::Vector3d p3(v3->coords[0], v3->coords[1], v3->coords[2]);

    Eigen::Vector3d vecA, vecB;

    if (vertexIdx == v1i) {
        vecA = p2 - p1;
        vecB = p3 - p1;
    } else if (vertexIdx == v2i) {
        vecA = p1 - p2;
        vecB = p3 - p2;
    } else if (vertexIdx == v3i) {
        vecA = p1 - p3;
        vecB = p2 - p3;
    } else {
        // Should not happen if vertexIdx is part of the triangle
        return 0.0;
    }

    double normA = vecA.norm();
    double normB = vecB.norm();

    if (normA < 1e-10 || normB < 1e-10) {
        return 0.0; // Degenerate case
    }

    double dot = vecA.dot(vecB);
    double cosTheta = dot / (normA * normB);

    // Clamp cosTheta to avoid numerical issues with acos
    cosTheta = std::max(-1.0, std::min(1.0, cosTheta));

    return std::acos(cosTheta);

}

Eigen::Vector3d Mesh::compteNormal(int vertexIdx) {

    if(vertexIdx < 0 || vertexIdx >= verts.size()) {
        std::cerr << "Error: Invalid vertex index in computeNormal: " << vertexIdx << std::endl;
        return Eigen::Vector3d::Zero();
    
    }
    Vertex* v = verts[vertexIdx];
    Eigen::Vector3d accumulatedNormal = Eigen::Vector3d::Zero();
    double totalAngle = 0.0; // Keep track for potential normalization issues, though not strictly needed for direction

    if (v->triList.empty()) {
         // Handle isolated vertex case (optional: return default normal like Y-up?)
         std::cerr << "Warning: Vertex " << vertexIdx << " has no adjacent triangles. Cannot compute normal." << std::endl;
         return Eigen::Vector3d(0, 1, 0); // Or Zero()
    }


    for (int triIdx : v->triList) {
        if (triIdx < 0 || triIdx >= tris.size()) continue; // Safety check

        Triangle* tri = tris[triIdx];
        Vertex* v1 = verts[tri->v1i];
        Vertex* v2 = verts[tri->v2i];
        Vertex* v3 = verts[tri->v3i];

        Eigen::Vector3d p1(v1->coords[0], v1->coords[1], v1->coords[2]);
        Eigen::Vector3d p2(v2->coords[0], v2->coords[1], v2->coords[2]);
        Eigen::Vector3d p3(v3->coords[0], v3->coords[1], v3->coords[2]);

        // Calculate face normal (counter-clockwise order assumed for OFF files)
        Eigen::Vector3d edge1 = p2 - p1;
        Eigen::Vector3d edge2 = p3 - p1;
        Eigen::Vector3d faceNormal = edge1.cross(edge2);

        // Calculate angle at the vertex for this triangle
        double angle = calculateAngleAtVertex(tri, vertexIdx);

        // Check for degenerate triangles/normals or zero angles
        if (faceNormal.squaredNorm() > 1e-12 && angle > 1e-6) {
             accumulatedNormal += faceNormal.normalized() * angle; // Weight face normal by angle
             totalAngle += angle; // Optional: track total angle
        } else {
             // Optionally handle degenerate face normal or zero angle contribution
             // std::cerr << "Warning: Degenerate face or zero angle for vertex " << vertexIdx << " in triangle " << triIdx << std::endl;
        }
    }

    // Normalize the accumulated normal
    if (accumulatedNormal.squaredNorm() > 1e-12) {
        return accumulatedNormal.normalized();
    } else {
        // Handle cases where accumulation resulted in zero vector (e.g., highly symmetric configurations or all degenerate faces)
        // Fallback: Simple average of face normals (less robust) or default normal
        std::cerr << "Warning: Could not compute robust normal for vertex " << vertexIdx << ". Returning default normal." << std::endl;
        // As a simple fallback, compute non-weighted average if needed, or return default
        Eigen::Vector3d fallbackNormal = Eigen::Vector3d::Zero();
         for (int triIdx : v->triList) {
             if (triIdx < 0 || triIdx >= tris.size()) continue;
             Triangle* tri = tris[triIdx];
             Vertex* v1 = verts[tri->v1i]; Vertex* v2 = verts[tri->v2i]; Vertex* v3 = verts[tri->v3i];
             Eigen::Vector3d p1(v1->coords[0], v1->coords[1], v1->coords[2]);
             Eigen::Vector3d p2(v2->coords[0], v2->coords[1], v2->coords[2]);
             Eigen::Vector3d p3(v3->coords[0], v3->coords[1], v3->coords[2]);
             Eigen::Vector3d faceNormal = (p2 - p1).cross(p3 - p1);
             if (faceNormal.squaredNorm() > 1e-12) fallbackNormal += faceNormal.normalized();
         }
         if (fallbackNormal.squaredNorm() > 1e-12) return fallbackNormal.normalized();
         else return Eigen::Vector3d(0, 1, 0); // Final fallback: Y-up
    }
}

/**
 * Computes the geometric centroid of a given face (triangle).
 * @param face_idx The index of the triangle in the mesh->tris vector.
 * @return The 3D coordinates of the centroid as an Eigen::Vector3d.
 *         Returns a zero vector if the face index is invalid.
 */
Eigen::Vector3d Mesh::getFaceCentroid(int face_idx) {
    if (face_idx < 0 || face_idx >= tris.size()) {
        std::cerr << "Error: Invalid face index in getFaceCentroid." << face_idx << std::endl;
        return Eigen::Vector3d::Zero();
    }
    Triangle* tri = tris[face_idx];
    Vertex* v1 = verts[tri->v1i];
    Vertex* v2 = verts[tri->v2i];
    Vertex* v3 = verts[tri->v3i];

    Eigen::Vector3d p1(v1->coords[0], v1->coords[1], v1->coords[2]);
    Eigen::Vector3d p2(v2->coords[0], v2->coords[1], v2->coords[2]);
    Eigen::Vector3d p3(v3->coords[0], v3->coords[1], v3->coords[2]);

    return (p1+p2 +p3)/3.0; // Return centroid
}

/**
 * @brief Computes the closest point on a line segment (p1, p2) to a given point.
 *
 * @param point The query point.
 * @param p1 Start point of the segment.
 * @param p2 End point of the segment.
 * @return The closest point on the segment [p1, p2] to 'point'.
 */
Eigen::Vector3d Mesh::closestPointOnSegment(const Eigen::Vector3d& point, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
    Eigen::Vector3d segmentDir = p2 - p1;
    double segmentLenSq = segmentDir.squaredNorm();

    // If segment has zero length, return one of the endpoints
    if (segmentLenSq < 1e-12) {
        return p1;
    }

    // Project point onto the line defined by the segment
    // t = dot(point - p1, segmentDir) / segmentLenSq
    double t = (point - p1).dot(segmentDir) / segmentLenSq;

    // Clamp t to [0, 1] to stay within the segment
    t = std::max(0.0, std::min(1.0, t));

    // Calculate the closest point on the segment
    return p1 + t * segmentDir;
}

/**
 * @brief Checks if a point projected onto the plane of a triangle lies inside the triangle.
 * Uses barycentric coordinates.
 *
 * @param point The point to check (assumed to be on the plane of the triangle).
 * @param v1 Vertex 1 of the triangle.
 * @param v2 Vertex 2 of the triangle.
 * @param v3 Vertex 3 of the triangle.
 * @param tolerance Tolerance for barycentric coordinate checks.
 * @return true if the point is inside or on the boundary of the triangle, false otherwise.
 */
bool isPointInTriangle(const Eigen::Vector3d& point, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3, double tolerance = 1e-6) {
    Eigen::Vector3d edge1 = v2 - v1;
    Eigen::Vector3d edge2 = v3 - v1;
    Eigen::Vector3d pointVec = point - v1;

    double dot11 = edge1.dot(edge1);
    double dot12 = edge1.dot(edge2);
    double dot22 = edge2.dot(edge2);
    double dotP1 = pointVec.dot(edge1);
    double dotP2 = pointVec.dot(edge2);

    double invDenom = dot11 * dot22 - dot12 * dot12;

    // Check for degenerate triangle (collinear vertices)
    if (std::abs(invDenom) < tolerance) {
        // Fallback: Check if point is close to any segment? For simplicity, return false.
        return false;
    }

    invDenom = 1.0 / invDenom;
    double u = (dot22 * dotP1 - dot12 * dotP2) * invDenom; // Barycentric coord for edge1
    double v = (dot11 * dotP2 - dot12 * dotP1) * invDenom; // Barycentric coord for edge2

    // Check if point is within the triangle boundaries (including edges/vertices)
    return (u >= -tolerance) && (v >= -tolerance) && (u + v <= 1.0 + tolerance);
}


int Mesh::findClosestFace(const Eigen::Vector3d& point) {
    if (tris.empty()) {
        return -1;
    }

    double minProjectionDistSq = std::numeric_limits<double>::max();
    int closestProjectionFaceIdx = -1;

    double minBoundaryDistSq = std::numeric_limits<double>::max();
    int closestBoundaryFaceIdx = -1;

    for (size_t i = 0; i < tris.size(); ++i) {
        Triangle* tri = tris[i];
        Vertex* v1_ptr = verts[tri->v1i];
        Vertex* v2_ptr = verts[tri->v2i];
        Vertex* v3_ptr = verts[tri->v3i];

        Eigen::Vector3d v1(v1_ptr->coords[0], v1_ptr->coords[1], v1_ptr->coords[2]);
        Eigen::Vector3d v2(v2_ptr->coords[0], v2_ptr->coords[1], v2_ptr->coords[2]);
        Eigen::Vector3d v3(v3_ptr->coords[0], v3_ptr->coords[1], v3_ptr->coords[2]);

        // Calculate face normal and plane distance
        Eigen::Vector3d edge1 = v2 - v1;
        Eigen::Vector3d edge2 = v3 - v1;
        Eigen::Vector3d faceNormal = edge1.cross(edge2);
        double faceNormalLenSq = faceNormal.squaredNorm();

        if (faceNormalLenSq < 1e-12) continue; // Skip degenerate triangles

        faceNormal /= std::sqrt(faceNormalLenSq); // Normalize
        double planeDist = faceNormal.dot(point - v1); // Signed distance to plane

        // Project point onto the plane
        Eigen::Vector3d projectedPoint = point - planeDist * faceNormal;

        // Check if the projected point is inside the triangle
        if (isPointInTriangle(projectedPoint, v1, v2, v3)) {
            double distSq = planeDist * planeDist; // Squared distance to the plane (and projected point)
            if (distSq < minProjectionDistSq) {
                minProjectionDistSq = distSq;
                closestProjectionFaceIdx = static_cast<int>(i);
            }
        } else {
            // If projection is outside, find closest point on triangle boundary
            Eigen::Vector3d cp12 = closestPointOnSegment(point, v1, v2);
            Eigen::Vector3d cp23 = closestPointOnSegment(point, v2, v3);
            Eigen::Vector3d cp31 = closestPointOnSegment(point, v3, v1);

            double dSq12 = (point - cp12).squaredNorm();
            double dSq23 = (point - cp23).squaredNorm();
            double dSq31 = (point - cp31).squaredNorm();

            double minDistBoundaryCurrentFace = std::min({dSq12, dSq23, dSq31});

            if (minDistBoundaryCurrentFace < minBoundaryDistSq) {
                minBoundaryDistSq = minDistBoundaryCurrentFace;
                closestBoundaryFaceIdx = static_cast<int>(i);
            }
        }
    }

    // Prefer faces where the projection landed inside
    if (closestProjectionFaceIdx != -1) {
        return closestProjectionFaceIdx;
    } else {
        // Otherwise, return the face with the closest boundary point
        return closestBoundaryFaceIdx;
    }
}

/**
 * @brief Computes the length of the diagonal of the mesh's axis-aligned bounding box.
 * @return The length of the bounding box diagonal. Returns 0 if mesh has no vertices.
 */
float Mesh::getBoundingBoxDiagonal() const {
    if (verts.empty()) {
        return 0.0f;
    }

    // Find bounding box
    float minX = std::numeric_limits<float>::max(), minY = std::numeric_limits<float>::max(), minZ = std::numeric_limits<float>::max();
    float maxX = -std::numeric_limits<float>::max(), maxY = -std::numeric_limits<float>::max(), maxZ = -std::numeric_limits<float>::max();

    for (const Vertex* v : verts) {
        minX = std::min(minX, v->coords[0]);
        minY = std::min(minY, v->coords[1]);
        minZ = std::min(minZ, v->coords[2]);

        maxX = std::max(maxX, v->coords[0]);
        maxY = std::max(maxY, v->coords[1]);
        maxZ = std::max(maxZ, v->coords[2]);
    }

    // Calculate diagonal length
    float dx = maxX - minX;
    float dy = maxY - minY;
    float dz = maxZ - minZ;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

