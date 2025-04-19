#pragma once

#include <vector>
#include <cstdio>
using namespace std;

/**
 * Vertex class representing a single point in the mesh
 * Stores coordinates and connectivity information
 */
struct Vertex
{
	float* coords, * normals; // 3D coordinates and normals
	int idx; // Index in the mesh's vertex array

	vector<int> vertList; // Adjacent vertices (neighbors)
	vector<int> triList;  // Triangles containing this vertex
	vector<int> edgeList; // Edges containing this vertex

	Vertex(int i, float* c) : idx(i), coords(c) {};
};

/**
 * Edge class representing a connection between two vertices
 */
struct Edge
{
	int idx; // Index in the mesh's edge array
	int v1i, v2i; // Indices of endpoints
	float length; // Edge length
	
	Edge(int id, int v1, int v2) : idx(id), v1i(v1), v2i(v2) { computeLength(); };

	void computeLength()
	{
		length = 7; // Note: This appears to be a placeholder
	}
};

/**
 * Triangle class representing a face in the mesh
 */
struct Triangle
{
	int idx; // Index in the mesh's triangle array
	int v1i, v2i, v3i; // Indices of the three vertices
	
	Triangle(int id, int v1, int v2, int v3) : idx(id), v1i(v1), v2i(v2), v3i(v3) {};
};

/**
 * Mesh class providing the core mesh data structure and processing algorithms
 * Supports geodesic path computation, farthest point sampling, and patch creation
 */
class Mesh
{
private:
	/**
	 * Add a triangle to the mesh and update connectivity information
	 * @param v1 First vertex index
	 * @param v2 Second vertex index
	 * @param v3 Third vertex index
	 */
	void addTriangle(int v1, int v2, int v3);
	
	/**
	 * Add an edge to the mesh
	 * @param v1 First vertex index
	 * @param v2 Second vertex index
	 */
	void addEdge(int v1, int v2);
	
	/**
	 * Add a vertex to the mesh
	 * @param x X coordinate
	 * @param y Y coordinate
	 * @param z Z coordinate
	 */
	void addVertex(float x, float y, float z);
	
	/**
	 * Make two vertices neighbors by updating their adjacency lists
	 * @param v1i First vertex index
	 * @param v2i Second vertex index
	 * @return True if the vertices were already neighbors, false otherwise
	 */
	bool makeVertsNeighbor(int v1i, int v2i);
	
public:
	vector<Vertex*> verts;   // Mesh vertices
	vector<Triangle*> tris;  // Mesh triangles (faces)
	vector<Edge*> edges;     // Mesh edges

	/**
	 * Structure to store information about path intersections
	 * Used for analyzing where geodesic paths cross
	 */
	struct PathIntersection {
		int segmentIndex1;    // Index in first path
		int segmentIndex2;    // Index in second path
		float parameter1;     // Parameter on first segment (0 to 1)
		float parameter2;     // Parameter on second segment (0 to 1)
		float coords[3];      // 3D coordinates of intersection point
	};

	/**
	 * Default constructor
	 */
	Mesh() {};
	
	/**
	 * Create a simple cube mesh with the specified side length
	 * @param side Length of each side of the cube
	 */
	void createCube(float side);
	
	/**
	 * Load a mesh from an OFF format file
	 * @param name Path to the .off file
	 */
	void loadOff(char* name);
	
	/**
	 * Find the shortest path from a source vertex to all other vertices
	 * @param source Source vertex index
	 * @param N Output parameter for number of vertices
	 * @return Array of predecessors for path reconstruction (caller must free)
	 */
	int* findShortestPath(int source, int& N);
	
	/**
	 * Compute geodesic distances from source vertex to all other vertices
	 * @param source Source vertex index
	 * @param N Output parameter for number of vertices
	 * @return Array of distances from source to all vertices (caller must free)
	 */
	float* computeGeodesicDistances(int source, int& N);
	
	/**
	 * Compute and save the full geodesic distance matrix
	 * @param outputFile Path to output file for distance matrix
	 */
	void computeGeodesicDistanceMatrix(const char* outputFile);
	
	/**
	 * Perform farthest point sampling on the mesh
	 * Selects points that are geodesically far from each other
	 * 
	 * @param numSamples Number of sample points to compute
	 * @param seedVertex Index of initial seed vertex (default: 0)
	 * @return Vector of vertex indices of the sampled points
	 */
	vector<int> farthestPointSampling(int numSamples, int seedVertex = 0);
	
	/**
	 * Find intersections between two geodesic paths
	 * @param path1 First path as vector of vertex indices
	 * @param path2 Second path as vector of vertex indices
	 * @return Vector of intersection information
	 */
	vector<PathIntersection> findPathIntersections(vector<int>& path1, vector<int>& path2);
	
	/**
	 * Create patches from a set of sampled vertices
	 * @param sampledVertices Vector of sampled vertex indices
	 */
	void createPatches(vector<int>& sampledVertices);
	
	/**
	 * Normalize mesh coordinates to fit within a standard bounding box
	 * Centers the mesh and scales it to fit within [-1,1] range
	 */
	void normalizeCoordinates();

    /**
     * Computes the geometric centroid of a given face (triangle).
     * @param face_idx The index of the triangle in the mesh->tris vector.
     * @return The 3D coordinates of the centroid as an Eigen::Vector3d.
     *         Returns a zero vector if the face index is invalid.
     */
    Eigen::Vector3d getFaceCentroid(int face_idx);

	/**
	 * Helper function to calculate the angle at a specific vertex within a triangle.
	 * @param tri Pointer to the Triangle.
	 * @param vertexIdx The index of the vertex within the mesh (not the triangle's local index).
	 * @return The angle in radians at the specified vertex within the triangle. Returns 0 if invalid.
	 */
	double calculateAngleAtVertex(Triangle* tri, int vertexIdx);

	/**
	 * Computes the normal vector for a given vertex using angle-weighted face normals.
	 * @param vertexIdx The index of the vertex.
	 * @return The normalized normal vector as an Eigen::Vector3d.
	 *         Returns a zero vector if the vertex index is invalid or has no adjacent triangles.
	 */
	Eigen::Vector3d compteNormal(int vertexIdx);
};