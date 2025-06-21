#pragma once

#include <vector>
#include <cstdio>
#include <Eigen/Dense>
#include <Eigen/Sparse>

// LibIGL includes for robust geometry processing
#include <igl/cotmatrix.h>
#include <igl/massmatrix.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/adjacency_list.h>
#include <igl/grad.h>
#include <igl/principal_curvature.h>
#include <igl/gaussian_curvature.h>

// Additional LibIGL includes for enhanced geometry processing
#include <igl/heat_geodesics.h>
#include <igl/exact_geodesic.h>
#include <igl/avg_edge_length.h>
#include <igl/doublearea.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>
#include <igl/barycenter.h>
#include <igl/boundary_loop.h>
#include <igl/slice.h>
#include <igl/slice_into.h>

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

	// LibIGL data structures for robust geometry processing
	Eigen::MatrixXd V;       // Vertex positions (#V x 3)
	Eigen::MatrixXi F;       // Face indices (#F x 3)
	Eigen::SparseMatrix<double> L;   // Cotangent Laplacian matrix
	Eigen::SparseMatrix<double> M;   // Mass matrix
	std::vector<std::vector<int>> VV; // Vertex-vertex adjacency list
	std::vector<std::vector<int>> VF; // Vertex-face adjacency list
	std::vector<std::vector<int>> VFi; // Vertex-face-index adjacency list

	// Enhanced LibIGL structures for advanced computations
	Eigen::VectorXd face_areas;      // Face areas for mesh analysis
	Eigen::MatrixXd face_normals;    // Face normals
	Eigen::MatrixXd vertex_normals;  // Vertex normals (computed via LibIGL)
	Eigen::MatrixXd face_barycenters; // Face barycenters
	double avg_edge_length;          // Average edge length for heat geodesics

	// Heat geodesics precomputed data structures
	igl::HeatGeodesicsData<double> heat_data; // Precomputed heat geodesics data
	bool heat_geodesics_precomputed;         // Flag for heat geodesics initialization

    // Curvature information (computed on demand)
    Eigen::VectorXd gaussian_curvature;
    Eigen::VectorXd mean_curvature;
    Eigen::MatrixXd principal_directions1, principal_directions2;
    Eigen::VectorXd principal_curvatures1, principal_curvatures2;

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
	Mesh() : heat_geodesics_precomputed(false), avg_edge_length(0.0) {};

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

	/**
	 * @brief Finds the index of the mesh face closest to a given 3D point.
	 *
	 * It first checks for faces where the point's projection onto the face plane
	 * lies inside the triangle. If found, it returns the face with the minimum
	 * projection distance. If not found, it returns the face containing the
	 * edge or vertex closest to the point.
	 *
	 * @param point The query point in 3D space.
	 * @return The index of the closest face in mesh->tris, or -1 if the mesh has no faces.
	 */
	Eigen::Vector3d closestPointOnSegment(const Eigen::Vector3d& point, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);

	/**
	 * @brief Finds the index of the mesh face closest to a given 3D point.
	 *
	 * It first checks for faces where the point's projection onto the face plane
	 * lies inside the triangle. If found, it returns the face with the minimum
	 * projection distance. If not found, it returns the face containing the
	 * edge or vertex closest to the point.
	 *
	 * @param point The query point in 3D space.
	 * @return The index of the closest face in mesh->tris, or -1 if the mesh has no faces.
	 */

	 int findClosestFace(const Eigen::Vector3d& point);
	/**
	 * @brief Computes the length of the diagonal of the mesh's axis-aligned bounding box.
	 * @return The length of the bounding box diagonal. Returns 0 if mesh has no vertices.
	 */
	float getBoundingBoxDiagonal() const;

	// ========== LibIGL Integration Methods ==========

	/**
	 * Convert mesh data to LibIGL format (V, F matrices)
	 * This should be called after loading a mesh to enable LibIGL functionality
	 */
	void convertToLibIGL();

	/**
	 * Compute cotangent Laplacian matrix using LibIGL
	 * Much more robust than manual implementation
	 * @return True if successful, false otherwise
	 */
	bool computeCotangentLaplacianLibIGL();

	/**
	 * Compute mass matrix using LibIGL
	 * @return True if successful, false otherwise
	 */
	bool computeMassMatrixLibIGL();

	/**
	 * Compute vertex adjacency information using LibIGL
	 */
	void computeAdjacencyLibIGL();

	/**
	 * Compute curvature information using LibIGL
	 * Includes Gaussian and mean curvature, principal curvatures and directions
	 */
	void computeCurvatureLibIGL();

	/**
	 * Precompute heat geodesics data for fast distance computation
	 * This is much faster than Dijkstra for multiple queries
	 * @return True if successful, false otherwise
	 */
	bool precomputeHeatGeodesics();

	/**
	 * Compute geodesic distances from a source vertex using heat method
	 * Much faster and more accurate than Dijkstra's algorithm
	 * @param source_vertex Index of source vertex
	 * @return Vector of distances to all vertices
	 */
	Eigen::VectorXd computeHeatGeodesicDistances(int source_vertex);

	/**
	 * Compute geodesic distances between multiple source vertices
	 * @param source_vertices Vector of source vertex indices
	 * @return Matrix where each column contains distances from one source
	 */
	Eigen::MatrixXd computeMultipleHeatGeodesicDistances(const std::vector<int>& source_vertices);

	/**
	 * Compute mesh face areas using LibIGL
	 */
	void computeFaceAreasLibIGL();

	/**
	 * Compute face and vertex normals using LibIGL
	 */
	void computeNormalsLibIGL();

	/**
	 * Compute face barycenters using LibIGL
	 */
	void computeBarycenterLibIGL();

	/**
	 * Enhanced farthest point sampling using LibIGL heat geodesics
	 * Much faster and more accurate than original implementation
	 * @param num_samples Number of samples to generate
	 * @param seed_vertex Optional seed vertex (default: random)
	 * @return Vector of sampled vertex indices
	 */
	std::vector<int> farthestPointSamplingLibIGL(int num_samples, int seed_vertex = -1);

	/**
	 * Solve Laplace equation with boundary conditions using LibIGL
	 * Much more robust than manual sparse solving
	 * @param boundary_vertices Indices of boundary vertices
	 * @param boundary_values Values at boundary vertices
	 * @return Solution vector for all vertices
	 */
	Eigen::VectorXd solveLaplaceEquationLibIGL(
        const std::vector<int>& boundary_vertices,
        const std::vector<double>& boundary_values
    );

	/**
	 * Compute discrete gradient operator using LibIGL
	 * @return Gradient matrix
	 */
	Eigen::SparseMatrix<double> computeGradientLibIGL();

	/**
	 * Get LibIGL vertex matrix (const access)
	 */
	const Eigen::MatrixXd& getVertexMatrix() const { return V; }

	/**
	 * Get LibIGL face matrix (const access)
	 */
	const Eigen::MatrixXi& getFaceMatrix() const { return F; }

	/**
	 * Get cotangent Laplacian matrix (const access)
	 */
	const Eigen::SparseMatrix<double>& getLaplacianMatrix() const { return L; }

	/**
	 * Get mass matrix (const access)
	 */
	const Eigen::SparseMatrix<double>& getMassMatrix() const { return M; }

	/**
	 * Get vertex normals computed by LibIGL
	 */
	const Eigen::MatrixXd& getVertexNormals() const { return vertex_normals; }

	/**
	 * Get face normals computed by LibIGL
	 */
	const Eigen::MatrixXd& getFaceNormals() const { return face_normals; }

	/**
	 * Get average edge length
	 */
	double getAverageEdgeLength() const { return avg_edge_length; }
};