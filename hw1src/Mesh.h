#pragma once

#include <vector>
#include <cstdio>
using namespace std;

struct Vertex
{
	float* coords, * normals; //3d coordinates etc
	int idx; //who am i; verts[idx]

	vector< int > vertList; //adj vertices
	vector< int > triList;
	vector< int > edgeList;

	Vertex(int i, float* c) : idx(i), coords(c) {};
};

struct Edge
{
	int idx; //edges[idx]
	int v1i, v2i; //endpnts
	float length;
	Edge(int id, int v1, int v2) : idx(id), v1i(v1), v2i(v2) { computeLength(); };

	void computeLength()
	{
		length = 7;
	}
};

struct Triangle
{
	int idx; //tris[idx]
	int v1i, v2i, v3i;
	Triangle(int id, int v1, int v2, int v3) : idx(id), v1i(v1), v2i(v2), v3i(v3) {};
};

class Mesh
{
private:
	void addTriangle(int v1, int v2, int v3);
	void addEdge(int v1, int v2);
	void addVertex(float x, float y, float z);
	bool makeVertsNeighbor(int v1i, int v2i);
public:
	vector< Vertex* > verts;
	vector< Triangle* > tris;
	vector< Edge* > edges;

	struct PathIntersection {
		int segmentIndex1;
		int segmentIndex2;
		float parameter1;  // Between 0 and 1
		float parameter2;  // Between 0 and 1
		float coords[3];   // 3D coordinates
	};

	Mesh() {};
	void createCube(float side);
	void loadOff(char* name);
	int* findShortestPath(int source, int& N);
	float* computeGeodesicDistances(int source, int& N);
	void computeGeodesicDistanceMatrix(const char* outputFile);
	vector<int> farthestPointSampling(int numSamples, int seedVertex = 0);
	vector<PathIntersection> findPathIntersections(vector<int>& path1, vector<int>& path2);
	void createPatches(vector<int>& sampledVertices);
	void normalizeCoordinates();
};