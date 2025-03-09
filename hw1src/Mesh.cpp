#include <cstdio>

#include "Mesh.h"


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


void Mesh::createCube(float sideLen)
{
	//coordinates
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

void Mesh::addTriangle(int v1, int v2, int v3)
{
	int idx = tris.size();
	tris.push_back( new Triangle(idx, v1, v2, v3) );

	//set up structure

	verts[v1]->triList.push_back(idx);
	verts[v2]->triList.push_back(idx);
	verts[v3]->triList.push_back(idx);

	if (! makeVertsNeighbor(v1, v2) )
		addEdge(v1, v2);

	if (! makeVertsNeighbor(v1, v3) )
		addEdge(v1, v3);

	if (! makeVertsNeighbor(v2, v3) )
		addEdge(v2, v3);

}

bool Mesh::makeVertsNeighbor(int v1i, int v2i)
{
	//returns true if v1i already neighbor w/ v2i; false o/w

	for (int i = 0; i < verts[v1i]->vertList.size(); i++)
		if (verts[v1i]->vertList[i] == v2i)
			return true;

	verts[v1i]->vertList.push_back(v2i);
	verts[v2i]->vertList.push_back(v1i);
	return false;
}

void Mesh::addVertex(float x, float y, float z)
{
	int idx = verts.size();
	float* c = new float[3];
	c[0] = x;
	c[1] = y;
	c[2] = z;

	verts.push_back( new Vertex(idx, c) );
}

void Mesh::addEdge(int v1, int v2)
{
	int idx = edges.size();

	edges.push_back( new Edge(idx, v1, v2) );

	verts[v1]->edgeList.push_back(idx);
	verts[v2]->edgeList.push_back(idx);
}

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
    
    // Min-heap implementation for Dijkstra
    // Use priority_queue or implement your own min-heap
    
    // Process all vertices
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

int* Mesh::findShortestPath(int source, int& N) {
    N = verts.size();  // Set the output parameter N to the number of vertices
    float* dist = new float[N];
    int* prev = new int[N];
    bool* visited = new bool[N];
    
    // Initialize
    for (int i = 0; i < N; i++) {
        dist[i] = FLT_MAX;
        prev[i] = -1;
        visited[i] = false;
    }
    
    dist[source] = 0;
    
    // Dijkstra's algorithm
    for (int count = 0; count < N; count++) { // Changed N-1 to N for safety
        int u = -1;
        float minDist = FLT_MAX;
        for (int i = 0; i < N; i++) {
            if (!visited[i] && dist[i] < minDist) {
                minDist = dist[i];
                u = i;
            }
        }
        
        if (u == -1) break; // No more vertices to process
        
        visited[u] = true;
        
        // Process neighbors - ADD BOUNDS CHECKING
        for (size_t i = 0; i < verts[u]->vertList.size(); i++) {
            int v = verts[u]->vertList[i];
            
            // Verify v is a valid index
            if (v < 0 || v >= N) continue;
            
            // Calculate edge length
            float dx = verts[u]->coords[0] - verts[v]->coords[0];
            float dy = verts[u]->coords[1] - verts[v]->coords[1];
            float dz = verts[u]->coords[2] - verts[v]->coords[2];
            float edgeLength = sqrt(dx*dx + dy*dy + dz*dz);
            
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

void Mesh::computeGeodesicDistanceMatrix(const char* outputFile) {
    int N = verts.size();
    float** distMatrix = new float*[N];
    
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

vector<int> Mesh::farthestPointSampling(int numSamples, int seedVertex) {
    int N = verts.size();
    vector<int> samples;
    
    if (numSamples <= 0 || numSamples > N)
        return samples;
    
    // Initialize with the seed vertex
    samples.push_back(seedVertex);
    
    // Initialize distance array
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
    
    // Select remaining samples
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
        
        // Add to samples
        samples.push_back(farthestVertex);
        
        // Update minimum distances
        distances = computeGeodesicDistances(farthestVertex, dummy);
        for (int v = 0; v < N; v++) {
            minDistToSamples[v] = min(minDistToSamples[v], distances[v]);
        }
        delete[] distances;
    }
    
    delete[] minDistToSamples;
    return samples;
}

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
            
            // Check if these segments intersect
            // (This is a simplified check and might need more robust implementation)
            // ... implement line segment intersection in 3D
            
            // If they intersect, add to list
            // PathIntersection intersection;
            // intersection.segmentIndex1 = i;
            // intersection.segmentIndex2 = j;
            // ... compute parameters and coordinates
            // intersections.push_back(intersection);
        }
    }
    
    return intersections;
}

// Helper function to convert from prev array to path vector
vector<int> getPathVector(int* prev, int source, int target, int N) {
    vector<int> path;
    
    // Safety check 
    if (!prev || target < 0 || target >= N) {
        return path;
    }
    
    // Build path - prevent infinite loops
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
    
    reverse(path.begin(), path.end());
    return path;
}

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
                    int* prev1_2 = findShortestPath(s1, s2);
                    int* prev1_3 = findShortestPath(s1, s3);
                    int* prev1_4 = findShortestPath(s1, s4);
                    int* prev2_3 = findShortestPath(s2, s3);
                    int* prev2_4 = findShortestPath(s2, s4);
                    int* prev3_4 = findShortestPath(s3, s4);
                    
                    // Convert to path vector
                    vector<int> path1_2 = getPathVector(prev1_2, s1, s2, verts.size());
                    vector<int> path1_3 = getPathVector(prev1_3, s1, s3, verts.size());
                    vector<int> path1_4 = getPathVector(prev1_4, s1, s4, verts.size());
                    vector<int> path2_3 = getPathVector(prev2_3, s2, s3, verts.size());
                    vector<int> path2_4 = getPathVector(prev2_4, s2, s4, verts.size());
                    vector<int> path3_4 = getPathVector(prev3_4, s3, s4, verts.size());
                    
                    // Find intersections between paths
                    // ...
                    
                    // If we have 4 intersection points forming a quadrilateral, create a patch
                    // ...
                    
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
    
    // Apply transformation
    for (unsigned int i = 0; i < verts.size(); i++) {
        verts[i]->coords[0] = (verts[i]->coords[0] - centerX) * scale;
        verts[i]->coords[1] = (verts[i]->coords[1] - centerY) * scale;
        verts[i]->coords[2] = (verts[i]->coords[2] - centerZ) * scale;
    }
    
    printf("Mesh normalized: Scale=%f, Center=(%f,%f,%f)\n", 
           scale, centerX, centerY, centerZ);
}