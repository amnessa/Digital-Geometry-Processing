#pragma once

#include <stdint.h>

#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoIndexedLineSet.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoTransform.h>
#include "Mesh.h"


class Painter
{
public:
	SoSeparator* getShapeSep(Mesh* mesh);
	SoSeparator* get1PointSep(Mesh* obj, int pnt, float r, float g, float b, float pointSize, bool showNeighbours=false);
	SoSeparator* paintNeighbours(Mesh* mesh, int pnt);
	SoSeparator* DrawLines(Mesh* mesh,int source,int targ,int N, int* prev);
	vector<int> getPath(int* prev, int source, int targ, int N);
	SoSeparator* visualizeSampledPoints(Mesh* mesh, vector<int>& samples);
	SoSeparator* visualizePatches(Mesh* mesh, vector<vector<int>>& patches);

private:
	// Helper function to interpolate points along a curve
	SbVec3f interpolateAlongCurve(const vector<SbVec3f>& curve, float t);

	// Helper function to get geodesic path between two vertices
	vector<int> computeGeodesicPath(int source, int target, Mesh* mesh);
};