#include <stdint.h>

#include "Painter.h"

SoSeparator* Painter::getShapeSep(Mesh* mesh)
{
	SoSeparator* res = new SoSeparator();
	
	// Create the transparent mesh faces
	SoSeparator* facesSep = new SoSeparator();
	SoMaterial* facesMat = new SoMaterial();
	facesMat->diffuseColor.setValue(1, 1, 1);
	facesMat->transparency = 0.7f; // Make faces transparent
	facesSep->addChild(facesMat);

	SoShapeHints* hints = new SoShapeHints;
	hints->creaseAngle = 3.14;
	facesSep->addChild(hints);
	
	SoCoordinate3* coords = new SoCoordinate3();
	for (int c = 0; c < mesh->verts.size(); c++)
		coords->point.set1Value(c, mesh->verts[c]->coords[0], mesh->verts[c]->coords[1], mesh->verts[c]->coords[2]);
	
	SoIndexedFaceSet* faceSet = new SoIndexedFaceSet();
	for (int c = 0; c < mesh->tris.size(); c++)
	{
		faceSet->coordIndex.set1Value(c*4, mesh->tris[c]->v1i);
		faceSet->coordIndex.set1Value(c*4 + 1, mesh->tris[c]->v2i);
		faceSet->coordIndex.set1Value(c*4 + 2, mesh->tris[c]->v3i);
		faceSet->coordIndex.set1Value(c*4 + 3, -1);
	}
	
	facesSep->addChild(coords);
	facesSep->addChild(faceSet);
	res->addChild(facesSep);

	// Keep the edges with full opacity if drawThickEdges is true
	bool drawThickEdges = true;
	if (drawThickEdges)
	{
		SoSeparator* thickEdgeSep = new SoSeparator;
		SoMaterial* ma = new SoMaterial;
		ma->diffuseColor.set1Value(0, 0.0f, 0.0f, 1.0f);
		thickEdgeSep->addChild(ma);
		SoDrawStyle* sty = new SoDrawStyle;	sty->lineWidth = 1.0f;	thickEdgeSep->addChild(sty);

		SoIndexedLineSet* ils = new SoIndexedLineSet;
		SoCoordinate3* co = new SoCoordinate3;

		//assumes no edge in sedges is removed
		for (unsigned int se = 0; se < mesh->edges.size(); se++)
		{
			SbVec3f end1 = mesh->verts[mesh->edges[se]->v1i]->coords + SbVec3f(0.0f, 0.0f, 0.0f),
				end2 = mesh->verts[mesh->edges[se]->v2i]->coords + SbVec3f(0.0f, 0.0f, 0.0f);
			co->point.set1Value(2 * se, end1);
			co->point.set1Value(2 * se + 1, end2);
		}

		for (unsigned int ci = 0; ci < mesh->edges.size(); ci++)
		{
			ils->coordIndex.set1Value(3 * ci, 2 * ci);	ils->coordIndex.set1Value(3 * ci + 1, 2 * ci + 1);
			ils->coordIndex.set1Value(3 * ci + 2, -1); //end this edge with -1
		}
		thickEdgeSep->addChild(co);	thickEdgeSep->addChild(ils);
		res->addChild(thickEdgeSep);
	}

	return res;
}
SoSeparator* Painter::DrawLines(Mesh* mesh, int source, int target, int N, int* prev) {
	SoSeparator* lineSep = new SoSeparator();
	
	// Safety check - ensure source and target are valid and different
	if (source < 0 || source >= N || target < 0 || target >= N || source == target) {
		return lineSep;  // Return empty separator
	}
	
	// Get the path
	vector<int> path;
	int at = target;
	while (at != source && at != -1) {
		path.push_back(at);
		at = prev[at];
	}
	
	// Add source if path completed successfully
	if (at == source) {
		path.push_back(source);
	} else {
		// Path not found
		return lineSep;
	}
	
	// Reverse to get from source to target
	reverse(path.begin(), path.end());
	
	// Safety check - need at least two points for a line
	if (path.size() < 2) {
		return lineSep;
	}
	
	// Style and material setup
	SoMaterial* mat = new SoMaterial();
	mat->diffuseColor.setValue(1, 0, 0); // Red path
	lineSep->addChild(mat);
	
	SoDrawStyle* style = new SoDrawStyle();
	style->lineWidth = 3.0f;
	lineSep->addChild(style);
	
	// Create coordinates for path vertices
	SoCoordinate3* coords = new SoCoordinate3();
	for (unsigned int i = 0; i < path.size(); i++) {
		int vertIdx = path[i];
		if (vertIdx >= 0 && vertIdx < mesh->verts.size()) {
			coords->point.set1Value(i, 
				mesh->verts[vertIdx]->coords[0],
				mesh->verts[vertIdx]->coords[1],
				mesh->verts[vertIdx]->coords[2]);
		}
	}
	lineSep->addChild(coords);
	
	// Create indexed line set
	SoIndexedLineSet* lineSet = new SoIndexedLineSet();
	int coordIndex = 0;
	for (unsigned int i = 0; i < path.size() - 1; i++) {
		lineSet->coordIndex.set1Value(coordIndex++, i);
		lineSet->coordIndex.set1Value(coordIndex++, i+1);
	}
	lineSet->coordIndex.set1Value(coordIndex, -1);
	
	lineSep->addChild(lineSet);
	return lineSep;
}
	
SoSeparator* Painter::get1PointSep(Mesh* obj, int pnt, float r, float g, float b, float pointSize, bool showNeighbours)
{
	Mesh* mesh = obj;

	SoSeparator* pntSep = new SoSeparator;
	
	// Material with configurable color
	SoMaterial* mat = new SoMaterial;
	mat->diffuseColor.setValue(SbColor(r, g, b));
	mat->emissiveColor.setValue(r*0.3f, g*0.3f, b*0.3f);  // Add some glow
	pntSep->addChild(mat);
	
	// Use a transform to position the sphere
	SoTransform* xform = new SoTransform;
	xform->translation.setValue(
		mesh->verts[pnt]->coords[0],
		mesh->verts[pnt]->coords[1],
		mesh->verts[pnt]->coords[2]
	);
	pntSep->addChild(xform);
	
	// Create a sphere with size based on pointSize parameter
	SoSphere* sphere = new SoSphere;
	sphere->radius = pointSize * 0.01f; // Scale factor to keep spheres reasonably sized
	pntSep->addChild(sphere);
	
	if(showNeighbours)
		pntSep->addChild(paintNeighbours(mesh, pnt));

	return pntSep;
}

vector<int> Painter::getPath(int* prev,int source,int targ,int N){
	int u=targ;
	vector<int> path;
	if (prev[u] != NULL || u == source) {
		//printf("in if");
		while (u != -1) {
			path.push_back(u);
			u = prev[u];
		}
	}
	return path;
}

/// <summary>
/// Use this function for debugging, to check if we succesfully found the neighbours of the vertex.
/// </summary>
/// <param name="mesh"></param>
/// <param name="pnt"></param>
/// <returns></returns>
SoSeparator* Painter::paintNeighbours(Mesh* mesh, int pnt) {
	SoSeparator* pntSep = new SoSeparator;
	//material
	SoMaterial* mat = new SoMaterial;
	mat->diffuseColor.setValue(SbColor(1.0f, 1.0f, 0.0f));
	pntSep->addChild(mat);
	SoDrawStyle* style = new SoDrawStyle;
	style->pointSize = 10.0f;
	pntSep->addChild(style);

	//shape
	SoVertexProperty* vp = new SoVertexProperty;
	for (int i = 0; i < mesh->verts[pnt]->vertList.size(); i++) {
		//printf("Neighbour %d is %d", i, mesh->verts[pnt]->vertList[i]);
		vp->vertex.
			set1Value(i, mesh->verts[mesh->verts[pnt]->vertList[i]]->coords);
	}
	SoPointSet* pSet = new SoPointSet;
	pSet->numPoints = mesh->verts[pnt]->vertList.size();
	pSet->vertexProperty = vp;
	pntSep->addChild(pSet);
	return pntSep;
}

SoSeparator* Painter::visualizeSampledPoints(Mesh* mesh, vector<int>& samples) {
    SoSeparator* pointsSep = new SoSeparator();
    
    // Material for sample points
    SoMaterial* mat = new SoMaterial();
    mat->diffuseColor.setValue(0, 1, 0); // Green
    pointsSep->addChild(mat);
    
    // Style for sample points (larger)
    SoDrawStyle* style = new SoDrawStyle();
    style->pointSize = 10.0f;
    pointsSep->addChild(style);
    
    // Create coordinates for sample points
    SoCoordinate3* coords = new SoCoordinate3();
    for (int i = 0; i < samples.size(); i++) {
        int vertIdx = samples[i];
        coords->point.set1Value(i, mesh->verts[vertIdx]->coords);
    }
    pointsSep->addChild(coords);
    
    // Create point set
    SoPointSet* pointSet = new SoPointSet();
    pointSet->numPoints = samples.size();
    pointsSep->addChild(pointSet);
    
    return pointsSep;
}

SoSeparator* Painter::visualizePatches(Mesh* mesh, vector<vector<int>>& patchBoundaries) {
    SoSeparator* patchesSep = new SoSeparator();
    
    SoMaterial* patchMat = new SoMaterial();
    patchMat->diffuseColor.setValue(0.3f, 0.8f, 0.6f);
    patchMat->transparency = 0.2f;
    patchesSep->addChild(patchMat);
    
    // Process each patch boundary
    for (const vector<int>& boundary : patchBoundaries) {
        if (boundary.size() < 4) continue; // Need at least a quad
        
        // Generate a parametric grid within the boundary
        int gridSize = 10; // Resolution of the patch
        vector<SbVec3f> gridPoints;
        vector<int> gridFaces;
        
        // For a quadrilateral boundary, generate a bilinear patch
        if (boundary.size() == 4) {
            // Get the four corner points
            SbVec3f p00(mesh->verts[boundary[0]]->coords);
            SbVec3f p10(mesh->verts[boundary[1]]->coords);
            SbVec3f p11(mesh->verts[boundary[2]]->coords);
            SbVec3f p01(mesh->verts[boundary[3]]->coords);
            
            // Generate grid using bilinear interpolation
            for (int i = 0; i <= gridSize; i++) {
                float u = i / float(gridSize);
                for (int j = 0; j <= gridSize; j++) {
                    float v = j / float(gridSize);
                    
                    // Bilinear interpolation formula
                    SbVec3f point = p00 * (1-u) * (1-v) + 
                                   p10 * u * (1-v) + 
                                   p11 * u * v + 
                                   p01 * (1-u) * v;
                    
                    gridPoints.push_back(point);
                    
                    // Create faces (except for the last row/column)
                    if (i < gridSize && j < gridSize) {
                        int idx = i * (gridSize + 1) + j;
                        // Add two triangles per grid cell
                        gridFaces.push_back(idx);
                        gridFaces.push_back(idx + 1);
                        gridFaces.push_back(idx + gridSize + 1);
                        gridFaces.push_back(-1);
                        
                        gridFaces.push_back(idx + 1);
                        gridFaces.push_back(idx + gridSize + 2);
                        gridFaces.push_back(idx + gridSize + 1);
                        gridFaces.push_back(-1);
                    }
                }
            }
            
            // Set up coordinates
            SoCoordinate3* coords = new SoCoordinate3();
            for (size_t i = 0; i < gridPoints.size(); i++) {
                coords->point.set1Value(i, gridPoints[i]);
            }
            
            // Set up face set
            SoIndexedFaceSet* faceSet = new SoIndexedFaceSet();
            for (size_t i = 0; i < gridFaces.size(); i++) {
                faceSet->coordIndex.set1Value(i, gridFaces[i]);
            }
            
            // Add to separator
            SoSeparator* patchSep = new SoSeparator();
            patchSep->addChild(coords);
            patchSep->addChild(faceSet);
            patchesSep->addChild(patchSep);
        }
    }
    
    return patchesSep;
}