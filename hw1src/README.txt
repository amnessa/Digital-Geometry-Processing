In this project, I implemented the following functions:

1. Dijkstra's algorithm to find the shortest path between two vertices.
2. A function to find the intersection point between two geodesic paths.
3. A function to compute the geodesic distance between two vertices.

In the patching part, I used the 249.off and 348.off meshes.

in the 249.off mesh, the object was cylindrical and meshes were streched along the z-axis. So since we are interpolating the fps points the patch were inside object.

in the 348.off mesh, the object was an eagle so it look like planar. So the patch was more sensible compared to the original mesh.

Another point related to both parts was that when After picking the farthest points and creating a loop between them should guarantee no intersection around the edges otherwise we would get a folded patch.

To overcome problem mentioned in the 249.off mesh, I should test using more than 4 FPS points and try to smooth the interpolation as a result of this. 

Lastly the Dijkstra's furthest point on geodesic paths does not always give the best looking path. For example in the 348.off mesh algorithm picked one side of the tail as a furthest point and path to reaching this point stayed close to each other.
As a result the patch could not cover the whole object. 

For further improvement I would try to find non exhaustive search method with higher number of FPS points. Search of ideal number of FPS points can be done by trying different number of FPS points and checking the patch.


