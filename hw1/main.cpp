#include <igl/unproject_onto_mesh.h>
#include <igl/parula.h>
#include <igl/PI.h>
#include <igl/exact_geodesic.h>

#include <igl/isolines.h>
#include <igl/readOBJ.h>
#include <igl/opengl/glfw/Viewer.h>
#include <iostream>




int main(int argc, char* argv[])
{
  using namespace Eigen;
  using namespace std;
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  igl::opengl::glfw::Viewer viewer;

  // Load a mesh in OBJ format
  igl::readOBJ("geomesh/for timing/dragon.obj", V, F);

  const auto update_distance = [&](const int vid)
  {
    Eigen::VectorXi VS, FS, VT, FT;
    // The selected vertex is the source

    VS.resize(1);
    VS << vid;

    // All vertices are the targets
    VT.setLinSpaced(V.rows(), 0, V.rows() - 1);
    Eigen::VectorXd d;

    std::cout << "Computing geodesic distance from vertex " << vid <<"....\n" ;
    igl::exact_geodesic(V, F, VS, FS, VT, FT, d);

    // Plot the mesh

    Eigen::MatrixXd CM;
    igl::parula(Eigen::VectorXd::LinSpaced(21,0,1).eval(),false,CM);
    //igl::isolines_map(Eigen::MatrixXd(CM),CM);
    viewer.data().set_colormap(CM);
    viewer.data().set_data(d);
  };

  // Plot a distance when a vertex is picked
  viewer.callback_mouse_down =
  [&](igl::opengl::glfw::Viewer& viewer, int, int)->bool
  {
    int fid;
    Eigen::Vector3f bc;
    // Cast a ray in the view direction starting from the mouse position
    double x = viewer.current_mouse_x;
    double y = viewer.core().viewport(3) - viewer.current_mouse_y;
    if(igl::unproject_onto_mesh(
      Eigen::Vector2f(x,y),
      viewer.core().view,
      viewer.core().proj,
      viewer.core().viewport,
      V,
      F,
      fid,
      bc))
    {
      int max;
      bc.maxCoeff(&max);
      int vid = F(fid,max);
      update_distance(vid);
      return true;
    }
    return false;
  };
  viewer.data().set_mesh(V,F);
  viewer.data().show_lines = false;

  cout << "Click on mesh to define new source.\n" << std::endl;
  update_distance(0);
  return viewer.launch();

}


// int main(int argc, char *argv[])
// {
//   // Inline mesh of a cube
//   const Eigen::MatrixXd V= (Eigen::MatrixXd(8,3)<<
//     0.0,0.0,0.0,
//     0.0,0.0,1.0,
//     0.0,1.0,0.0,
//     0.0,1.0,1.0,
//     1.0,0.0,0.0,
//     1.0,0.0,1.0,
//     1.0,1.0,0.0,
//     1.0,1.0,1.0).finished();
//   const Eigen::MatrixXi F = (Eigen::MatrixXi(12,3)<<
//     0,6,4,
//     0,2,6,
//     0,3,2,
//     0,1,3,
//     2,7,6,
//     2,3,7,
//     4,6,7,
//     4,7,5,
//     0,4,5,
//     0,5,1,
//     1,5,7,
//     1,7,3).finished();

//   // Plot the mesh
//   igl::opengl::glfw::Viewer viewer;
//   viewer.data().set_mesh(V, F);
//   viewer.data().set_face_based(true);
//   viewer.launch();

// }