#include "per_vertex_normals.h"
#include "triangle_area_normal.h"
#include <vector>
#include <unordered_map>
#include <Eigen/Geometry>

void per_vertex_normals(
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & N)
{
  N = Eigen::MatrixXd::Zero(V.rows(),3);
  ////////////////////////////////////////////////////////////////////////////
  // Add your code here:
  std::unordered_map<int, Eigen::RowVector3d> face_normal;
  std::unordered_map<int, std::vector<int>> vertex_adjacent_faces;

  for (int i = 0; i < F.rows(); i++) {
    // get normal for each face
		face_normal[i] = triangle_area_normal(V.row(F(i,0)), V.row(F(i,1)), V.row(F(i,2)));
		
    // get all adjacent faces for each vertax
		for (int j = 0; j < F.cols(); j++) {
			vertex_adjacent_faces[F(i,j)].push_back(i);		
		}	  
  }

  for (int i = 0; i < V.rows(); i++) {
		Eigen::RowVector3d sum_normal(0, 0, 0);
    // get all face normals that surrounding the vertex
		for (auto face: vertex_adjacent_faces[i]){
			sum_normal += face_normal[face];
		}
    // use area-weighted average  
		N.row(i) = sum_normal.normalized();
  }
  ////////////////////////////////////////////////////////////////////////////
}
