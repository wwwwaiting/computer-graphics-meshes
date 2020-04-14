#include "per_corner_normals.h"
#include "triangle_area_normal.h"
// Hint:
#include "vertex_triangle_adjacency.h"
#include <iostream>
#include "per_face_normals.h"

void per_corner_normals(
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  const double corner_threshold,
  Eigen::MatrixXd & N)
{
  N = Eigen::MatrixXd::Zero(F.rows()*3,3);
  ////////////////////////////////////////////////////////////////////////////  
  
  // get adjacent faces for each vertax
  std::vector<std::vector<int>> adjacent_faces;
  vertex_triangle_adjacency(F, V.rows(), adjacent_faces);
  
  // get the normal for each faces
  Eigen::MatrixXd face_normals;
  per_face_normals(V, F, face_normals);

  // compute the area-weighted average of normals
  for (int i = 0; i < F.rows(); i++) {
    for (int j = 0; j < F.cols(); j++) {
      Eigen::RowVector3d sum_normal(0, 0, 0);
      for (auto face : adjacent_faces[F(i,j)]) {
        double threshold = cos(corner_threshold*M_PI /180);
        // ignore if it's smaller than threshold
        if (face_normals.row(face).dot(face_normals.row(i)) >= threshold) {
          sum_normal += triangle_area_normal(V.row(F(face,0)), V.row(F(face,1)), V.row(F(face,2)));		
        }			
      }
      N.row(i*3+j) = sum_normal.normalized();
    }   
  }
  ////////////////////////////////////////////////////////////////////////////
}
