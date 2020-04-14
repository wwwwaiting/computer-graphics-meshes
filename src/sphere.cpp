#include "sphere.h"
#include <iostream>

void sphere(
  const int num_faces_u,
  const int num_faces_v,
  Eigen::MatrixXd & V,
  Eigen::MatrixXi & F,
  Eigen::MatrixXd & UV,
  Eigen::MatrixXi & UF,
  Eigen::MatrixXd & NV,
  Eigen::MatrixXi & NF)
{
  ////////////////////////////////////////////////////////////////////////////
  // Add your code here:
  int num_faces = num_faces_u * num_faces_v;  
  int num_vertices = (num_faces_u + 1) * (num_faces_v + 1);

  // Resize the matrices
  V.resize(num_vertices, 3);
  F.resize(num_faces, 4);
  UV.resize(num_vertices, 2);
  UF.resize(num_faces, 4);
  NV.resize(num_vertices, 3);
  NF.resize(num_faces, 4);

  // see reference: https://stackoverflow.com/questions/7840429/calculate-the-xyz-point-of-a-sphere-given-a-uv-coordinate-of-its-texture

  const double u_unit = 1.0 / num_faces_u;
  const double v_unit = 1.0 / num_faces_v;

  // deal with V, UV and NV
  int idx = 0;
  for (int i=0; i<num_faces_u+1; i++){
    for (int j=0; j<num_faces_v+1; j++){
      double theta = 2 * M_PI * (u_unit*i);
      double phi = M_PI * (v_unit*j);

      double x = -cos(theta) * sin(phi);
      double y = -sin(theta) * sin(phi);
      double z = -cos(phi);
      V.row(idx) = Eigen::Vector3d(x, y, z);
      UV.row(idx) = Eigen::Vector2d(u_unit*i, v_unit*j);
      NV.row(idx) = Eigen::Vector3d(x, y, z);
      idx ++;
    }
  }

  // deal with F, UF and NF
  idx = 0;
  for (int i=0; i<num_faces_u; i++){
    for (int j=0; j<num_faces_v; j++){
      int bl = i*(num_faces_v+1) + j;
      int br = bl + 1;
      int tl = (i+1)*(num_faces_v+1) + j;
      int tr = tl + 1;

      F.row(idx) = Eigen::RowVector4i(bl, br, tr, tl);
      UF.row(idx) = Eigen::RowVector4i(bl, br, tr, tl);
      NF.row(idx) = Eigen::RowVector4i(bl, br, tr, tl);
      idx ++;
    }
  }


  ////////////////////////////////////////////////////////////////////////////
}
