#include "catmull_clark.h"
#include <unordered_map>
#include <utility>
#include <functional>
#include <unordered_map>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <iostream>


// helper functions ---------------------------------------------------------
Eigen::RowVector3d get_edge_point(
	const Eigen::MatrixXd &V,
	std::unordered_map<int, Eigen::RowVector3d> &face_points,
	std::unordered_map<std::string, std::vector<int>> &edge_faces, 
	int a, 
	int b)
{
  std::string key = std::to_string(a) + " " + std::to_string(b);
  Eigen::RowVector3d sum(0, 0, 0);
	// only two faces
	sum += face_points[edge_faces[key][0]];
	sum += face_points[edge_faces[key][1]];
  sum += V.row(a);
  sum += V.row(b);
  return (sum/4.0);
}

Eigen::RowVector3d get_new_vertex(
  const Eigen::MatrixXd &V,
	std::unordered_map<int, Eigen::RowVector3d> &face_points,
	std::unordered_map<int, std::vector<int>> &vertex_adjacent_faces, 
	std::unordered_map<int, std::vector<int>> &vertex_nbs,
	int v)
{
	int n = vertex_adjacent_faces[v].size();
		
	//compute f which is the average of all n face points
	Eigen::RowVector3d f(0, 0, 0);
	int count = 0;
	for (auto face : vertex_adjacent_faces[v]) {
		f += face_points[face]; 	
		count += 1;	
	}	
	f /= count;
		
	//computerR which is the average of all original edge midpoints
	Eigen::RowVector3d r(0, 0, 0);
	Eigen::RowVector3d mid;
	count = 0;
	for (auto vertex : vertex_nbs[v]) {
		r += (V.row(v) + V.row(vertex)) / 2;
		count += 1;
	}		
	r /= count;
		
	// formula (F + 2R + (N-3)P)/n
	Eigen::RowVector3d new_v = (f + 2*r + (n-3)*V.row(v)) / n;
	return new_v;
};

void catmull_clark(
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  const int num_iters,
  Eigen::MatrixXd & SV,
  Eigen::MatrixXi & SF)
{
  ////////////////////////////////////////////////////////////////////////////
  // Replace with your code here:
  if (num_iters <= 0){
    return;
  }

  // calculate all face points
  std::unordered_map<int, Eigen::RowVector3d> face_points;
  std::unordered_map<int, std::vector<int>> vertex_adjacent_faces;
  std::unordered_map<std::string, std::vector<int>> edge_faces;
  std::unordered_map<int, std::vector<int>> vertex_nbs;
	for (int i = 0; i < F.rows(); i++){
		Eigen::RowVector3d pt_sum(0, 0, 0);
		for (int j = 0; j < F.cols(); j++){
			int idx1 = F(i, j);
			int idx2 = F(i, (j+1)%F.cols());
			int idx3 = F(i, ((j-1)+F.cols())%F.cols());
      // sum for face points
			pt_sum += V.row(idx1);

      // update adjacent faces for each vertax
      vertex_adjacent_faces[idx1].push_back(i);

      // find faces for each egde. key is v1-v2 and v2-v1; value is list of faces(two faces)
      std::string key_1 = std::to_string(idx1) + " " + std::to_string(idx2);
			std::string key_2 = std::to_string(idx2) + " " + std::to_string(idx1);
			edge_faces[key_1].push_back(i);
			edge_faces[key_2].push_back(i); 

      // find neibours vertices for each vertices
			if (std::find(vertex_nbs[idx1].begin(), vertex_nbs[idx1].end(), idx2) == vertex_nbs[idx1].end()){
        // add when not contain next vertax
        vertex_nbs[idx1].push_back(idx2);
			}
      if (std::find(vertex_nbs[idx1].begin(), vertex_nbs[idx1].end(), idx3) == vertex_nbs[idx1].end()){
        // add when not contain previous vertax
        vertex_nbs[idx1].push_back(idx3);
      }
		}
		face_points[i] = pt_sum/4;
	}

	SV.resize(0, 3);
  SF.resize(0, 4);
  for (int i = 0; i < F.rows(); i++){
		for (int j = 0; j < F.cols(); j++) {
			//add one row of face in SF.
			SF.conservativeResize(SF.rows()+1, 4);
			
			std::unordered_map<int, Eigen::RowVector3d> all;
			int idx1 = F(i, j);
			int idx2 = F(i, (j+1)%F.cols());
			int idx3 = F(i, ((j-1)+F.cols())%F.cols());
			//compute four vertices for new face
			Eigen::RowVector3d ep_forward = get_edge_point(V, face_points, edge_faces, idx1,idx2);
			Eigen::RowVector3d ep_backward = get_edge_point(V, face_points, edge_faces, idx1,idx3);
			Eigen::RowVector3d facepoint = face_points[i];	
			Eigen::RowVector3d new_v = get_new_vertex(V, face_points, vertex_adjacent_faces, vertex_nbs, idx1);
			all[0] = new_v;
			all[1] = ep_forward;
			all[2] = facepoint;
			all[3] = ep_backward;						
			
			//find the indices of four vertices
			for (int z = 0; z < 4; z++) {
				bool flag = false;
				for (int w = 0; w < SV.rows(); w++) {
					if (all[z].isApprox(SV.row(w))) {
						//found the vertex's index in SV
						SF(SF.rows()-1, z) = w;
						flag = true;
						break;			
					}
				}
				
				//add vertex if does not exist in SV
				if (!flag) {
					SV.conservativeResize(SV.rows()+1, 3);
					SV.row(SV.rows()-1) = all[z];
					SF(SF.rows()-1, z) = SV.rows()-1;
				} 
			}
		}	
	}
  catmull_clark(Eigen::MatrixXd(SV), Eigen::MatrixXi(SF), num_iters-1, SV, SF);
  ////////////////////////////////////////////////////////////////////////////
}
