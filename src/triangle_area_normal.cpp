#include "triangle_area_normal.h"
#include <Eigen/Geometry>

Eigen::RowVector3d triangle_area_normal(
  const Eigen::RowVector3d & a, 
  const Eigen::RowVector3d & b, 
  const Eigen::RowVector3d & c)
{
  ////////////////////////////////////////////////////////////////////////////
  // Replace with your code:
  Eigen::RowVector3d normalized_n = (b-a).cross(c-a).normalized();
  // get area for the length of the vector
  double area = 0.5 * (b-a).cross(c-a).norm();
  ////////////////////////////////////////////////////////////////////////////
  return area * normalized_n;
}
