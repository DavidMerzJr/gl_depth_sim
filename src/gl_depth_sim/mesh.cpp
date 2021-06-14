#include "gl_depth_sim/mesh.h"

gl_depth_sim::Mesh::Mesh(const EigenAlignedVec<Eigen::Vector3f>& vertices, const std::vector<unsigned>& indices)
  : vertices_(vertices), indices_(indices)
{
  // TODO: Push points back one at a time as they appear in a triangle, then create the normals for
  // them.  This will ensure all vertices have the correct normal for the face.  RenderableMesh
  // already requires points to be duplicated anyway, so no efficiency is lost here.
  normals_ = EigenAlignedVec<Eigen::Vector3f>(vertices_.size(), Eigen::Vector3f::UnitZ());
  for (std::size_t i = 0; i + 2 < indices.size(); i += 3)
  {
    if (indices[i] < vertices.size() &&
        indices[i+1] < vertices.size() &&
        indices[i+2] < vertices.size())
    {
      Eigen::Vector3f s12 = vertices[indices[i+1]] - vertices[indices[i]];
      Eigen::Vector3f s13 = vertices[indices[i+2]] - vertices[indices[i]];
      Eigen::Vector3f normal = s12.cross(s13);
      normals_[indices[i]] = normal;
      normals_[indices[i+1]] = normal;
      normals_[indices[i+2]] = normal;
    }
  }
}

gl_depth_sim::Mesh::Mesh(const EigenAlignedVec<Eigen::Vector3f>& vertices, const EigenAlignedVec<Eigen::Vector3f>& normals, const std::vector<unsigned>& indices)
  : vertices_(vertices), normals_(normals), indices_(indices)
{}
