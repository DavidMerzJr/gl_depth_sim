#ifndef GL_DEPTH_SIM_MESH_H
#define GL_DEPTH_SIM_MESH_H

#include <Eigen/Dense>
#include <vector>

namespace gl_depth_sim
{

template <typename T>
using EigenAlignedVec = std::vector<T, Eigen::aligned_allocator<T>>;


class Mesh
{
public:
  Mesh(const EigenAlignedVec<Eigen::Vector3f>& vertices, const std::vector<unsigned>& indices);
  Mesh(const EigenAlignedVec<Eigen::Vector3f>& vertices, const EigenAlignedVec<Eigen::Vector3f>& normals, const std::vector<unsigned>& indices);

  std::size_t numIndices() const { return indices_.size(); }
  std::size_t numVertices() const { return vertices_.size(); }

  const std::vector<unsigned>& indices() const { return indices_; }
  const EigenAlignedVec<Eigen::Vector3f>& vertices() const { return vertices_; }
  const EigenAlignedVec<Eigen::Vector3f>& normals() const { return normals_; }

private:
  EigenAlignedVec<Eigen::Vector3f> vertices_;
  EigenAlignedVec<Eigen::Vector3f> normals_;
  std::vector<unsigned> indices_;
};

}

#endif // GL_DEPTH_SIM_MESH_H
