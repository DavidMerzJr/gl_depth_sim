#ifndef GL_DEPTH_SIM_RENDERABLE_MESH_H
#define GL_DEPTH_SIM_RENDERABLE_MESH_H

#include "gl_depth_sim/mesh.h"

namespace gl_depth_sim
{

class RenderableMesh
{
public:
  RenderableMesh();
  RenderableMesh(const Mesh& mesh);
  ~RenderableMesh();

  unsigned vao() const { return vao_; }
  unsigned vbo() const { return vbo_; }
  unsigned ebo() const { return ebo_; }
  unsigned tbo1() const { return tbo1_; }
  unsigned tbo2() const { return tbo2_; }

  std::size_t numIndices() const { return num_indices_; }

  void addMesh(const Mesh& mesh);
  void setupGL();

private:

  /** @brief Vertex array object: */
  unsigned int vao_;
  /** @brief Vertex buffer object: Stores vertices in GPU memory */
  unsigned int vbo_;
  /** @brief Transfer buffer object: Stores vertices to be transfered */
  unsigned int tbo1_;
  /** @brief Transfer buffer object: Stores vertices to be transfered back */
  unsigned int tbo2_;
  /** @brief Element buffer object: Stores indices in vbo that get drawn */
  unsigned int ebo_;
  std::size_t num_indices_;

  std::vector<Mesh> meshes_;
};

}

#endif
