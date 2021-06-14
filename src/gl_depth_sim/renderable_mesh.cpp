#include "gl_depth_sim/renderable_mesh.h"
#include "gl_depth_sim/glad/glad.h"
#include <iostream>

gl_depth_sim::RenderableMesh::RenderableMesh()
  : num_indices_{0} {}

gl_depth_sim::RenderableMesh::RenderableMesh(const gl_depth_sim::Mesh& mesh)
  : num_indices_{mesh.numIndices()}
{
  meshes_.push_back(mesh);
}

gl_depth_sim::RenderableMesh::~RenderableMesh()
{
  glDeleteVertexArrays(1, &vao_);
  glDeleteBuffers(1, &vbo_);
  glDeleteBuffers(1, &tbo1_);
  glDeleteBuffers(1, &tbo2_);
  glDeleteBuffers(1, &ebo_);
}

void gl_depth_sim::RenderableMesh::setupGL()
{
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glGenBuffers(1, &tbo1_);
  glGenBuffers(1, &ebo_);
  glGenBuffers(1, &tbo2_);

  glBindVertexArray(vao_);
  EigenAlignedVec<Eigen::Vector3f> combined;
  std::size_t total_length = 0;
  std::vector<unsigned> all_indices;
  unsigned prev_index_len = 0;

  // TODO: Check if normals and vertices are the same size
  for (auto &mesh : meshes_)
  {
    total_length += mesh.vertices().size();
    for (int i = 0; i < mesh.vertices().size(); i++)
    {
      combined.push_back(mesh.vertices()[i]);
      combined.push_back(mesh.normals()[i]);
    }
    for (auto index : mesh.indices())
    {
      all_indices.push_back(index + prev_index_len);
    }
    prev_index_len = all_indices.size();
  }

  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f) * total_length * 2, combined.data(),
               GL_STATIC_DRAW);

  std::vector<Eigen::Vector3f> init_data(total_length);
  for (int i = 0; i < total_length; i++)
    init_data[i] = Eigen::Vector3f(0.0, 0.0, 0.0);

  // Set the tranfer feedback buffer to be dynamic since it changes frequently
  glBindBuffer(GL_ARRAY_BUFFER, tbo1_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f) * total_length, init_data.data(), GL_DYNAMIC_DRAW);
  glBindBufferBase(GL_ARRAY_BUFFER_BINDING, 0, tbo1_);

  glBindBuffer(GL_TRANSFORM_FEEDBACK_BUFFER, tbo2_);
  glBufferData(GL_TRANSFORM_FEEDBACK_BUFFER, sizeof(Eigen::Vector3f) * total_length, init_data.data(), GL_DYNAMIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) *  total_length, all_indices.data(),
               GL_STATIC_DRAW);

  // These are in the vertex buffer
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  // vertex positions
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 2*sizeof(Eigen::Vector3f), (void*)0);
  glEnableVertexAttribArray(0);
  // vertex normals
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_TRUE, 2*sizeof(Eigen::Vector3f), (void*)(sizeof(Eigen::Vector3f)));
  glEnableVertexAttribArray(1);

  // These are in the tranform feedback buffer
  glBindBuffer(GL_ARRAY_BUFFER, tbo1_);
  glVertexAttribPointer(2, 3, GL_FLOAT, GL_TRUE, sizeof(Eigen::Vector3f), (void*)0);
  glEnableVertexAttribArray(2);

//  glBindBuffer(GL_TRANSFORM_FEEDBACK_BUFFER, tbo2_);
  glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, tbo2_);
  glVertexAttribPointer(3, 3, GL_FLOAT, GL_TRUE, sizeof(Eigen::Vector3f), (void*)0);
  glEnableVertexAttribArray(3);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

}

void gl_depth_sim::RenderableMesh::addMesh(const Mesh &mesh)
{
  meshes_.push_back(mesh);
  num_indices_ += mesh.numIndices();
}
