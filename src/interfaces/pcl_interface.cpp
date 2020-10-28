#include <gl_depth_sim/interfaces/pcl_interface.h>

#include <vector>

#include <Eigen/Dense>
#include <pcl/conversions.h>
#include <pcl/Vertices.h>

namespace gl_depth_sim
{

void toPointCloudXYZ(const gl_depth_sim::CameraProperties& camera, const gl_depth_sim::DepthImage& depth,
                                   pcl::PointCloud<pcl::PointXYZ>& out)
{
  out.width = depth.cols;
  out.height = depth.rows;
  out.resize(out.width * out.height);
  out.is_dense = false;

  for (int i = 0; i < depth.rows; ++i)
  {
    for (int j = 0; j < depth.cols; ++j)
    {
      const float distance = depth.distance(i, j);
      pcl::PointXYZ& pt = out(j, i);

      if (distance != 0.0f)
      {
        pt.z = distance;
        pt.x = (j - camera.cx) * distance / camera.fx;
        pt.y = (i - camera.cy) * distance / camera.fy;
      }
      else
      {
        pt.z = pt.x = pt.y = std::numeric_limits<float>::quiet_NaN();
      }
    }
  } // end of loop
}

pcl::PointCloud<pcl::PointXYZ> extractVerticesAsPointCloud(const Mesh& mesh)
{
  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  pointcloud.reserve(mesh.vertices().size());
  for (const Eigen::Vector3f& vertex : mesh.vertices())
  {
    pointcloud.push_back(pcl::PointXYZ(vertex(0), vertex(1), vertex(2)));
  }
  return pointcloud;
}

pcl::PolygonMesh toPolygonMesh(const Mesh& mesh)
{
  pcl::PolygonMesh pcl_mesh;

  // Retrieve the vertices and build the PolygonMesh point cloud
  pcl::toPCLPointCloud2(extractVerticesAsPointCloud(mesh), pcl_mesh.cloud);

  // Extract the vertex indices of every polygon in the PolygonMesh.  We assume the mesh is
  // triangular (that every polygon has three vertices).  There's not really a way to extract
  // any information to the contrary.
  pcl_mesh.polygons.reserve(mesh.indices().size() / 3);
  for (std::size_t i = 0; i + 2 < mesh.indices().size(); i += 3)
  {
    pcl::Vertices vertices;
    vertices.vertices = {mesh.indices()[i], mesh.indices()[i+1], mesh.indices()[i+2]};
    pcl_mesh.polygons.push_back(std::move(vertices));
  }

  // Return the completed pcl::PolygonMesh
  return pcl_mesh;
}

}
