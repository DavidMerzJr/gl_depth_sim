#ifndef GL_DEPTH_SIM_PCL_INTERFACE_H
#define GL_DEPTH_SIM_PCL_INTERFACE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

#include <gl_depth_sim/camera_properties.h>
#include <gl_depth_sim/mesh.h>

namespace gl_depth_sim
{

/**
 * @brief Projects the given depth image forward into space using the camera intrinsics from @e camera. The
 * resulting point cloud is organized (has width and height) and NOT dense. Invalid points are marked as NAN.
 */
void toPointCloudXYZ(const CameraProperties& camera, const DepthImage& depth, pcl::PointCloud<pcl::PointXYZ>& out);

/**
 * @brief toPolygonMesh - constructs a pcl::PolygonMesh mesh using the supplied gl_depth_sim::Mesh
 * as a template.  Vertices and faces should be 1:1 between the source and return value.  Note that
 * converting to pcl::PolygonMesh causes a loss of normal information.
 * @param mesh - input - a triangular mesh.  Non-triangular meshes will not process correctly.
 * @return - an analogous pcl::PolygonMesh
 */
pcl::PolygonMesh toPolygonMesh(const Mesh& mesh);

/**
 * @brief extractVerticesAsPointCloud - extract the vertices of a gl_depth_sim::Mesh as a PCL
 * PointCloud.
 * @param mesh - input - a triangular mesh in 3D Space
 * @return - a pcl::PointCloud containing pcl::PointXYZ representing each vertex of the mesh
 */
pcl::PointCloud<pcl::PointXYZ> extractVerticesAsPointCloud(const Mesh& mesh);

}

#endif
