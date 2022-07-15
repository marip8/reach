/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "colorization_utils.cpp"

#include <rbf_interpolation/impl/rbf_solver.hpp>
#include <rbf_interpolation/impl/rbf/gaussian_rbf.hpp>
#include <rbf_interpolation/impl/rbf/polyharmonic_rbf.hpp>
#include <rbf_interpolation/impl/rbf/thin_plate_spline_rbf.hpp>
#include <rbf_interpolation/impl/rbf/multiquadric_rbf.hpp>
#include <rbf_interpolation/impl/rbf/inverse_multiquadric_rbf.hpp>

#include <reach_core/utils/serialization_utils.h>
#include <reach_msgs/ReachDatabase.h>

#include <pcl/io/ply_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <colorized_mesh_display/ColorizedMeshStamped.h>

colorized_mesh_display::ColorizedMeshStamped toColorizedMesh(const pcl::PolygonMesh& in,
                                                             const std::string& base_frame,
                                                             const std::vector<reach_visualizer::utils::rgb>& colors)
{
  colorized_mesh_display::ColorizedMeshStamped out;
  out.header.frame_id = base_frame;
  out.header.stamp = ros::Time::now();

  pcl::PointCloud<pcl::PointNormal> vertex_cloud;
  pcl::fromPCLPointCloud2(in.cloud, vertex_cloud);

  if(colors.size() != vertex_cloud.points.size())
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": inputs not the same size!");
    return {};
  }

  // Add the vertex information
  for(std::size_t i = 0; i < vertex_cloud.points.size(); ++i)
  {
    const pcl::PointNormal& pt = vertex_cloud.points[i];
    // Add the vertex
    geometry_msgs::Point32 vertex;
    vertex.x = pt.x;
    vertex.y = pt.y;
    vertex.z = pt.z;
    out.mesh.vertices.push_back(vertex);

    // Add the normal
    geometry_msgs::Vector3 normal;
    normal.x = 0; //pt.normal_x;
    normal.y = 0; //pt.normal_y;
    normal.z = 1; //pt.normal_z;
    out.mesh.vertex_normals.push_back(normal);

    // Add the color
    std_msgs::ColorRGBA color;
    color.r = colors[i].r;
    color.g = colors[i].g;
    color.b = colors[i].b;
    color.a = 1.0f;
    out.mesh.vertex_colors.push_back(color);
  }

  // Add the triangle information
  for(std::size_t i = 0; i < in.polygons.size(); ++i)
  {
    const pcl::Vertices& poly = in.polygons[i];
    for(std::size_t j = 0; j < poly.vertices.size() - 2; ++j)
    {
      shape_msgs::MeshTriangle triangle;
      triangle.vertex_indices = {poly.vertices[0], poly.vertices[j + 1], poly.vertices[j + 2]};
      out.mesh.triangles.push_back(std::move(triangle));
    }
  }

  return out;
}

sensor_msgs::PointCloud2 toColorizedPointCloud(const std::vector<reach_msgs::ReachRecord>& records,
                                               const std::vector<reach_visualizer::utils::rgb>& colors)
{
  if(records.size() != colors.size())
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": inputs not the same size!");
    return {};
  }

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.points.reserve(records.size());
  for(std::size_t i = 0; i < records.size(); ++i)
  {
    pcl::PointXYZRGB pt;
    pt.x = (float)records[i].goal.position.x;
    pt.y = (float)records[i].goal.position.y;
    pt.z = (float)records[i].goal.position.z;
    pt.r = (uint8_t)(colors[i].r * 255.0f);
    pt.g = (uint8_t)(colors[i].g * 255.0f);
    pt.b = (uint8_t)(colors[i].b * 255.0f);

    cloud.points.push_back(std::move(pt));
  }

  sensor_msgs::PointCloud2 out;
  pcl::toROSMsg(cloud, out);

  return out;
}

double getOutputError(const reach_msgs::ReachDatabase& db,
                      const rbf_interpolation::RBFSolver<double>& rbf_solver)
{
  using RBFMatrix = rbf_interpolation::RBFMatrixX<double>;
  RBFMatrix test_input (db.records.size(), 3);
  for(std::size_t i = 0; i < db.records.size(); ++i)
  {
    const reach_msgs::ReachRecord& rec = db.records[i];
    test_input.row(i) << rec.goal.position.x, rec.goal.position.y, rec.goal.position.z;
  }

  std::vector<double> test_output = rbf_solver.calculateOutput(test_input);
  double error = 0.0;
  for(std::size_t i = 0; i < db.records.size(); ++i)
  {
    double e = std::abs(db.records[i].score - test_output[i]);
//    ROS_INFO("Current error: %f", e);
    error += e;
  }

  return error;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rbf_test_node");
  ros::NodeHandle pnh("~"), nh;

  std::string db_filename, mesh_filename, fixed_frame, object_frame;
  pnh.param<std::string>("db_filename", db_filename, "/home/mripperger/ros/robot_reach_study/src/10.22710/basf_support/reach_study/results/truck_update_2/optimized_reach.db");
  pnh.param<std::string>("mesh_filename", mesh_filename, "/home/mripperger/ros/robot_reach_study/src/10.22710/basf_support/meshes/basf_support/visual/ram_1500.ply");
  pnh.param<std::string>("fixed_frame", fixed_frame, "world");
  pnh.param<std::string>("object_frame", object_frame, "reach_object");

  double shaping_factor;
  pnh.param<double>("shaping_factor", shaping_factor, 10.0);

  reach_msgs::ReachDatabase db;
  try
  {
    if(!reach::utils::fromFile(db_filename, db))
    {
      ROS_ERROR("Failed to load database from '%s'", db_filename.c_str());
      return -1;
    }
  }
  catch (const std::exception &ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return -1;
  }

  pcl::PolygonMesh mesh;
  if(pcl::io::loadPLYFile(mesh_filename, mesh) != 0)
  {
    ROS_ERROR("Failed to load PLY mesh from '%s'", mesh_filename.c_str());
    return -2;
  }
  ROS_INFO_STREAM(mesh.cloud.height * mesh.cloud.width << " mesh vertices");

  // Transform the mesh
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);

  // Transform the point cloud into the correct frame
  tf::TransformListener tf_listener;
  tf::StampedTransform tf_transform;
  if(!tf_listener.waitForTransform(fixed_frame, object_frame, ros::Time(0), ros::Duration(3.0)))
  {
    ROS_ERROR_STREAM("Failed to get transform from " << fixed_frame << " to " << object_frame);
    return -1;
  }
  try
  {
    tf_listener.lookupTransform(fixed_frame, object_frame, ros::Time(0), tf_transform);
  }
  catch(const tf::TransformException& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return -1;
  }
  Eigen::Affine3d frame;
  tf::transformTFToEigen(tf_transform, frame);
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  pcl::transformPointCloud(cloud, transformed_cloud, frame);

  // Put the results of the database into a container
  using RBFDataMap = rbf_interpolation::DataMap<double>;
  using RBFVector = rbf_interpolation::RBFVectorX<double>;
  using RBFMatrix = rbf_interpolation::RBFMatrixX<double>;

  RBFDataMap knots;
  knots.reserve(db.records.size());

  ROS_INFO("%lu data points in database", db.records.size());
  for(const reach_msgs::ReachRecord& rec : db.records)
  {
    RBFVector pt;
    pt.resize(3);
    pt[0] = rec.goal.position.x;
    pt[1] = rec.goal.position.y;
    pt[2] = rec.goal.position.z;
    double score = rec.reached ? rec.score : 0.0;
    knots.push_back(std::pair<double, RBFVector> {score, std::move(pt)});
  }

  // Put the mesh vertices into a container
//  rbf_interpolation::RBFSolver::DataVector output_eval_pts;
//  for(const pcl::PointXYZ pt : transformed_cloud.points)
//  {
//    Eigen::VectorXd eval_pt;
//    eval_pt.resize(3);
//    eval_pt[0] = double(pt.x);
//    eval_pt[1] = double(pt.y);
//    eval_pt[2] = double(pt.z);
//    output_eval_pts.push_back(std::move(eval_pt));
//  }

  RBFMatrix output_eval_pts(transformed_cloud.points.size(), 3);
  for(std::size_t i = 0; i < transformed_cloud.points.size(); ++i)
  {
    const pcl::PointXYZ& pt = transformed_cloud.points[i];
    output_eval_pts.row(i) << (double)pt.x, (double)pt.y, (double)pt.z;
  }

// rbf_interpolation::PolyharmonicRBF<double>::Ptr rbf (new rbf_interpolation::PolyharmonicRBF<double> (3));
// rbf_interpolation::MultiquadricRBF<double>::Ptr rbf (new rbf_interpolation::MultiquadricRBF<double> (shaping_factor));
//  rbf_interpolation::InverseMultiquadricRBF<double>::Ptr rbf (new rbf_interpolation::InverseMultiquadricRBF<double> (shaping_factor));
  rbf_interpolation::ThinPlateSplineRBF<double>::Ptr rbf (new rbf_interpolation::ThinPlateSplineRBF<double>());
//  rbf_interpolation::GaussianRBF<double>::Ptr rbf (new rbf_interpolation::GaussianRBF<double> (shaping_factor));
  rbf_interpolation::RBFSolver<double> rbf_solver (rbf);

  ROS_INFO_STREAM("Starting RBF calculation...");
  ros::WallTime start (ros::WallTime::now());

  rbf_solver.setInputData(knots);
  rbf_solver.calculateWeights();
  std::vector<double> output = rbf_solver.calculateOutput(output_eval_pts);

  ros::WallTime end (ros::WallTime::now());
  ros::WallDuration delta = end - start;


  ROS_INFO("RBF calculation took %f seconds", delta.toSec());

//  for(std::size_t i = 0; i < output.size(); ++i)
//  {
//    ROS_INFO("Score: %f", output[i]);
//  }

  double error = getOutputError(db, rbf_solver);
  ROS_INFO("Total error is: %f", error);
  ROS_INFO("Average error at the seed points is: %f", error / double(db.records.size()));

  // Colorize the knots for display
  std::vector<double> knot_color_input;
  knot_color_input.reserve(db.records.size());
  for(const reach_msgs::ReachRecord& rec : db.records)
  {
    knot_color_input.push_back(rec.score);
  }
  std::vector<reach_visualizer::utils::rgb> knot_colors = reach_visualizer::utils::scoresToColors(knot_color_input);
  sensor_msgs::PointCloud2 knots_msg = toColorizedPointCloud(db.records, knot_colors);
  knots_msg.header.frame_id = fixed_frame;
  knots_msg.header.stamp = ros::Time::now();

  // Publish the point cloud of the knots
  ros::Publisher knots_pub = nh.advertise<sensor_msgs::PointCloud2>("knots", 1, true);
  knots_pub.publish(knots_msg);

  // Colorize the mesh vertices
  auto max_score_it = std::max_element(knot_color_input.begin(), knot_color_input.end());
  std::vector<reach_visualizer::utils::rgb> mesh_colors = reach_visualizer::utils::scoresToColors(output, *max_score_it);
  colorized_mesh_display::ColorizedMeshStamped colorized_mesh = toColorizedMesh(mesh, object_frame, mesh_colors);

  // Create and publish the colorized mesh and knots
  ros::Publisher pub = nh.advertise<colorized_mesh_display::ColorizedMeshStamped>("colorized_mesh", 1, true);
  pub.publish(colorized_mesh);

  ros::spin();

  return 0;
}
