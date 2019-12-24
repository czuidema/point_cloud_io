/*
 * Read.hpp
 *
 *  Created on: Aug 7, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_msgs/PolygonMesh.h>

#include <std_srvs/Trigger.h>
#include <shape_msgs/Mesh.h>

namespace point_cloud_io {

class Read {
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  explicit Read(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~Read() = default;

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Initializes node.
   */
  bool initialize();

  /*!
   * Read the point cloud from a .ply or .vtk file.
   * @param filePath the path to the .ply or .vtk file.
   * @param pointCloudFrameId the id of the frame of the point cloud data.
   * @return true if successful.
   */
  bool readFile(const std::string& filePath, const std::string& pointCloudFrameId);

  /*!
   * Timer callback function.
   * @param timerEvent the timer event.
   */
  void timerCallback(const ros::TimerEvent& timerEvent);

  bool readServiceCallback(std_srvs::Trigger::Request &req,
                                   std_srvs::Trigger::Response &res);

  /*!
   * Publish the point cloud as a PointCloud2.
   * @return true if successful.
   */
  bool publish();

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Point cloud message to publish.
  pcl::PolygonMesh::Ptr polygonMesh_;
  pcl_msgs::PolygonMesh polygonMeshMessage_;

  //! Point cloud publisher.
  ros::Publisher polygonMeshPublisher_;

  //! Timer for publishing the point cloud.
  ros::Timer timer_;

  //! Path to the point cloud file.
  std::string filePath_;

  //! Point cloud topic to be published at.
  std::string polygonMeshTopic_;

  //! Point cloud frame id.
  std::string pointCloudFrameId_;

  /*!
   * If true, continuous publishing is used.
   * If false, point cloud is only published once.
   */
  bool isContinuouslyPublishing_ = false;

  //! Duration between publishing steps.
  ros::Duration updateDuration_;

  // service
  ros::ServiceServer pub_srv_;
};

}  // namespace point_cloud_io
