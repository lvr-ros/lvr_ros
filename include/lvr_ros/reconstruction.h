/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2013 University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * reconstruction.h
 *
 * Author: Sebastian Pütz <spuetz@uos.de>,
 *
 */

#ifndef LVR_ROS_RECONSTRUCTION_H_
#define LVR_ROS_RECONSTRUCTION_H_

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include "lvr_ros/ReconstructionConfig.h"


#include <mesh_msgs/TriangleMesh.h>
#include <mesh_msgs/TriangleMeshStamped.h>

#include <lvr/io/PointBuffer.hpp>
#include <lvr/io/MeshBuffer.hpp>

namespace lvr_ros{

class Reconstruction{
  public:
  	Reconstruction();
    ~Reconstruction();
  protected:
  
  private:

  	void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
	bool createMesh(const sensor_msgs::PointCloud2& cloud, mesh_msgs::TriangleMeshStamped& mesh);
	bool createMesh(lvr::PointBufferPtr& point_buffer, lvr::MeshBufferPtr& mesh_buffer);
	float* getStatsCoeffs(std::string filename)const;

  	ros::NodeHandle node_handle;
  	ros::Publisher mesh_publisher;
  	ros::Subscriber cloud_subscriber;
  	ReconstructionConfig config;

};


} /* namespace lvr_ros */

#endif /* LVR_ROS_RECONSTRUCTION_H_ */