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
 * reconstruction.cpp
 *
 * Author: Sebastian Pütz <spuetz@uos.de>,
 *
 */

#include "lvr_ros/reconstruction.h"
#include "lvr_ros/conversions.h"
#include <lvr/io/Model.hpp>


namespace lvr_ros{

	Reconstruction::Reconstruction(){

		cloud_subscriber = node_handle.subscribe("/pointcloud", 1, &Reconstruction::pointCloudCallback, this);
		mesh_publisher = node_handle.advertise<mesh_msgs::TriangleMeshStamped>("/mesh", 1);
	}

	Reconstruction::~Reconstruction(){}

	void Reconstruction::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
		lvr::ModelPtr model_ptr(new lvr::Model());
		lvr_ros::convertPointCloud2ToModelPtr(*cloud, model_ptr);
	}


} /* namespace lvr_ros */


	int main(int argc, char** args)
	{
		ros::init(argc, args, "lvr_ros_reconstruction");

		ros::Rate loop_rate(10);
		while(ros::ok())
		{
			ros::spinOnce();
			loop_rate.sleep();
		}

		return 0;
	}