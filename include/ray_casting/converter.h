#ifndef __CONVERTER_H
#define __CONVERTER_H

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

#include "ray_casting/LidarPosition.h"


typedef pcl::PointXYZINormal PointINormal;
typedef pcl::PointCloud<PointINormal> CloudINormal;
typedef pcl::PointCloud<PointINormal>::Ptr CloudINormalPtr;

class Converter
{
	public:
		Converter(void);

		void execution(void);
		void raw_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
        void lidar_positioning(void);
        CloudINormalPtr to_voxel_tf(CloudINormalPtr);
	
	private:
		bool raw_pc_callback_flag = false;
		bool converted_pc_callback_flag = false;
		bool tf_listen_flag = false;
		bool tf_ln_flag = false;

		double Hz;
        
        double LENGTH, WIDTH, HEIGHT; // x, y, z;
        int VOXEL_NUM_X, VOXEL_NUM_Y, VOXEL_NUM_Z;
        float  voxel_size_x, voxel_size_y, voxel_size_z;

        ros::NodeHandle n;
        ros::NodeHandle nh;
        
		ros::Subscriber raw_pc_subscriber;
		ros::Subscriber converted_pc_subscriber;

		ros::Publisher converted_pc_publisher;
		ros::Publisher lidar_position_msg_publisher;
		
		tf::TransformListener listener;
        tf::TransformListener ln;
		tf::StampedTransform transform;

        geometry_msgs::PoseStamped lidar_position;

        sensor_msgs::PointCloud2 input_raw_pc;
        
        CloudINormalPtr pcl_input_raw_pc {new CloudINormal};

        ray_casting::LidarPosition lidar_position_msg;
};

#endif// __CONVERTER_H
