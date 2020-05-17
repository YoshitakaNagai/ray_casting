#ifndef __RAY_CASTING_H
#define __RAY_CASTING_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/common/pca.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#include "ray_casting/RayCasting.h"
#include "ray_casting/Format.h"
#include "ray_casting/Voxel.h"
#include "ray_casting/Field.h"
#include "ray_casting/IX.h"
#include "ray_casting/IY.h"
#include "ray_casting/IZ.h"
#include "ray_casting/IH.h"
#include "ray_casting/AngleID.h"
#include "ray_casting/List.h"
#include "ray_casting/LidarPosition.h"


typedef pcl::PointXYZINormal PointINormal;
typedef pcl::PointCloud<PointINormal> CloudINormal;
typedef pcl::PointCloud<PointINormal>::Ptr CloudINormalPtr;

class RayCasting
{
	public:
		RayCasting(void);

		void execution(void);
		void pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
        pcl::PointCloud<PointINormal>::Ptr pc_downsampling(pcl::PointCloud<PointINormal>::Ptr);
		void lidar_position_msg_callback(const ray_casting::LidarPositionConstPtr&);
        void formatting(void);
        void initialization(void);
        void pre_casting(void);
        void ray_casting(void);
        float deg_to_rad(double);

	private:
		bool pc_callback_flag = false;
        bool lidar_position_msg_callback_flag = false;
		bool first_flag = false;

		constexpr static float Occupied = 1.0, free = 0.0, Unknown = 0.5;

		double Hz;

        double LENGTH, WIDTH, HEIGHT; // x, y, z;
        int VOXEL_NUM_X, VOXEL_NUM_Y, VOXEL_NUM_Z;
        float voxel_size_x, voxel_size_y, voxel_size_z;

        double RAY_FOV_YAW;
        int RAY_NUM_YAW;
        float ray_delta_yaw;

        double LOWER_RAY_FOV_PITCH, UPPER_RAY_FOV_PITCH; // deg
        double lower_ray_fov_pitch, upper_ray_fov_pitch; // rad

        ros::NodeHandle n;
        ros::NodeHandle nh;
        
		ros::Subscriber pc_subscriber;
		ros::Subscriber lidar_position_subscriber;

		ros::Publisher raycast_msg_publisher;
		
		tf::TransformListener listener;
		tf::StampedTransform transform;
 
        CloudINormalPtr pcl_filtered_pc {new CloudINormal};

        Eigen::Vector3f zero_vector = Eigen::Vector3f::Zero();

        ray_casting::RayCasting raycast_msg;
        ray_casting::LidarPosition lidar_position_msg;

        struct PreCast{
            int ix;
            int iy;
            int iz;
            float dist;
            float phi;
            float theta;
        };
        struct List{
            std::vector<PreCast> list;
        };
        std::vector<std::vector<List> > precast;
};

#endif// __RAY_CASTING_H
