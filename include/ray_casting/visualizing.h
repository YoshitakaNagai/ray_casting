#ifndef __VISUALIZING_H
#define __VISUALIZING_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

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


class Visualizing
{
	public:
		Visualizing(void);

		void execution(void);
		void raycast_msg_callback(const ray_casting::RayCastingConstPtr&);
        void formatting(void);
        void initialization(void);

	private:
        bool raycast_msg_callback_flag = false;
		bool first_flag = false;

		constexpr static float Occupied = 1.0, Free = 0.0, Unknown = 0.5;

		double Hz;

        double LENGTH, WIDTH, HEIGHT; // x, y, z;
        int VOXEL_NUM_X, VOXEL_NUM_Y, VOXEL_NUM_Z;
        int voxel_num;
        float voxel_size_x, voxel_size_y, voxel_size_z;

        ros::NodeHandle n;
        ros::NodeHandle nh;
        
		ros::Subscriber raycast_msg_subscriber;
		ros::Publisher raycast_viz_publisher;

        visualization_msgs::MarkerArray voxel_marker;

        ray_casting::RayCasting raycast_msg;

};

#endif// __VISUALIZING_H
