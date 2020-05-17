#include "ray_casting/converter.h"

Converter::Converter(void)
: nh("~")
{
    nh.param("Hz", Hz, 100.0);
    nh.param("INITIAL_BUFFER", INITIAL_BUFFER, 10);
    nh.param("LENGTH", LENGTH, 50.0); // ->X
    nh.param("WIDTH", WIDTH, 50.0); // ->Y
    nh.param("HEIGHT", HEIGHT, 2.0); // ->Z
    nh.param("VOXEL_NUM_X", VOXEL_NUM_X, 500);
    nh.param("VOXEL_NUM_Y", VOXEL_NUM_Y, 500);
    nh.param("VOXEL_NUM_Z", VOXEL_NUM_X, 20);
    // nh.param("", , );

    voxel_size_x = (float)LENGTH / (float)VOXEL_NUM_X;
    voxel_size_y = (float)WIDTH / (float)VOXEL_NUM_Y;
    voxel_size_z = (float)HEIGHT / (float)VOXEL_NUM_Z;

    raw_pc_subscriber = n.subscribe("/velodyne_points", 10, &Converter::raw_pc_callback, this);
	
    converted_pc_publisher = n.advertise<sensor_msgs::PointCloud2>("/converted_pc", 10);
	lidar_position_msg_publisher = n.advertise<ray_casting::LidarPosition>("/lidar_position_msg", 10);
}


void Converter::execution(void)
{
	ros::Rate r(Hz);
	while(ros::ok()){
        // To transform point cloud
		try{
       		listener.lookupTransform("/odom","/velodyne", ros::Time(0), transform);
			tf_listen_flag = true;
     	}catch(tf::TransformException ex){
       		ROS_ERROR("%s",ex.what());
       		ros::Duration(1.0).sleep();
    	}

        // To get velodyne position in global
        try{
            geometry_msgs::PoseStamped source_pose;
            source_pose.header.frame_id = "/velodyne";
            source_pose.header.stamp = ros::Time::now();
            source_pose.pose.orientation.w = 1.0;
            std::string target_frame = "/odom";
            ln.waitForTransform(source_pose.header.frame_id, target_frame, source_pose.header.stamp, ros::Duration(1.0));
            ln.transformPose(target_frame, source_pose, lidar_position);
            tf_ln_flag = true;
        }catch(tf::TransformException ex){
       		ROS_ERROR("%s",ex.what());
       		ros::Duration(1.0).sleep();
    	}


		//if(raw_pc_callback_flag && tf_listen_flag && tf_ln_flag){
		if(raw_pc_callback_flag && tf_ln_flag){
            lidar_positioning();
            /*
            CloudINormalPtr pcl_odom_transformed_pc {new CloudINormal};
            CloudINormalPtr pcl_odom_voxel_transformed_pc {new CloudINormal};

			pcl_ros::transformPointCloud("/odom", *pcl_input_raw_pc, *pcl_odom_transformed_pc, listener);
			pcl_odom_voxel_transformed_pc = to_voxel_tf(pcl_odom_transformed_pc);

            sensor_msgs::PointCloud2 converted_pc;
            pcl::toROSMsg(*pcl_odom_voxel_transformed_pc, converted_pc);
            converted_pc.header = input_raw_pc.header;
            converted_pc_publisher.publish(converted_pc);
            */

            lidar_position_msg.header.stamp = ros::Time::now();
            lidar_position_msg.header.frame_id = "/velodyne";
            lidar_position_msg_publisher.publish(lidar_position_msg);

            tf_listen_flag = false;
            raw_pc_callback_flag = false;
		}
		r.sleep();
		ros::spinOnce();
	}
}


void Converter::raw_pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	input_raw_pc = *msg;
	pcl::fromROSMsg(input_raw_pc, *pcl_input_raw_pc);
    raw_pc_callback_flag = true;
}


void Converter::lidar_positioning(void)
{
    lidar_position_msg.global_x = lidar_position.pose.position.x;
    lidar_position_msg.global_y = lidar_position.pose.position.y;
    lidar_position_msg.global_z = lidar_position.pose.position.z;

    lidar_position_msg.voxel_ix = (int)((lidar_position_msg.global_x + 0.5 * LENGTH) / voxel_size_x);
    lidar_position_msg.voxel_iy = (int)((lidar_position_msg.global_y + 0.5 * WIDTH) / voxel_size_y);
    lidar_position_msg.voxel_iz = (int)((lidar_position_msg.global_z + 0.5 * HEIGHT) / voxel_size_z);
}


CloudINormalPtr Converter::to_voxel_tf(CloudINormalPtr pcl_odom_pc)
{
    for(auto& pt : pcl_odom_pc->points){
        pt.x += 0.5 * LENGTH;
        pt.y += 0.5 * WIDTH;
        // pt.z is needless to transform from global to voxel
    }

    return pcl_odom_pc;
}
