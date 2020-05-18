#include "ray_casting/visualizing.h"


Visualizing::Visualizing(void)
: nh("~")
{
    nh.param("Hz", Hz, 100.0);
    nh.param("LENGTH", LENGTH, 50.0); // ->X
    nh.param("WIDTH", WIDTH, 50.0); // ->Y
    nh.param("HEIGHT", HEIGHT, 2.0); // ->Z
    nh.param("VOXEL_NUM_X", VOXEL_NUM_X, 500);
    nh.param("VOXEL_NUM_Y", VOXEL_NUM_Y, 500);
    nh.param("VOXEL_NUM_Z", VOXEL_NUM_X, 20);
    nh.param("RAY_FOV_YAW", RAY_FOV_YAW, 2*M_PI);
    nh.param("RAY_NUM_YAW", RAY_NUM_YAW, 360);
    nh.param("UPPER_RAY_FOV_PITCH", UPPER_RAY_FOV_PITCH, 10.67);
    nh.param("LOWER_RAY_FOV_PITCH", LOWER_RAY_FOV_PITCH, -30.67);
    // nh.param("", , );

    raycast_msg_subscriber = n.subscribe("/raycast_msg", 10, &Visualizing::lidar_position_msg_callback, this);
	
	raycast_viz_publisher = n.advertise<ray_casting::Visualizing>("/raycast_msg", 10);
}


void Visualizing::execution(void)
{
    formatting();

	ros::Rate r(Hz);
	while(ros::ok()){
        initialization();

		if(raycast_msg_callback_flag){

            raycast_viz_publisher.publish(________);
		}
		r.sleep();
		ros::spinOnce();
	}
}


void Visualizing::raycast_msg_callback(const ray_casting::RayCastingConstPtr &msg)
{
    raycast_msg = *msg;
    raycast_msg_callback_flag = true;
}


void Visualizing::formatting(void)
{
    upper_ray_fov_pitch = deg_to_rad(UPPER_RAY_FOV_PITCH);
    lower_ray_fov_pitch = deg_to_rad(LOWER_RAY_FOV_PITCH);

    // Field Setting for ray casting

    voxel_size_x = (float)LENGTH / (float)VOXEL_NUM_X;
    voxel_size_y = (float)WIDTH / (float)VOXEL_NUM_Y;
    voxel_size_z = (float)HEIGHT / (float)VOXEL_NUM_Z;

    // Formatting the ray
    ray_delta_yaw = (float)RAY_FOV_YAW / (float)RAY_NUM_YAW;
}


void Visualizing::initialization(void)
{
}


