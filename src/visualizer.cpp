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
    // nh.param("", , );

    raycast_msg_subscriber = n.subscribe("/raycast_msg", 10, &Visualizing::raycast_msg_callback, this);
	
	raycast_viz_publisher = n.advertise<visualization_msgs::MarkerArray>("/raycast_msg", 10);
}


void Visualizing::execution(void)
{
    formatting();

	ros::Rate r(Hz);
	while(ros::ok()){
		if(raycast_msg_callback_flag){
            vizualization();
            raycast_viz_publisher.publish(voxel_marker);
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
    voxel_size_x = (float)LENGTH / (float)VOXEL_NUM_X;
    voxel_size_y = (float)WIDTH / (float)VOXEL_NUM_Y;
    voxel_size_z = (float)HEIGHT / (float)VOXEL_NUM_Z;
    voxel_num = VOXEL_NUM_X * VOXEL_NUM_Y * VOXEL_NUM_Z;

    voxel_marker.markers.resize(voxel_num);
}


void Visualizing::visualization(void)
{
    for(int ix = 0; ix < VOXEL_NUM_X; ix++){
        for(int iy = 0; iy < VOXEL_NUM_Y; iy++){
            for(int iz = 0; iz < VOXEL_NUM_Z; iz++){
                voxel_marker.markers[i].header.frame_id = "/base_link";
                voxel_marker.markers[i].header.stamp = ros::Time::now();
                voxel_marker.markers[i].ns = "/ray_casting";
                voxel_marker.markers[i].id = 0;
                voxel_marker.markers[i].type = visualization_msgs::Marker::CUBE;
                voxel_marker.markers[i].action = visualization_msgs::Marker::ADD;
                voxel_marker.markers[i].lifetime = ros::Duration(0);
                voxel_marker.markers[i].pose.position.x = raycast_msg.voxel.ix[ix].iy[iy].iz[iz].center_x;
                voxel_marker.markers[i].pose.position.y = raycast_msg.voxel.ix[ix].iy[iy].iz[iz].center_y;
                voxel_marker.markers[i].pose.position.z = raycast_msg.voxel.ix[ix].iy[iy].iz[iz].center_z;
                voxel_marker.markers[i].pose.orientation.x = 0.0;
                voxel_marker.markers[i].pose.orientation.y = 0.0;
                voxel_marker.markers[i].pose.orientation.z = 0.0;
                voxel_marker.markers[i].pose.orientation.w = 1.0;
                voxel_marker.markers[i].scale.x = voxel_size_x;
                voxel_marker.markers[i].scale.y = voxel_size_y;
                voxel_marker.markers[i].scale.z = voxel_size_z;

                if(raycast_msg.voxel.ix[ix].iy[iy].iz[iz].occupancy == Occupied){
                    voxel_marker.markers[i].color.r = 0.0;
                    voxel_marker.markers[i].color.g = 0.0;
                    voxel_marker.markers[i].color.b = 0.0;
                }
                if(raycast_msg.voxel.ix[ix].iy[iy].iz[iz].occupancy == Free){
                    voxel_marker.markers[i].color.r = 1.0;
                    voxel_marker.markers[i].color.g = 1.0;
                    voxel_marker.markers[i].color.b = 1.0;
                if(raycast_msg.voxel.ix[ix].iy[iy].iz[iz].occupancy == Unknown){
                    voxel_marker.markers[i].color.r = 0.5;
                    voxel_marker.markers[i].color.g = 0.5;
                    voxel_marker.markers[i].color.b = 0.5;
                }
                voxel_marker.markers[i].color.a = 0.5;
            }
        }
    }
}






