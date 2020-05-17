#include "ray_casting/ray_casting.h"


RayCasting::RayCasting(void)
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

    pc_subscriber = n.subscribe("/velodyne_points", 10, &RayCasting::pc_callback, this);
    lidar_position_subscriber = n.subscribe("/lidar_position_msg", 10, &RayCasting::lidar_position_msg_callback, this);
	
	raycast_msg_publisher = n.advertise<ray_casting::RayCasting>("/raycast_msg", 10);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "ray_casting");

	RayCasting ray_casting;
	ray_casting.execution();

	return 0;
}


void RayCasting::execution(void)
{
    formatting();

	ros::Rate r(Hz);
	while(ros::ok()){
        initialization();

		if(pc_callback_flag && lidar_position_msg_callback_flag){
            ray_casting();
            raycast_msg_publisher.publish(raycast_msg);
		}
		r.sleep();
		ros::spinOnce();
	}
}


void RayCasting::pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // sensor_msgs::PointCloud2 input_pc;
    sensor_msgs::PointCloud2::Ptr input_pc{new sensor_msgs::PointCloud2};
    sensor_msgs::PointCloud2::Ptr filtered_pc{new sensor_msgs::PointCloud2};

    input_pc = msg;
	// input_pc = *msg;
	// pcl::fromROSMsg(input_pc, *pcl_input_pc);
    filtered_pc = pc_downsampling(input_pc);
	pcl::fromROSMsg(*filtered_pc, *pcl_filtered_pc);
    pc_callback_flag = true;
}


sensor_msgs::PointCloud2::Ptr RayCasting::pc_downsampling(sensor_msgs::PointCloud2::Ptr input_pc)
{
    sensor_msgs::PointCloud2::Ptr filterd_pc{new sensor_msgs::PointCloud2};
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;

    sor.setInputCloud(input_pc);
    sor.setLeafSize(voxel_size_x, voxel_size_y, voxel_size_z);
    sor.filter(*filtered_pc);

    return filtered_pc;
}


void RayCasting::lidar_position_msg_callback(const ray_casting::LidarPositionConstPtr &msg)
{
    lidar_position_msg = *msg;
    lidar_position_msg_callback_flag = true;
}


void RayCasting::formatting(void)
{
    upper_ray_fov_pitch = deg_to_rad(UPPER_RAY_FOV_PITCH);
    lower_ray_fov_pitch = deg_to_rad(LOWER_RAY_FOV_PITCH);

    // Field Setting for ray casting
    raycast_msg.field.length = LENGTH;
    raycast_msg.field.width = WIDTH;
    raycast_msg.field.height = HEIGHT;

    voxel_size_x = (float)LENGTH / (float)VOXEL_NUM_X;
    voxel_size_y = (float)WIDTH / (float)VOXEL_NUM_Y;
    voxel_size_z = (float)HEIGHT / (float)VOXEL_NUM_Z;
    raycast_msg.voxel.format.num_x = VOXEL_NUM_X;
    raycast_msg.voxel.format.num_y = VOXEL_NUM_Y;
    raycast_msg.voxel.format.num_z = VOXEL_NUM_Z;
    raycast_msg.voxel.format.size_x = voxel_size_x;
    raycast_msg.voxel.format.size_y = voxel_size_y;
    raycast_msg.voxel.format.size_z = voxel_size_z;
    raycast_msg.voxel.ix.resize(VOXEL_NUM_X);
    for(int ix = 0; ix < VOXEL_NUM_X; ix++){
        raycast_msg.voxel.ix[ix].iy.resize(VOXEL_NUM_Y);
        for(int iy = 0; iy < VOXEL_NUM_Y; iy++){
            raycast_msg.voxel.ix[ix].iy[iy].iz.resize(VOXEL_NUM_Z);
            for(int iz = 0; iz < VOXEL_NUM_Z; iz++){
                raycast_msg.voxel.ix[ix].iy[iy].iz[iz].occupancy = 0.5;
            }
        }
    }

    // Formatting the ray
    ray_delta_yaw = (float)RAY_FOV_YAW / (float)RAY_NUM_YAW;
    raycast_msg.ray.num_yaw = RAY_NUM_YAW;
    raycast_msg.ray.fov_yaw = RAY_FOV_YAW;
    raycast_msg.ray.delta_yaw = ray_delta_yaw;

    // Formatting angle id for precasting
    std::vector<List> angle_id;
    angle_id.resize(RAY_NUM_YAW);
    for(int it = 0; it < RAY_NUM_YAW; it++){
        angle_id[it].list.resize(0);
    }
    for(int ih = 0; ih < VOXEL_NUM_Z; ih++){
        precast.push_back(angle_id);
    }

    pre_casting();
}


void RayCasting::initialization(void)
{
    for(int ix = 0; ix < VOXEL_NUM_X; ix++){
        for(int iy = 0; iy < VOXEL_NUM_Y; iy++){
            for(int iz = 0; iz < VOXEL_NUM_Z; iz++){
                raycast_msg.voxel.ix[ix].iy[iy].iz[iz].occupancy = 0.5;
            }
        }
    }
}


void RayCasting::pre_casting(void)
{
    for(int ix = 0; ix < VOXEL_NUM_X; ix++){
        for(int iy = 0; iy < VOXEL_NUM_Y; iy++){
            for(int iz = 0; iz < VOXEL_NUM_Z; iz++){
                float range_x = ix * voxel_size_x - 0.5 * LENGTH;
                float range_y = iy * voxel_size_y - 0.5 * WIDTH;
                float relative_range_z = iz * voxel_size_x - lidar_position_msg.global_z;
                float range_xy = sqrt(range_x * range_x + range_y * range_y);
                float relative_dist = sqrt(range_x * range_x + range_y * range_y + relative_range_z * relative_range_z);
                float relative_theta = atan2(range_y, range_x);
                if(relative_theta < 0){
                    relative_theta += 2 * M_PI;
                }
                float relative_phi = atan2(relative_range_z, range_xy);
                int iyaw = (int)(relative_theta / ray_delta_yaw);
                PreCast precast_data;
                precast_data.ix = ix;
                precast_data.iy = iy;
                precast_data.iz = iz;
                precast_data.dist = relative_dist;
                precast_data.phi = relative_phi;
                precast_data.theta = relative_theta;
                precast[iz][iyaw].list.push_back(precast_data);
            }
        }
    }
}


void RayCasting::ray_casting(void)
{
    for(auto& pt : pcl_filtered_pc->points){
        int ix = (int)((pt.x + 0.5 * LENGTH) / voxel_size_x);
        int iy = (int)((pt.y + 0.5 * WIDTH) / voxel_size_y);
        int iz = (int)(pt.z / voxel_size_z);
        float range_xy = sqrt(pt.x * pt.x + pt.y * pt.y);
        float relative_range_z = pt.z - lidar_position_msg.global_z;
        float relative_dist = sqrt(pt.x * pt.x + pt.y * pt.y + relative_range_z * relative_range_z);
        float relative_theta = atan2(pt.y, pt.x);
        if(relative_theta < 0){
            relative_theta += 2 * M_PI;
        }
        float relative_phi = atan2(relative_range_z, range_xy);
        int iyaw = (int)(relative_theta / ray_delta_yaw);

        for(auto& list : precast[iz][iyaw].list){
            if(list.dist < relative_dist && lower_ray_fov_pitch < list.phi && list.phi < upper_ray_fov_pitch){
                raycast_msg.voxel.ix[list.ix].iy[list.iy].iz[list.iz].occupancy = 0;
            }
        }

        raycast_msg.voxel.ix[ix].iy[iy].iz[iz].occupancy = 1.0;
    }
}


float RayCasting::deg_to_rad(double deg)
{
    return (float)(deg * M_PI / 180);
}
