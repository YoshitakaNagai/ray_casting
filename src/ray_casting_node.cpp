#include "ray_casting/ray_casting.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ray_casting");

	RayCasting ray_casting;
	ray_casting.execution();

	return 0;
}
