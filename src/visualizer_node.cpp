#include "ray_casting/visualizing.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "visualizer");

	Visualizing visualizing;
	visualizing.execution();

	return 0;
}
