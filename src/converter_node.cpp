#include "ray_casting/converter.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "converter");

	Converter converter;
	converter.execution();

	return 0;
}
