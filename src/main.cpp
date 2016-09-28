#include <nidhi/nidhi_headers.h>
#include <nidhi/nidhi_system.h>
	
int main(int argc, char** argv)
{
	ros::init(argc,argv,"camera_image");
	ros::start();
	ROS_INFO("No. of threads for NIDHI= %d ",boost::thread::hardware_concurrency());
	
	
	nidhi_system nidhi_system_object;

	ros::spin();
	ros::shutdown();
	return 0;
}