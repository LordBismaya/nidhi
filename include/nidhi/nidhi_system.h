#ifndef __NIDHI_SYSTEM_H
#define __NIDHI_SYSTEM_H

#include <nidhi/nidhi_headers.h>
#include <nidhi/nidhi_tracker.h>
#include <nidhi/nidhi_datastruct.h>

class nidhi_system
{
public:
	nidhi_system();
    ~nidhi_system();
	void imgcb(const sensor_msgs::Image::ConstPtr& msg);
	NidhiTracker semidense_tracker;

	
	int frame_id;
    double stamps;
    cv::Mat image_frame,image_frame_aux;
    double depth_stamps;
    ros::Time current_time,stamps_ros;

	ros::NodeHandle nh;
	image_transport::Subscriber sub_image;
	image_transport::Publisher pub_image;

	ros::Publisher odom_pub;

};
#endif

