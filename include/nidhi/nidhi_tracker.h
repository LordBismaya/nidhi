#ifndef __NIDHI_TRACKER_H
#define __NIDHI_TRACKER_H

#include <nidhi/nidhi_headers.h>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>
#include <nidhi/nidhi_datastruct.h>

class NidhiTracker
{
public:
	NidhiTracker();

	cv::Mat poses;
	tf::TransformBroadcaster mTfBfr;

	//Import stuff from yaml.
	cv::Mat distCoeffs,cameraMatrix;
    int distortion,reduction;
    int pyramid_levels;

	//Frame No. and KF. no.
	int frame_curr_n,frame_kf_n;
	
	//returned from Nidhi System
	cv::Mat *image_frame;
	int *cont_frames;
	double *stamps;
    ros::Time *stamps_ros;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr_t;
    boost::mutex viz_mutex;
    
};
void ThreadSemiDenseTracker(NidhiTracker *semidense_tracker,ros::Publisher *odom_pub,image_transport::Publisher *pub_image);
///semidense_tracking function
void semidense_tracking(ImageFrame *KeyFrame,ImageFrame *CurrentFrame,NidhiTracker *semidense_tracker,ros::Publisher *odom_pub,image_transport::Publisher *pub_image);
///prepare the image for tracking
//void prepare_image(cv::Mat &image_frame, cv::Mat &image_rgb,cv::Mat &image_to_track,int &image_n,cv::Mat &image_gray,cv::Mat cameraMatrix,cv::Mat distCoeffs,\
                   double &fx,double &fy, double &cx, double &cy, int distortion, int reduction);
void visualizePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr);



#endif