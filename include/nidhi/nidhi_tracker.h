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

	cv::Mat distCoeffs,cameraMatrix;
    int distortion,reduction;

	cv::Mat image_rgb,image_to_track,image_gray,image_prev,image_keyframe;

	cv::Mat grad_X,grad_Y,grad;
    int frame_curr_n,frame_kf_n;
	
	cv::Mat *image_frame;
	cv::Mat image_frame_oldest;
//toDEL
	cv::Mat image_gray_0,image_gray_2,image_gray_m;
	cv::Mat image_pyr_1,image_pyr_2,image_pyr_3;
	cv::Mat grad_pyr_1,grad_pyr_2,grad_pyr_3;
//toDEL
	
	int *cont_frames;
    double *stamps;
    ros::Time *stamps_ros;

    int pyramid_levels;

};
void ThreadSemiDenseTracker(NidhiTracker *semidense_tracker,ros::Publisher *odom_pub,image_transport::Publisher *pub_image);
///semidense_tracking function
void semidense_tracking(ImageFrame *KeyFrame,ImageFrame *CurrentFrame,NidhiTracker *semidense_tracker,ros::Publisher *odom_pub,image_transport::Publisher *pub_image);
///prepare the image for tracking
void prepare_image(cv::Mat &image_frame, cv::Mat &image_rgb,cv::Mat &image_to_track,int &image_n,cv::Mat &image_gray,cv::Mat cameraMatrix,cv::Mat distCoeffs,\
                   double &fx,double &fy, double &cx, double &cy, int distortion, int reduction);
#endif