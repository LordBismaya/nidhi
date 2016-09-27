#ifndef __NIDHI_DATASTRUCT_H
#define __NIDHI_DATASTRUCT_H

#include <nidhi/nidhi_headers.h>


class ImageFrame
{
public:
	ImageFrame(int pyra_levels);
	~ImageFrame();
	int pyramid_levels;

	cv::Mat image_frame;
	cv::Mat image_gray;

	std::vector<cv::Mat> image_pyr,grad_pyr;

	bool isKF;

	cv::Mat R,t,t_r;

	vector<cv::Mat> points;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr;

    
	
	void setFrame(cv::Mat image_frame_inp);
	void calc_GrayImage(cv::Mat image_frame);
	void calc_GradPyramids(cv::Mat image_gray);
	void computePointsImage();
	cv::Mat calcRot(cv::Mat KF_rot);
	cv::Mat calcTrans(cv::Mat KF_trans);
	pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ();
	void type2str(cv::Mat M);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr MatToPoinXYZ2();

};
#endif