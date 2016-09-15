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

	int kf_n;

	cv::Mat R,t,t_r;

	vector<cv::Mat> points;
	
	void setFrame(cv::Mat image_frame_inp);
	void calc_GrayImage(cv::Mat image_frame);
	void calc_GradPyramids(cv::Mat image_gray);
	void computePointsImage();


};
#endif