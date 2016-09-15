#include <nidhi/nidhi_datastruct.h>

ImageFrame::ImageFrame(int pyra_levels=4)
{
	pyramid_levels=pyra_levels;
	R=(cv::Mat_<double>(3,3)<<1,0,0,0,1,0,0,0,1);
	t=(cv::Mat_<double>(3,1)<<0,0,0);
	image_pyr.resize(pyramid_levels+1);
	grad_pyr.resize(pyramid_levels+1);
	
}
ImageFrame::~ImageFrame()
{
	image_gray.release();
	image_frame.release();
	image_pyr.clear();
	grad_pyr.clear();
	points.clear();
}
void ImageFrame::setFrame(cv::Mat image_frame_inp)
{
	image_frame=(image_frame_inp).clone();
	ImageFrame::calc_GrayImage(image_frame);
	ImageFrame::calc_GradPyramids(image_gray);
	ImageFrame::computePointsImage();
}
void ImageFrame::calc_GrayImage(cv::Mat image_frame)
{
	cv::Mat auxIm=image_frame.clone();
	cv::cvtColor(auxIm, image_gray, CV_BGR2GRAY);
}
void ImageFrame::calc_GradPyramids(cv::Mat image_gray)
{
	//calculate and store the image_pyramids
	image_pyr[0]=image_gray.clone();
	for (int ii=0;ii<pyramid_levels;ii++)
	{
		cv::pyrDown(image_pyr[ii],image_pyr[ii+1], cv::Size(image_pyr[ii].cols/2, image_pyr[ii].rows/2));
		
	}
	//calculate the gradients in the topmost pyramid
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	cv::Mat grad_x, grad_y,grad;
	cv::Mat abs_grad_x, abs_grad_y;

	cv::Sobel(image_pyr[pyramid_levels], grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
	cv::convertScaleAbs( grad_x, abs_grad_x );
	cv::Sobel(image_pyr[pyramid_levels], grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
	cv::convertScaleAbs( grad_y, abs_grad_y );
	addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
	threshold( grad, grad,20,255,0);

	//scale up the gradient and store in the grad pyramid
	grad_pyr[pyramid_levels]=grad.clone();
	for (int ii=pyramid_levels;ii>0;ii--)
	{
		cv::pyrUp(grad_pyr[ii],grad_pyr[ii-1], cv::Size(grad_pyr[ii].cols*2, grad_pyr[ii].rows*2));
		
	}

}
void ImageFrame::computePointsImage()
{
	cv::Mat image_con=grad_pyr[pyramid_levels].clone();
	cv::Mat point_aux=cv::Mat(1,6,CV_64FC1);
	for (int ii=0;ii<image_con.rows;ii++)
		for(int jj=0;jj<image_con.cols;jj++)
		{
			if(image_con.at<double>(ii,jj)>20)
			{
				point_aux.at<double>(0,0)=ii;
				point_aux.at<double>(0,1)=jj;
				point_aux.at<double>(0,2)=0;
				point_aux.at<double>(0,3)=image_pyr[pyramid_levels].at<double>(ii,jj);
				point_aux.at<double>(0,4)=0;
				point_aux.at<double>(0,5)=0;
				
				points.push_back(point_aux);
				
			}
		}
}
