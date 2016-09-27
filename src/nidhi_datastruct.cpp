#include <nidhi/nidhi_datastruct.h>

ImageFrame::ImageFrame(int pyra_levels=4)
{
	pyramid_levels=pyra_levels;
	R=(cv::Mat_<double>(3,3)<<1,0,0,0,1,0,0,0,1);
	t=(cv::Mat_<double>(3,1)<<0,0,0);
	image_pyr.resize(pyramid_levels+1);
	grad_pyr.resize(pyramid_levels+1);

	isKF=false;
	
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
	point_cloud_ptr = ImageFrame::MatToPoinXYZ2();
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
/** Points Configuration
0-x
1-y
2-depth
3-intensity
*/
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
				point_aux.at<double>(0,2)=0.05;//set initial depth here
				point_aux.at<double>(0,3)=image_pyr[pyramid_levels].at<double>(ii,jj);
				point_aux.at<double>(0,4)=0;
				point_aux.at<double>(0,5)=0;
				
				points.push_back(point_aux);
				
			}
		}
}
pcl::PointCloud<pcl::PointXYZ>::Ptr ImageFrame::MatToPoinXYZ()
{
	/*
	*  Function: Get from a Mat to pcl pointcloud datatype
	*  In: cv::Mat
	*  Out: pcl::PointCloud
	*/
	//char pr=100, pg=100, pb=100;
	cv::Mat image_con=grad_pyr[0].clone();
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr_aux(new pcl::PointCloud<pcl::PointXYZ>);//(new pcl::pointcloud<pcl::pointXYZ>);
	pcl::PointXYZ pointPCL;

	for (int ii=0;ii<image_con.rows;ii++)
	{
		for(int jj=0;jj<image_con.cols;jj++)
		{
		   	if(image_con.at<double>(ii,jj)>20)
			{
				pointPCL.x = jj;
	   			pointPCL.y = ii;
	   			pointPCL.z = 0.01;
    		// when color needs to be added:
   			//uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
    		//point.rgb = *reinterpret_cast<float*>(&rgb);
	   			point_cloud_ptr_aux -> points.push_back(pointPCL);
    		}

    	}
    }
	point_cloud_ptr_aux->width = (int)point_cloud_ptr_aux->points.size();

	point_cloud_ptr_aux->height = 1;
	return point_cloud_ptr_aux;
}

void ImageFrame::type2str(cv::Mat M)
{
  string r;
  int type=M.type();

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  ROS_INFO("Matrix: %s %dx%d \n", r.c_str(), M.cols, M.rows );
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ImageFrame::MatToPoinXYZ2()
{
	/*
	*  Function: Get from a Mat to pcl pointcloud datatype
	*  In: cv::Mat
	*  Out: pcl::PointCloud
	*/
	//char pr=100, pg=100, pb=100;
	cv::Mat image_con=image_frame.clone();
	cv::Mat image_con_grad=grad_pyr[0].clone();
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr_aux(new pcl::PointCloud<pcl::PointXYZRGB>);//(new pcl::pointcloud<pcl::pointXYZ>);
	pcl::PointXYZRGB pointPCL;
	cv::Vec3b intensity;
	cv::Scalar grad_intensity;

	for (int ii=0;ii<image_con.rows;ii++)
	{
		for(int jj=0;jj<image_con.cols;jj++)
		{
		   	grad_intensity=image_con_grad.at<uchar>(ii,jj);
		   	if(grad_intensity.val[0]>20)
		   	{
			   	intensity = image_con.at<cv::Vec3b>(ii,jj);
			   	pointPCL.x = jj;
		   		pointPCL.y = ii;
		   		pointPCL.z = 0.05;
		   		// pack r/g/b into rgb
				uint8_t r = intensity.val[2];
				uint8_t g = intensity.val[1];
				uint8_t b = intensity.val[0];
				uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
				pointPCL.rgb = *reinterpret_cast<float*>(&rgb);
		   		point_cloud_ptr_aux -> points.push_back(pointPCL);
	    	}		
    	}
    }
	point_cloud_ptr_aux->width = (int)point_cloud_ptr_aux->points.size();

	point_cloud_ptr_aux->height = 1;
	return point_cloud_ptr_aux;
}



