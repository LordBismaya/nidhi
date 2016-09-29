#include <nidhi/nidhi_tracker.h>
#include <nidhi/nidhi_system.h>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

NidhiTracker::NidhiTracker()
{
   cv::FileStorage  fs2( (ros::package::getPath("nidhi")+"/src/data.yml").c_str(), cv::FileStorage::READ);
   fs2["cameraMatrix"] >> cameraMatrix;
   fs2["distCoeffs"] >> distCoeffs;
   fs2["pyramids"] >> pyramid_levels;
   reduction=2;
   fs2.release();
   //frame_curr_n=*cont_frames;
   frame_kf_n=0;
}


void ThreadSemiDenseTracker(NidhiTracker *semidense_tracker,ros::Publisher *odom_pub,image_transport::Publisher *pub_image)
{
    
    ImageFrame KeyFrame(semidense_tracker->cameraMatrix,semidense_tracker->pyramid_levels);
    ImageFrame CurrentFrame(semidense_tracker->cameraMatrix,semidense_tracker->pyramid_levels);
    ImageFrame lastFrame(semidense_tracker->cameraMatrix,semidense_tracker->pyramid_levels);
    while (ros::ok())
    {
		   
		   semidense_tracking(&KeyFrame,&CurrentFrame,semidense_tracker,odom_pub,pub_image);
           boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }
}
void semidense_tracking(ImageFrame *KeyFrame,ImageFrame *CurrentFrame,NidhiTracker *semidense_tracker,\
	ros::Publisher *odom_pub,image_transport::Publisher *pub_image)
{
	if(!(*semidense_tracker->image_frame).empty())
	{
		
		semidense_tracker->frame_curr_n=*semidense_tracker->cont_frames;
		CurrentFrame->setFrame((*semidense_tracker->image_frame).clone());
		
		
		//AON: Set the KF as the very first frame. NEXT:Each 10 frames?
		if((semidense_tracker->frame_kf_n)==0)
		{
			KeyFrame->setFrame((CurrentFrame->image_frame).clone());
			semidense_tracker->frame_kf_n=semidense_tracker->frame_curr_n;
			KeyFrame->isKF=true;
		}

		cv::Mat image_show=(CurrentFrame->grad_pyr[semidense_tracker->pyramid_levels-1-1-1]).clone();
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"mono8",image_show).toImageMsg();
		pub_image->publish(msg);
		
		//visualizePoints(CurrentFrame->point_cloud_ptr);
		semidense_tracker->viz_mutex.lock();

		semidense_tracker->point_cloud_ptr_t=CurrentFrame->point_cloud_ptr;
		semidense_tracker->viz_mutex.unlock();
	}
}
void visualizePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr)
{
	pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> source_cloud_color_handler (point_cloud_ptr, 255, 255, 255);
 	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> source_cloud_color_handler (point_cloud_ptr);
 	 
 	 // We add the point cloud to the viewer and pass the color handler
	viewer.addPointCloud (point_cloud_ptr, source_cloud_color_handler, "original_cloud");
	viewer.addCoordinateSystem (100.0, 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  	while (!viewer.wasStopped ()) 
  	{ // Display the visualiser until 'q' key is pressed
    	viewer.spinOnce ();
  	}

}
/*cv::Mat(cv::Mat image_KF,cv::Mat image_CF)
{
	// Some constants for iterative minimization process.
    const float EPS = 1E-5f; // Threshold value for termination criteria.
    const int MAX_ITER = 100;  // Maximum iteration count.

	//convert to gray images
	cv::cvtColor(image_KF, gray_KF, CV_BGR2GRAY);
	cv::cvtColor(image_CF, gray_CF, CV_BGR2GRAY);
	//calculate gradients
	cv::Mat pGradTx,pGradTy;
	cv::cvSobel(image_KF, pGradTx, 1, 0); // Gradient in X direction
	cv::cvConvertScale(pGradTx, pGradTx, 0.125);
	cv::cvSobel(image_KF, pGradTy, 0, 1); // Gradient in X direction
	cv::cvConvertScale(pGradTy, pGradTy, 0.125);
	cv::Mat pStDesc;
	// Here we will store matrices.
	cv::Mat W = cv::Mat::eye(4, 4, CV_32F);  // Current value of warp W(x,p)
	cv::Mat dW = cv::Mat::zeros(4, 4, CV_32F);  // Warp update.
	cv::Mat idW = cv::Mat::eye(4, 4, CV_32F); // Inverse of warp update.
	cv::Mat X = cv::Mat::zeros(4, 1, CV_32F);  // Point in coordinate frame of T.
	cv::Mat Z = cv::Mat::zeros(4, 1, CV_32F);  // Point in coordinate frame of I.

	cv::Mat H = cv::Mat::zeros(4, 4, CV_32F);  // Hessian.
	cv::Mat iH = cv::Mat::eye(4, 4, CV_32F); // Inverse of Hessian.
	cv::Mat b = cv::Mat::zeros(4, 1, CV_32F);  // Vector in the right side of the system of linear equations.
	cv::Mat delta_p =cv::Mat::zeros(4, 1, CV_32F); // Parameter update value.

	int u,v;
	float u2,v2;

	/*Transform with Calibration Matrix Now*/

	/*Assume it is transformed and proceeding/

}
*/

/*
void prepare_image(cv::Mat &image_frame, cv::Mat &image_rgb,cv::Mat &image_to_track,int &image_n,cv::Mat &image_gray,cv::Mat cameraMatrix,cv::Mat distCoeffs,\
                   double &fx,double &fy, double &cx, double &cy, int distortion, int reduction)
{
    if (image_frame.type()==CV_8UC1) 
    {
		//input image is grayscale
		cv::cvtColor(image_frame, image_frame, CV_GRAY2RGB);
	}

	cv::resize(image_frame,image_frame,cv::Size(image_frame.cols/reduction,image_frame.rows/reduction),0,0,cv::INTER_LINEAR);

	cv::Mat image_ff = image_frame.clone();

	cv::Size ksize,ksize1;
	ksize.width = image_frame.cols;
	ksize.height = image_frame.rows;
	
	cv::Mat newCameraMatrix;

	cv::Mat cameraMatrixAux = cameraMatrix.clone();
	cameraMatrixAux/=reduction;
	cameraMatrixAux.at<double>(2,2)=1;

	double alpha = 0;
	newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrixAux,distCoeffs,ksize,alpha,ksize1);


	cv::undistort(image_frame,image_ff,cameraMatrixAux,distCoeffs,newCameraMatrix);

	image_frame = image_ff.clone();

	fx = newCameraMatrix.at<double>(0,0);
	fy = newCameraMatrix.at<double>(1,1);
	cx = newCameraMatrix.at<double>(0,2);
	cy = newCameraMatrix.at<double>(1,2);


    image_rgb = image_frame.clone();
    image_n++;

    cv::cvtColor(image_rgb,image_to_track,CV_RGB2GRAY);

    image_to_track.convertTo(image_to_track, CV_64FC1);
    image_gray = image_to_track.clone();

    image_to_track /= (255*1.0);
}
*/