#include <nidhi/nidhi_system.h>
#include <ros/package.h>

nidhi_system::nidhi_system()
{

	cv::FileStorage fs2( (ros::package::getPath("nidhi")+"/src/data.yml").c_str(),cv::FileStorage::READ);

	std::string camera_path;
	fs2["camera_path"] >> camera_path;
	ROS_INFO("(2) (%s) \n",camera_path.c_str());
	frame_id =0;

	image_transport::ImageTransport it(nh);
	sub_image=it.subscribe(camera_path,1, &nidhi_system::imgcb,this);
	pub_image=it.advertise("nidhi/camera/image",1);
	
	odom_pub=nh.advertise<nav_msgs::Odometry>("odom1",50);
	pubPCL= nh.advertise<pcl::PointCloud <pcl::PointXYZRGB> >("points2", 1);

	
	semidense_tracker.image_frame = &image_frame_aux;
	semidense_tracker.cont_frames = &frame_id;
    semidense_tracker.stamps = &stamps;
    semidense_tracker.stamps_ros = &stamps_ros ;
    
            
    boost::thread thread_semidense_tracker(&ThreadSemiDenseTracker,&semidense_tracker,&odom_pub,&pub_image);
    boost::thread thread_semidense_mapper(&ThreadSemiDenseMapper,&semidense_mapper,&semidense_tracker,&pubPCL);
};
nidhi_system::~nidhi_system()
{
	delete semidense_tracker.image_frame;
}

void nidhi_system::imgcb(const sensor_msgs::Image::ConstPtr& msg)
{
	cv_bridge::CvImageConstPtr cv_ptr;
	try{
		//cv_bridge::toCvShare(msg);
		cv_ptr=cv_bridge::toCvShare(msg);
		stamps_ros =  cv_ptr->header.stamp;
        stamps = cv_ptr->header.stamp.toSec();
        //current_time = cv_ptr->header.stamp;
        image_frame_aux =  cv_ptr->image.clone();
		frame_id++;
		//pub_image.publish(cv_ptr->toImageMsg());

}
	catch(const cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s",e.what());
	}
}
