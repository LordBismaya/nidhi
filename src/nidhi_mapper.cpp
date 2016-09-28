#include <nidhi/nidhi_mapper.h>
#include <nidhi/nidhi_system.h>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

NidhiMapper::NidhiMapper()
{
	//point_cloud_ptr_m=0;
}
NidhiMapper::~NidhiMapper()
{}

void ThreadSemiDenseMapper(NidhiMapper *semidense_mapper,NidhiTracker *semidense_tracker)
{
    

    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
    viewer.addCoordinateSystem (100.0, 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    
    while (ros::ok())
    {
		
		while(semidense_tracker->point_cloud_ptr_t!=NULL)
		{	
			viewer.removePointCloud ("original_cloud");
			semidense_tracker->viz_mutex.lock();
			semidense_mapper->point_cloud_ptr_m=semidense_tracker->point_cloud_ptr_t;
			
			//VISUALIZATION
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> source_cloud_color_handler (semidense_mapper->point_cloud_ptr_m);
			viewer.addPointCloud (semidense_mapper->point_cloud_ptr_m, source_cloud_color_handler, "original_cloud");
			//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	  		//VISUALIZATION

			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
			viewer.spinOnce();
			semidense_tracker->viz_mutex.unlock();
		}
    }
}