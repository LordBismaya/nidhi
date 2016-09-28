#ifndef __NIDHI_MAPPER_H
#define __NIDHI_MAPPER_H

#include <nidhi/nidhi_headers.h>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>
#include <nidhi/nidhi_datastruct.h>
#include <nidhi/nidhi_tracker.h>

class NidhiMapper
{
public:
	NidhiMapper();
	~NidhiMapper();   
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr_m; 
	
};
void ThreadSemiDenseMapper(NidhiMapper *semidense_mapper,NidhiTracker *semidense_tracker, ros::Publisher *pubPCL);

#endif