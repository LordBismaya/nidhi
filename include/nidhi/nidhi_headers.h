#ifndef __NIDHI_HEADERS_H
#define __NIDHI_HEADERS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <math.h>
#include <iomanip>
#include <string>
#include <fstream>
#include <unistd.h>

#define _USE_MATH_DEFINES

#include <Eigen/Dense>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <omp.h>
#pragma omp

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <nav_msgs/Odometry.h>

//PCL Libraries
#include <pcl/common/projection_matrix.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

#endif
