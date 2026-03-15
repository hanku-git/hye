/**
 * @file image2cloud_processing.h
 * @brief 
 */
#ifdef max
#undef max
#undef min
#endif

#pragma once

#include <iostream>
#include <string>
#include <numeric>

// #include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/pcl_exports.h>
#include <pcl/search/kdtree.h>


#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


/**
 * @class IMAGE2CLOUDPROCESSING
 * @brief 연산 모듈 클래스
 * @ingroup MATH
 */
class IMAGE2CLOUDPROCESSING
{
public: 
	IMAGE2CLOUDPROCESSING();
	~IMAGE2CLOUDPROCESSING();
public:
    void extractCloudFromMaskImage(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_scan, const cv::Mat &img_tmp, const size_t &sampling_num, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_mask);
    void extractCloudFromMaskImageWithMargin(size_t pixel_margin, const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_scan, const cv::Mat &img_tmp, const size_t &sampling_num, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_mask);

};