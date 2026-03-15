/**
 * @file IMAGE2CLOUDPROCESSING.cpp
 * @brief 
 */
#include "module/image2cloud_processing.h"
#include <string>
#include <iostream>
#include <math.h>

IMAGE2CLOUDPROCESSING::IMAGE2CLOUDPROCESSING()
{

}

IMAGE2CLOUDPROCESSING::~IMAGE2CLOUDPROCESSING()
{

}


void IMAGE2CLOUDPROCESSING::extractCloudFromMaskImage(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_scan, const cv::Mat &img_tmp, const size_t &sampling_num, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_mask)
{
    pcl::PointXYZRGBA pt_tmp;
    for (size_t x = 0; x < img_tmp.cols; x++) // 1920
    {
        for (size_t y = 0; y < img_tmp.rows; y++) // 1200
        {
            if (img_tmp.channels() == 1)
            {
                // img_tmp.at<cv::uchar>(x, y) = 255;
            }
            else if (img_tmp.channels() == 3)
            {
                int b = img_tmp.at<cv::Vec3b>(y, x)[0]; // b
                int g = img_tmp.at<cv::Vec3b>(y, x)[1]; // g
                int r = img_tmp.at<cv::Vec3b>(y, x)[2]; // r

                if (b < 1 && g < 1 && r < 1)
                {
                }
                else
                {
                    pt_tmp.x = cloud_scan.points[img_tmp.cols * y + x].x;
                    pt_tmp.y = cloud_scan.points[img_tmp.cols * y + x].y;
                    pt_tmp.z = cloud_scan.points[img_tmp.cols * y + x].z;
                    pt_tmp.r = cloud_scan.points[img_tmp.cols * y + x].r;
                    pt_tmp.g = cloud_scan.points[img_tmp.cols * y + x].g;
                    pt_tmp.b = cloud_scan.points[img_tmp.cols * y + x].b;
                    if (std::isnan(pt_tmp.x) == 0 && std::isnan(pt_tmp.y) == 0 && std::isnan(pt_tmp.z) == 0)
                    {
                        // Downsampling in scan cloud - 중간 픽셀만 선택
                        if (x % sampling_num == 0 && y % sampling_num == 0)
                        {
                            cloud_mask.points.push_back(pt_tmp);
                        }
                    }
                }
            }
        }
    }
}


void IMAGE2CLOUDPROCESSING::extractCloudFromMaskImageWithMargin(size_t pixel_margin, const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_scan, const cv::Mat &img_tmp, const size_t &sampling_num, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_mask)
{
    pcl::PointXYZRGBA pt_tmp;
    // size_t pixel_margin = 15;
    int b,g,r;
    cv::Mat img_check = img_tmp;
    for (size_t x = 0; x < img_tmp.cols; x++) // 1920
    {
        for (size_t y = 0; y < img_tmp.rows; y++) // 1200
        {
            if (img_tmp.channels() == 1)
            {
                // img_tmp.at<cv::uchar>(x, y) = 255;
            }
            else if (img_tmp.channels() == 3)
            {
                b = img_tmp.at<cv::Vec3b>(y, x)[0]; // b
                g = img_tmp.at<cv::Vec3b>(y, x)[1]; // g
                r = img_tmp.at<cv::Vec3b>(y, x)[2]; // r

                if (b < 1 && g < 1 && r < 1)
                {
                }
                else
                {
                    pt_tmp.x = cloud_scan.points[img_tmp.cols * y + x].x;
                    pt_tmp.y = cloud_scan.points[img_tmp.cols * y + x].y;
                    pt_tmp.z = cloud_scan.points[img_tmp.cols * y + x].z;
                    pt_tmp.r = cloud_scan.points[img_tmp.cols * y + x].r;
                    pt_tmp.g = cloud_scan.points[img_tmp.cols * y + x].g;
                    pt_tmp.b = cloud_scan.points[img_tmp.cols * y + x].b;
                    if (std::isnan(pt_tmp.x) == 0 && std::isnan(pt_tmp.y) == 0 && std::isnan(pt_tmp.z) == 0)
                    {
                        // Downsampling in scan cloud - 중간 픽셀만 선택
                        if (x % sampling_num == 0 && y % sampling_num == 0)
                        {
                            // printf("(r,g,b) = %i, %i, %i\n", r, g, b);
                            cloud_mask.points.push_back(pt_tmp);

                            // pixel margin - 경계 부근에서 mask로 할당되지 않은 픽셀의 점을 추가
                            // Mask RCNN의 경계 부근에서 잘리는 경우 방지, Template matching에서 Euclidean filtering으로 노이즈는 제거됨.
                            if((x-pixel_margin) > 0 && (y-pixel_margin) > 0 && (x+pixel_margin) < img_tmp.cols && (y+pixel_margin) < img_tmp.rows)
                            {
                                size_t idx_y, idx_x;
                                for (size_t i = 0; i < 2*pixel_margin+1; i++)
                                {
                                    for (size_t j = 0; j < 2*pixel_margin+1; j++)
                                    {
                                        idx_x = x+i-pixel_margin;
                                        idx_y = y+j-pixel_margin;
                                        if (idx_y % sampling_num == 0 && idx_x % sampling_num == 0) 
                                        { 
                                            b = img_check.at<cv::Vec3b>(idx_y, idx_x)[0]; // b
                                            g = img_check.at<cv::Vec3b>(idx_y, idx_x)[1]; // g
                                            r = img_check.at<cv::Vec3b>(idx_y, idx_x)[2]; // r
                                            if (b < 1 && g < 1 && r < 1)
                                            {
                                                pt_tmp.x = cloud_scan.points[img_tmp.cols * idx_y + idx_x].x;
                                                pt_tmp.y = cloud_scan.points[img_tmp.cols * idx_y + idx_x].y;
                                                pt_tmp.z = cloud_scan.points[img_tmp.cols * idx_y + idx_x].z;
                                                pt_tmp.r = cloud_scan.points[img_tmp.cols * idx_y + idx_x].r;
                                                pt_tmp.g = cloud_scan.points[img_tmp.cols * idx_y + idx_x].g;
                                                pt_tmp.b = cloud_scan.points[img_tmp.cols * idx_y + idx_x].b;
                                                if (std::isnan(pt_tmp.x) == 0 && std::isnan(pt_tmp.y) == 0 && std::isnan(pt_tmp.z) == 0)
                                                {
                                                    cloud_mask.points.push_back(pt_tmp);
                                                    // // mask로 할당
                                                    // img_check.at<cv::Vec3b>(idx_y, idx_x)[0] = 255; // b
                                                    // img_check.at<cv::Vec3b>(idx_y, idx_x)[1] = 255; // g
                                                    // img_check.at<cv::Vec3b>(idx_y, idx_x)[2] = 255; // r
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

