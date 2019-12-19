#ifndef __EDGE2D_H__
#define __EDGE2D_H__

#include <pcl/point_cloud.h>
#include <opencv2/core.hpp>
#include "common.h"
#include "obb.h"

/**
 * @brief 投影
 *
 * @param cloud 点云
 * @param obb 最小包围盒
 * @param axis 主轴
 * @param padding 边距
 * @return cv::Mat 深度图
 */
cv::Mat Projection2D(pcl::PointCloud<PointT>::Ptr cloud, const OBB &obb, const std::string &axis = "x", int padding = 0);

/**
 * @brief 投影
 *
 * @param image 深度图
 * @param axis 主轴
 * @param deltaX X补偿值
 * @param deltaY Y补偿值
 * @param deltaZ Z补偿值
 * @return pcl::PointCloud<PointT>::Ptr 点云
 */
pcl::PointCloud<PointT>::Ptr Projection3D(const cv::Mat &image, const std::string &axis = "x", int deltaX = 0, int deltaY = 0, float deltaZ = 0.0f);

/**
 * @brief 查找边缘
 *
 * @param cloud 点云
 * @param obb 最小包围盒
 * @return pcl::PointCloud<PointT>::Ptr 边缘点云
 */
pcl::PointCloud<PointT>::Ptr EdgeDetect2D(pcl::PointCloud<PointT>::Ptr cloud, const OBB &obb);

#endif // __EDGE2D_H__
