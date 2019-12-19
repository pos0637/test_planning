#ifndef __ROI_H__
#define __ROI_H__

#include <pcl/point_cloud.h>
#include "common.h"

/**
 * @brief 提取感兴趣区域点云
 * 
 * @param cloud 点云
 * @param min 感兴趣区域最小点
 * @param max 感兴趣区域最大点
 * @return pcl::PointCloud<PointT>::Ptr 点云
 */
pcl::PointCloud<PointT>::Ptr ROI(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointXYZ min, pcl::PointXYZ max);

#endif // __ROI_H__
