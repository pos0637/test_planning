#ifndef __RESAMPLING_H__
#define __RESAMPLING_H__

#include <pcl/point_cloud.h>
#include "common.h"

/**
 * @brief 重采样
 * 
 * @param cloud 点云
 * @param radius 邻近点半径
 * @return pcl::PointCloud<PointT>::Ptr 点云
 */
pcl::PointCloud<PointT>::Ptr Resampling(pcl::PointCloud<PointT>::Ptr cloud, float radius);

#endif // __RESAMPLING_H__
