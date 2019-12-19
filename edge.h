#ifndef __EDGE_H__
#define __EDGE_H__

#include <pcl/point_cloud.h>
#include "common.h"
#include "obb.h"

/**
 * @brief 投影
 * 
 * @param cloud 点云
 * @param obb 最小包围盒
 * @return pcl::PointCloud<PointT>::Ptr 投影点云
 */
pcl::PointCloud<PointT>::Ptr Projection(pcl::PointCloud<PointT>::Ptr cloud, const OBB &obb);

/**
 * @brief 边缘查找
 * 
 * @param cloud 点云
 * @param obb 最小包围盒
 * @return pcl::PointCloud<PointT>::Ptr 边缘点云
 */
pcl::PointCloud<PointT>::Ptr EdgeDetect(pcl::PointCloud<PointT>::Ptr cloud, const OBB &obb);

#endif // __EDGE_H__
