#ifndef __ZOOM_H__
#define __ZOOM_H__

#include <pcl/point_cloud.h>
#include "common.h"
#include "obb.h"

/**
 * @brief 沿法线方向收缩点云
 * 
 * @param cloud 点云
 * @param normals 法线
 * @param distance 距离
 * @return std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr>  点云, 法线
 */
std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> Shrink(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, float distance);

/**
 * @brief 多边形收缩点云
 * 
 * @param cloud 点云
 * @param obb 最小包围盒
 * @param distance 距离
 * @param offsetZ Z轴偏移
 * @return std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr>  点云, 法线
 */
std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> Shrink2D(pcl::PointCloud<PointT>::Ptr cloud, const OBB &obb, float distance, float offsetZ);

#endif // __ZOOM_H__
