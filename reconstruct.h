#ifndef __RECONSTRUCT_H__
#define __RECONSTRUCT_H__

#include <pcl/point_cloud.h>
#include "common.h"

/**
 * @brief 曲面重建
 * 
 * @param cloud 点云
 * @param radius 邻近点半径
 * @return pcl::PolygonMesh::Ptr 曲面
 */
pcl::PolygonMesh::Ptr Reconstruct(pcl::PointCloud<PointT>::Ptr cloud, double radius = 5.0f);

#endif // __RECONSTRUCT_H__
