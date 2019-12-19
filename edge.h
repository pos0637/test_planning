#ifndef __EDGE_H__
#define __EDGE_H__

#include <pcl/point_cloud.h>
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

/**
 * @brief 匹配候选点云
 * 
 * @param cloud 点云
 * @param normals 法线
 * @param candidate 候选点云
 * @return std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> 匹配候选点云, 匹配候选法线
 */
std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> MatchCandidateCloud(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<PointT>::Ptr candidate);

/**
 * @brief 沿法线方向收缩点云
 * 
 * @param cloud 点云
 * @param normals 法线
 * @param distance 距离
 * @return pcl::PointCloud<PointT>::Ptr 点云
 */
pcl::PointCloud<PointT>::Ptr ShrinkEdge(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, float distance);

/**
 * @brief 点云投影排序
 * 
 * @param cloud 点云
 * @param normals 法线
 * @param obb 最小包围盒
 * @param axis 主轴
 * @return std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr>  点云, 法线
 */
std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> Sort(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, const OBB &obb, const std::string &axis);

#endif // __EDGE_H__
