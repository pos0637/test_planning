#ifndef __FILTER_H__
#define __FILTER_H__

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "common.h"

/**
 * @brief 统计滤波
 * 
 * @param cloud 点云
 * @param k 邻近点数量
 * @param threshold 阈值
 * @return pcl::PointCloud<PointT>::Ptr 点云
 */
pcl::PointCloud<PointT>::Ptr StatisticalFilter(pcl::PointCloud<PointT>::Ptr cloud, int k = 50, float threshold = 1.0f);

/**
 * @brief 下采样
 * 
 * @param cloud 点云
 * @param size 采样体积
 * @return pcl::PointCloud<PointT>::Ptr 点云
 */
pcl::PointCloud<PointT>::Ptr Downsampling(pcl::PointCloud<PointT>::Ptr cloud, float size = 5.0f);

/**
 * @brief 计算法线
 * 
 * @param cloud 点云
 * @param radius 邻近点半径
 * @param inverse 是否反转
 * @return pcl::PointCloud<pcl::Normal>::Ptr 法线
 */
pcl::PointCloud<pcl::Normal>::Ptr ComputeNormals(pcl::PointCloud<PointT>::Ptr cloud, double radius = 5.0f, bool inverse = false);

/**
 * @brief 计算边界
 * 
 * @param cloud 点云
 * @param radius 邻近点半径
 * @return pcl::PointCloud<PointT>::Ptr 边界点云
 */
pcl::PointCloud<PointT>::Ptr ComputreBoundaries(pcl::PointCloud<PointT>::Ptr cloud, double radius = 10.0f);

/**
 * @brief 求平均值
 * 
 * @param cloud 点云
 * @param axis 轴
 * @return float 平均值
 */
float Mean(pcl::PointCloud<PointT>::Ptr cloud, const std::string &axis);

#endif // __FILTER_H__
