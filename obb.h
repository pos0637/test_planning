#ifndef __OBB_H__
#define __OBB_H__

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "common.h"

/**
 * @brief 最小包围盒
 */
struct OBB
{
    pcl::PointXYZ min;
    pcl::PointXYZ max;
    pcl::PointXYZ position;
    Eigen::Vector3f majorVector;
    Eigen::Vector3f middleVector;
    Eigen::Vector3f minorVector;
    pcl::PointXYZ center;
    Eigen::Matrix3f rotationMatrix;
    Eigen::Matrix4f transformMatrix;
    Eigen::Matrix4f inverseTransformMatrix;
};

/**
 * @brief 计算变换矩阵
 * 
 * @param rotationMatrix 旋转矩阵
 * @return Eigen::Matrix4f 变换矩阵
 */
Eigen::Matrix4f GenerateTransformMatrix(const Eigen::Matrix3f &rotationMatrix);

/**
 * @brief 计算变换矩阵
 * 
 * @param center 中心点
 * @return Eigen::Matrix4f 变换矩阵
 */
Eigen::Matrix4f GenerateTransformMatrix(const pcl::PointXYZ &center);

/**
 * @brief 计算变换矩阵
 * 
 * @param rotationMatrix 旋转矩阵
 * @param center 中心点
 * @return Eigen::Matrix4f 变换矩阵
 */
Eigen::Matrix4f GenerateTransformMatrix(const Eigen::Matrix3f &rotationMatrix, const pcl::PointXYZ &center);

/**
 * @brief 计算逆变换矩阵
 * 
 * @param rotationMatrix 旋转矩阵
 * @param center 中心点
 * @return Eigen::Matrix4f 逆变换矩阵
 */
Eigen::Matrix4f GenerateInverseTransformMatrix(const Eigen::Matrix3f &rotationMatrix, const pcl::PointXYZ &center);

/**
 * @brief 计算最小包围盒
 * 
 * @param cloud 点云
 * @return OBB 最小包围盒
 */
OBB ComputeOBB(const pcl::PointCloud<PointT>::Ptr cloud);

/**
 * @brief 点云对齐到坐标系
 * 
 * @param cloud 点云
 * @param obb 最小包围盒
 * @return pcl::PointCloud<PointT>::Ptr 对齐后点云
 */
pcl::PointCloud<PointT>::Ptr AlignToCenter(const pcl::PointCloud<PointT>::Ptr cloud, const OBB &obb);
/**
 * @brief 绘制最小包围盒坐标系
 * 
 * @param viewer 视图
 * @param obb 最小包围盒
 * @param id 名称
 */
void DrawOBBAxis(const pcl::visualization::PCLVisualizer::Ptr viewer, const OBB &obb, const std::string &id = "");
/**
 * @brief 绘制最小包围盒
 * 
 * @param viewer 视图
 * @param obb 最小包围盒
 * @param id 名称
 */
void DrawOBB(const pcl::visualization::PCLVisualizer::Ptr viewer, const OBB &obb, const std::string &id = "");

#endif // __OBB_H__
