﻿#ifndef __MISC_H__
#define __MISC_H__

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/core.hpp>
#include "common.h"
#include "obb.h"

/**
 * @brief 绘制坐标系
 * 
 * @param viewer 视图
 */
void DrawCoordinateSystemAxis(const pcl::visualization::PCLVisualizer::Ptr viewer);

/**
 * @brief 显示图像
 *
 * @param name 名称
 * @param image 图像
 */
void ShowImage(const std::string& name, const cv::Mat& image);

/**
 * @brief 显示曲面
 * 
 * @param name 名称
 * @param mesh 曲面
 */
void ShowMesh(const std::string &name, pcl::PolygonMesh::Ptr mesh);

/**
 * @brief 显示点云
 * 
 * @param name 名称
 * @param cloud 点云
 */
void ShowPointCloud(const std::string &name, pcl::PointCloud<PointT>::Ptr cloud);

/**
 * @brief 绘制轮廓点
 * 
 * @param image 图像
 * @param contours 轮廓
 */
void DrawContours(const cv::Mat &image, const std::vector<std::vector<cv::Point>> &contours);

/**
 * @brief 显示轮廓
 * 
 * @param name 名称
 * @param width 宽度
 * @param height 高度
 * @param contours 轮廓
 */
void ShowContours(const std::string &name, int width, int height, const std::vector<std::vector<cv::Point>>& contours);

/**
 * @brief 保存轮廓索引
 * 
 * @param name 名称
 * @param width 宽度
 * @param height 高度
 * @param contours 轮廓
 */
void SaveContoursId(const std::string &name, int width, int height, const std::vector<std::vector<cv::Point>> &contours);

/**
 * @brief 读取轮廓
 * 
 * @param name 文件名
 * @return std::vector<std::vector<cv::Point>> 轮廓
 */
std::vector<std::vector<cv::Point>> LoadContours(const std::string &name);

/**
 * @brief 保存轮廓
 * 
 * @param name 文件名
 * @param contours 轮廓
 */
void SaveContours(const std::string &name, const std::vector<std::vector<cv::Point>> &contours);

/**
 * @brief 保存图像
 * 
 * @param name 文件名
 * @param image 图像
 */
void SaveImage(const std::string &name, const cv::Mat& image);

/**
 * @brief 保存图像值
 * 
 * @param name 名称
 * @param image 图像
 */
void SaveImageValue(const std::string &name, const cv::Mat &image);

/**
 * @brief 保存点云(欧拉角版本)
 * 
 * @param name 名称
 * @param cloud 点云
 * @param normals 法线
 * @param obb 最小包围盒
 */
void SavePointNormals(const std::string &name, pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);

/**
 * @brief 保存点云(欧拉角版本)
 * 
 * @param name 名称
 * @param cloud 点云
 * @param quaternions 四元数
 */
void SavePointNormals(const std::string &name, pcl::PointCloud<PointT>::Ptr cloud, std::vector<Eigen::Quaternionf> quaternions);

/**
 * @brief 保存点云(四元数版本)
 * 
 * @param name 名称
 * @param cloud 点云
 * @param normals 法线
 * @param obb 最小包围盒
 */
void SavePointNormals2(const std::string &name, pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);

/**
 * @brief 保存点云(四元数版本)
 * 
 * @param name 名称
 * @param cloud 点云
 * @param normals 法线
 */
void SavePointNormals2(const std::string &name, pcl::PointCloud<PointT>::Ptr cloud, std::vector<Eigen::Quaternionf> quaternions);

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
 * @brief 点云投影排序
 * 
 * @param cloud 点云
 * @param normals 法线
 * @param obb 最小包围盒
 * @param axis 主轴
 * @return std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr>  点云, 法线
 */
std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> Sort(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, const OBB &obb, const std::string &axis);

/**
 * @brief 变换点云与法线
 * 
 * @param cloud 点云
 * @param normals 法线
 * @param transformMatrix 变换矩阵
 * @return std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> 点云, 法线
 */
std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> TransformPointCloud(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, const Eigen::Matrix4f & transformMatrix);

#endif // __MISC_H__
