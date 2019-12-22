#ifndef __EULER_ANGLE_H__
#define __EULER_ANGLE_H__

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/**
 * @brief 获取四元数
 * 
 * @param transformMatrix 变换矩阵
 * @return Eigen::Quaternionf 四元数
 */
Eigen::Quaternionf GetQuaternion(const Eigen::Matrix4f &transformMatrix);

/**
 * @brief 计算欧拉角
 * 
 * @param z Z轴向量
 * @param normal 法线
 * @return Eigen::Vector3f 欧拉角
 */
Eigen::Vector3f ComputeEulerAngle(const Eigen::Vector3f &z, const pcl::Normal &normal);

/**
 * @brief 计算四元数
 * 
 * @param z Z轴向量
 * @param normal 法线
 * @return Eigen::Quaternionf 四元数
 */
Eigen::Quaternionf ComputeQuaternion(const Eigen::Vector3f &z, const pcl::Normal &normal);

/**
 * @brief 计算固定XY轴方向的欧拉角
 * 
 * @param quaternion 四元数
 * @return Eigen::Vector3f 欧拉角
 */
Eigen::Vector3f ComputeFixedEulerAngle(const Eigen::Quaternionf &quaternion);

/**
 * @brief 计算固定XY轴方向的欧拉角
 * 
 * @param normal 法线
 * @return Eigen::Vector3f 欧拉角
 */
Eigen::Vector3f ComputeFixedEulerAngle(const pcl::Normal &normal);

/**
 * @brief 计算固定XY轴方向的四元数
 * 
 * @param normal 法线
 * @return Eigen::Quaternionf 四元数
 */
Eigen::Quaternionf ComputeFixedQuaternion(const pcl::Normal &normal);

/**
 * @brief 计算固定XY轴方向的四元数
 * 
 * @param normals 法线
 * @return std::vector<Eigen::Quaternionf> 四元数
 */
std::vector<Eigen::Quaternionf> ComputeFixedQuaternions(pcl::PointCloud<pcl::Normal>::Ptr normals);

/**
 * @brief 变换四元数
 * 
 * @param quaternions 四元数
 * @param transformMatrix 变换矩阵
 * @return std::vector<Eigen::Quaternionf> 四元数
 */
std::vector<Eigen::Quaternionf> TransformQuaternions(const std::vector<Eigen::Quaternionf> &quaternions, const Eigen::Matrix4f &transformMatrix);

#endif // __EULER_ANGLE_H__
