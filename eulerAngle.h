#ifndef __EULER_ANGLE_H__
#define __EULER_ANGLE_H__

#include <Eigen/Dense>
#include <pcl/point_types.h>

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
 * @brief 计算固定XY轴方向的四元数
 * 
 * @param normal 法线
 * @return Eigen::Quaternionf 四元数
 */
Eigen::Quaternionf ComputeFixedQuaternion(const pcl::Normal &normal);

#endif // __EULER_ANGLE_H__
