#include <pcl/common/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include "obb.h"

/**
 * @brief 计算变换矩阵
 * 
 * @param rotationMatrix 旋转矩阵
 * @return Eigen::Matrix4f 变换矩阵
 */
Eigen::Matrix4f GenerateTransformMatrix(const Eigen::Matrix3f &rotationMatrix)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 0) = rotationMatrix(0, 0);
    transform(0, 1) = rotationMatrix(0, 1);
    transform(0, 2) = rotationMatrix(0, 2);
    transform(1, 0) = rotationMatrix(1, 0);
    transform(1, 1) = rotationMatrix(1, 1);
    transform(1, 2) = rotationMatrix(1, 2);
    transform(2, 0) = rotationMatrix(2, 0);
    transform(2, 1) = rotationMatrix(2, 1);
    transform(2, 2) = rotationMatrix(2, 2);

    return transform;
}

/**
 * @brief 计算变换矩阵
 * 
 * @param center 中心点
 * @return Eigen::Matrix4f 变换矩阵
 */
Eigen::Matrix4f GenerateTransformMatrix(const pcl::PointXYZ &center)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 3) = -center.x;
    transform(1, 3) = -center.y;
    transform(2, 3) = -center.z;

    return transform;
}

/**
 * @brief 计算变换矩阵
 * 
 * @param rotationMatrix 旋转矩阵
 * @param center 中心点
 * @return Eigen::Matrix4f 变换矩阵
 */
Eigen::Matrix4f GenerateTransformMatrix(const Eigen::Matrix3f &rotationMatrix, const pcl::PointXYZ &center)
{
    Eigen::Matrix3f rt = rotationMatrix.transpose();
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 0) = rt(0, 0);
    transform(0, 1) = rt(0, 1);
    transform(0, 2) = rt(0, 2);
    transform(1, 0) = rt(1, 0);
    transform(1, 1) = rt(1, 1);
    transform(1, 2) = rt(1, 2);
    transform(2, 0) = rt(2, 0);
    transform(2, 1) = rt(2, 1);
    transform(2, 2) = rt(2, 2);

    Eigen::Vector3f translaction = -1 * rt * Eigen::Vector3f(center.x, center.y, center.z);
    transform(0, 3) = translaction(0);
    transform(1, 3) = translaction(1);
    transform(2, 3) = translaction(2);

    return transform;
}

/**
 * @brief 计算逆变换矩阵
 * 
 * @param rotationMatrix 旋转矩阵
 * @param center 中心点
 * @return Eigen::Matrix4f 逆变换矩阵
 */
Eigen::Matrix4f GenerateInverseTransformMatrix(const Eigen::Matrix3f &rotationMatrix, const pcl::PointXYZ &center)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 0) = rotationMatrix(0, 0);
    transform(0, 1) = rotationMatrix(0, 1);
    transform(0, 2) = rotationMatrix(0, 2);
    transform(1, 0) = rotationMatrix(1, 0);
    transform(1, 1) = rotationMatrix(1, 1);
    transform(1, 2) = rotationMatrix(1, 2);
    transform(2, 0) = rotationMatrix(2, 0);
    transform(2, 1) = rotationMatrix(2, 1);
    transform(2, 2) = rotationMatrix(2, 2);

    Eigen::Vector3f translaction = Eigen::Vector3f(center.x, center.y, center.z);
    transform(0, 3) = translaction(0);
    transform(1, 3) = translaction(1);
    transform(2, 3) = translaction(2);

    return transform;
}

/**
 * @brief 计算最小包围盒
 * 
 * @param cloud 点云
 * @return OBB 最小包围盒
 */
OBB ComputeOBB(const pcl::PointCloud<PointT>::Ptr cloud)
{
    pcl::MomentOfInertiaEstimation<PointT> est;
    est.setInputCloud(cloud);
    est.compute();

    // 计算最小包围盒
    OBB obb;
    PointT min, max;
    est.getOBB(min, max, obb.position, obb.rotationMatrix);
    est.getEigenVectors(obb.majorVector, obb.middleVector, obb.minorVector);
    obb.min = pcl::PointXYZ(obb.position.x + min.x, obb.position.y + min.y, obb.position.z + min.z);
    obb.max = pcl::PointXYZ(obb.position.x + max.x, obb.position.y + max.y, obb.position.z + max.z);

    // 计算质心
    Eigen::Vector3f massCenter;
    est.getMassCenter(massCenter);
    obb.center = pcl::PointXYZ(massCenter(0), massCenter(1), massCenter(2));

    // 计算变换矩阵
    obb.transformMatrix = GenerateTransformMatrix(obb.rotationMatrix, obb.center);
    obb.inverseTransformMatrix = GenerateInverseTransformMatrix(obb.rotationMatrix, obb.center);

    return obb;
}

/**
 * @brief 点云对齐到坐标系
 * 
 * @param cloud 点云
 * @param obb 最小包围盒
 * @return pcl::PointCloud<PointT>::Ptr 对齐后点云
 */
pcl::PointCloud<PointT>::Ptr AlignToCenter(const pcl::PointCloud<PointT>::Ptr cloud, const OBB &obb)
{
    pcl::PointXYZ center(-obb.center.x, -obb.center.y, -obb.center.z);
    pcl::PointCloud<PointT>::Ptr transform(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*cloud, *transform, GenerateTransformMatrix(center));
    return transform;
}

/**
 * @brief 绘制最小包围盒坐标系
 * 
 * @param viewer 视图
 * @param obb 最小包围盒
 * @param id 名称
 */
void DrawOBBAxis(const pcl::visualization::PCLVisualizer::Ptr viewer, const OBB &obb, const std::string &id)
{
    float length = std::max(std::max(obb.max.x - obb.min.x, obb.max.y - obb.min.y), obb.max.z - obb.min.z) / 2.0f;
    Eigen::Vector3f majorVector = obb.majorVector * length;
    Eigen::Vector3f middleVector = obb.middleVector * length;
    Eigen::Vector3f minorVector = obb.minorVector * length;
    pcl::PointXYZ center = obb.center;

    pcl::PointXYZ x(majorVector(0) + center.x, majorVector(1) + center.y, majorVector(2) + center.z);
    pcl::PointXYZ y(middleVector(0) + center.x, middleVector(1) + center.y, middleVector(2) + center.z);
    pcl::PointXYZ z(minorVector(0) + center.x, minorVector(1) + center.y, minorVector(2) + center.z);
    viewer->addArrow(x, center, 1.0f, 0.0f, 0.0f, false, id + "X");
    viewer->addArrow(y, center, 0.0f, 1.0f, 0.0f, false, id + "Y");
    viewer->addArrow(z, center, 0.0f, 0.0f, 1.0f, false, id + "Z");
}

/**
 * @brief 绘制最小包围盒
 * 
 * @param viewer 视图
 * @param obb 最小包围盒
 * @param id 名称
 */
void DrawOBB(const pcl::visualization::PCLVisualizer::Ptr viewer, const OBB &obb, const std::string &id)
{
    Eigen::Vector3f pos(obb.position.x, obb.position.y, obb.position.z);
    Eigen::Quaternionf quat(obb.rotationMatrix);
    viewer->addCube(pos, quat, obb.max.x - obb.min.x, obb.max.y - obb.min.y, obb.max.z - obb.min.z, id + "OBB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id + "OBB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, id + "OBB");
    DrawOBBAxis(viewer, obb, id);

    Eigen::Quaternionf quatI(Eigen::Matrix3f::Identity());
    Eigen::Vector3f min(obb.min.x, obb.min.y, obb.min.z);
    Eigen::Vector3f max(obb.max.x, obb.max.y, obb.max.z);
    viewer->addCube(min, quatI, 5, 5, 5, id + "OBB_MIN");
    viewer->addCube(max, quatI, 5, 5, 5, id + "OBB_MAX");
}
