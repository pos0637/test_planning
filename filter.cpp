#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include "filter.h"

/**
 * @brief 统计滤波
 * 
 * @param cloud 点云
 * @param k 邻近点数量
 * @param threshold 阈值
 * @return pcl::PointCloud<PointT>::Ptr 点云
 */
pcl::PointCloud<PointT>::Ptr StatisticalFilter(pcl::PointCloud<PointT>::Ptr cloud, int k, float threshold)
{
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(k);
    sor.setStddevMulThresh(threshold);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    sor.filter(*filtered);

    return filtered;
}

/**
 * @brief 下采样
 * 
 * @param cloud 点云
 * @param size 采样体积
 * @return pcl::PointCloud<PointT>::Ptr 点云
 */
pcl::PointCloud<PointT>::Ptr Downsampling(pcl::PointCloud<PointT>::Ptr cloud, float size)
{
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(size, size, size);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    vg.filter(*filtered);

    return filtered;
}

/**
 * @brief 计算法线
 * 
 * @param cloud 点云
 * @param radius 邻近点半径
 * @param inverse 是否反转
 * @return pcl::PointCloud<pcl::Normal>::Ptr 法线
 */
pcl::PointCloud<pcl::Normal>::Ptr ComputeNormals(pcl::PointCloud<PointT>::Ptr cloud, double radius, bool inverse)
{
    pcl::NormalEstimation<PointT, pcl::Normal> net;
    net.setInputCloud(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    net.setSearchMethod(tree);
    net.setRadiusSearch(radius);

    // 计算点云质心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    net.setViewPoint(centroid[0], centroid[1], centroid[2]);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    net.compute(*normals);

    if (inverse)
    {
        for (auto normal : *normals)
        {
            normal.normal_x *= -1;
            normal.normal_y *= -1;
            normal.normal_z *= -1;
        }
    }

    return normals;
}

/**
 * @brief 计算边界
 * 
 * @param cloud 点云
 * @param radius 邻近点半径
 * @return pcl::PointCloud<PointT>::Ptr 边界点云
 */
pcl::PointCloud<PointT>::Ptr ComputreBoundaries(pcl::PointCloud<PointT>::Ptr cloud, double radius)
{
    pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary> est;
    est.setInputCloud(cloud);
    est.setInputNormals(ComputeNormals(cloud));
    est.setRadiusSearch(radius);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    est.setSearchMethod(tree);

    pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>);
    est.compute(*boundaries);

    pcl::PointCloud<PointT>::Ptr boundPoints(new pcl::PointCloud<PointT>);
    for (int i = 0; i < cloud->size(); i++)
    {
        int x = static_cast<int>(boundaries->points[i].boundary_point);
        if (x == 1)
        {
            boundPoints->push_back(cloud->points[i]);
        }
    }

    return boundPoints;
}

/**
 * @brief 求平均值
 * 
 * @param cloud 点云
 * @param axis 轴
 * @return float 平均值
 */
float Mean(pcl::PointCloud<PointT>::Ptr cloud, const std::string &axis)
{
    if (cloud->size() == 0)
    {
        return 0;
    }

    double total = 0.0f;
    if (axis == "x")
    {
        for (PointT point : *cloud)
        {
            total += point.x;
        }
    }
    else if (axis == "y")
    {
        for (PointT point : *cloud)
        {
            total += point.y;
        }
    }
    else if (axis == "z")
    {
        for (PointT point : *cloud)
        {
            total += point.z;
        }
    }
    else
    {
        return 0;
    }

    return total / cloud->size();
}
