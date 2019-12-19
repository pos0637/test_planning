#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include "resampling.h"

/**
 * @brief 重采样
 * 
 * @param cloud 点云
 * @param radius 邻近点半径
 * @return pcl::PointCloud<PointT>::Ptr 点云
 */
pcl::PointCloud<PointT>::Ptr Resampling(pcl::PointCloud<PointT>::Ptr cloud, float radius)
{
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::PointCloud<pcl::PointNormal> normals;
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setInputCloud(cloud);
    mls.setComputeNormals(true);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(4);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);
#if 0
    mls.setUpsamplingMethod(mls.VOXEL_GRID_DILATION);
    mls.setDilationVoxelSize(1.0f);
    mls.setDilationIterations(2);
#endif
    mls.process(normals);

    pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>());
    pcl::copyPointCloud(normals, *result);

    return result;
}
