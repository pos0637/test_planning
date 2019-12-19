#include <algorithm>
#include <pcl/filters/passthrough.h>
#include "roi.h"

/**
 * @brief 提取感兴趣区域点云
 * 
 * @param cloud 点云
 * @param min 感兴趣区域最小点
 * @param max 感兴趣区域最大点
 * @return pcl::PointCloud<PointT>::Ptr 点云
 */
pcl::PointCloud<PointT>::Ptr ROI(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointXYZ min, pcl::PointXYZ max)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<PointT>::Ptr cloud_x_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_y_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_z_filtered(new pcl::PointCloud<PointT>);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(std::min(min.x, max.x), std::max(min.x, max.x));
    pass.filter(*cloud_x_filtered);

    pass.setInputCloud(cloud_x_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(std::min(min.y, max.y), std::max(min.y, max.y));
    pass.filter(*cloud_y_filtered);

    pass.setInputCloud(cloud_y_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(std::min(min.z, max.z), std::max(min.z, max.z));
    pass.filter(*cloud_z_filtered);

    return cloud_z_filtered;
}
