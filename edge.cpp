#include <math.h>
#include <pcl/surface/convex_hull.h>
#include <opencv2/imgproc.hpp>
#include "edge.h"

/**
 * @brief 投影
 * 
 * @param cloud 点云
 * @param obb 最小包围盒
 * @return pcl::PointCloud<PointT>::Ptr 投影点云
 */
pcl::PointCloud<PointT>::Ptr Projection(pcl::PointCloud<PointT>::Ptr cloud, const OBB &obb)
{
    int width = std::fabs(obb.max.x - obb.min.x);
    int height = std::fabs(obb.max.y - obb.min.y);
    PointT min = obb.min;
    cv::Mat image = cv::Mat::zeros(height, width, CV_32FC1);
    pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>());

    for (PointT point : *cloud)
    {
        int x = point.x - min.x;
        int y = point.y - min.y;
        int z = point.z - min.z;
        if (image.at<float>(y, x) <= z)
        {
            image.at<float>(y, x) = z;
        }
    }

    for (int y = 0; y < image.rows; y++)
    {
        for (int x = 0; x < image.cols; x++)
        {
            float z = image.at<float>(y, x);
            if (z > 0)
            {
                result->push_back(PointT(x + min.x, y + min.y, z + min.z));
            }
        }
    }

    return result;
}

/**
 * @brief 边缘查找
 * 
 * @param cloud 点云
 * @param obb 最小包围盒
 * @return pcl::PointCloud<PointT>::Ptr 边缘点云
 */
pcl::PointCloud<PointT>::Ptr EdgeDetect(pcl::PointCloud<PointT>::Ptr cloud, const OBB &obb)
{
    pcl::ConvexHull<PointT> hull;
    hull.setInputCloud(Projection(cloud, obb));
    hull.setDimension(2);

    std::vector<pcl::Vertices> polygons;
    pcl::PointCloud<PointT>::Ptr surfaceHull(new pcl::PointCloud<PointT>);
    hull.reconstruct(*surfaceHull, polygons);

    return surfaceHull;
}
