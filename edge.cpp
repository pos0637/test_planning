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

/**
 * @brief 匹配候选点云
 * 
 * @param cloud 点云
 * @param normals 法线
 * @param candidate 候选点云
 * @return std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> 匹配候选点云, 匹配候选法线
 */
std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> MatchCandidateCloud(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<PointT>::Ptr candidate)
{
    pcl::KdTreeFLANN<PointT> tree;
    tree.setInputCloud(cloud);

    std::vector<int> index(1);
    std::vector<float> distance(1);
    pcl::PointCloud<PointT>::Ptr c(new pcl::PointCloud<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr n(new pcl::PointCloud<pcl::Normal>());
    for (PointT point : *candidate)
    {
        if (tree.nearestKSearch(point, 1, index, distance) > 0)
        {
            c->push_back(cloud->points[index[0]]);
            if (normals != NULL)
            {
                n->push_back(normals->points[index[0]]);
            }
        }
    }

    return std::make_tuple(c, n);
}

/**
 * @brief 沿法线方向收缩点云
 * 
 * @param cloud 点云
 * @param normals 法线
 * @param distance 距离
 * @return pcl::PointCloud<PointT>::Ptr 点云
 */
pcl::PointCloud<PointT>::Ptr ShrinkEdge(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, float distance)
{
    pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>());
    for (int i = 0; i < cloud->points.size(); ++i)
    {
        PointT point = cloud->points[i];
        pcl::Normal normal = normals->points[i];
        Eigen::Vector3f v = Eigen::Vector3f(normal.normal_x, normal.normal_y, normal.normal_z) * distance;
        result->push_back(PointT(point.x + v[0], point.y + v[1], point.z + v[2]));
    }

    return result;
}

/**
 * @brief 点云投影排序
 * 
 * @param cloud 点云
 * @param normals 法线
 * @param obb 最小包围盒
 * @param axis 主轴
 * @return std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr>  点云, 法线
 */
std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> Sort(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, const OBB &obb, const std::string &axis)
{
    struct PointInfo
    {
        int id;
        double angle;

        PointInfo(int id, double angle)
        {
            this->id = id;
            this->angle = angle;
        }
    };

    pcl::PointXYZ center = obb.center;
    std::vector<PointInfo> info;

    if (axis == "z")
    {
        for (int i = 0; i < cloud->points.size(); ++i)
        {
            PointT point = cloud->points[i];
            double angle = std::atan2(point.y - center.y, point.x - center.x) * 180.0 / M_PI;
            info.push_back(PointInfo(i, angle));
        }
    }
    else if (axis == "y")
    {
        for (int i = 0; i < cloud->points.size(); ++i)
        {
            PointT point = cloud->points[i];
            double angle = std::atan2(point.x - center.x, point.z - center.z) * 180.0 / M_PI;
            info.push_back(PointInfo(i, angle));
        }
    }
    else if (axis == "x")
    {
        for (int i = 0; i < cloud->points.size(); ++i)
        {
            PointT point = cloud->points[i];
            double angle = std::atan2(point.z - center.z, point.y - center.y) * 180.0 / M_PI;
            info.push_back(PointInfo(i, angle));
        }
    }

    std::sort(info.begin(), info.end(), [](const PointInfo &i1, const PointInfo &i2) {
        return i1.angle > i2.angle;
    });

    pcl::PointCloud<PointT>::Ptr c(new pcl::PointCloud<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr n(new pcl::PointCloud<pcl::Normal>());
    for (int i = 0; i < info.size(); ++i)
    {
        c->push_back(cloud->points[info[i].id]);
        n->push_back(normals->points[info[i].id]);
    }

    return std::make_tuple(c, n);
}
