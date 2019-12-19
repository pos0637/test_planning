#include "zoom.h"
#include <Eigen/Eigen>
#include <pcl/common/geometry.h>

/**
 * @brief 沿法线方向收缩点云
 * 
 * @param cloud 点云
 * @param normals 法线
 * @param distance 距离
 * @return std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr>  点云, 法线
 */
std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> Shrink(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, float distance)
{
    pcl::PointCloud<PointT>::Ptr c(new pcl::PointCloud<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr n(new pcl::PointCloud<pcl::Normal>());
    for (int i = 0; i < cloud->points.size(); ++i)
    {
        PointT point = cloud->points[i];
        pcl::Normal normal = normals->points[i];
        Eigen::Vector3f v = Eigen::Vector3f(normal.normal_x, normal.normal_y, normal.normal_z) * distance;
        c->push_back(PointT(point.x + v[0], point.y + v[1], point.z + v[2]));
        n->push_back(normal);
    }

    return std::make_tuple(c, n);
}

/**
 * @brief 多边形收缩点云
 * 
 * @param cloud 点云
 * @param obb 最小包围盒
 * @param distance 距离
 * @param offsetZ Z轴偏移
 * @return std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr>  点云, 法线
 */
std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> Shrink2D(pcl::PointCloud<PointT>::Ptr cloud, const OBB &obb, float distance, float offsetZ)
{
    // 计算投影XY平面后中心点坐标
    PointT center = obb.center;
    center.z = 0;

    pcl::PointCloud<PointT>::Ptr c(new pcl::PointCloud<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr n(new pcl::PointCloud<pcl::Normal>());
    for (int i = 0; i < cloud->points.size(); ++i) {
        // 获取前后两点
        PointT p = cloud->points[i];
        PointT p1 = cloud->points[i == 0? cloud->points.size() - 1: i - 1];
        PointT p2 = cloud->points[i == cloud->points.size() - 1? 0: i + 1];

        // 计算向量方向
        Eigen::Vector2f v1(p1.x - p.x, p1.y - p.y);
        Eigen::Vector2f v2(p2.x - p.x, p2.y - p.y);
        Eigen::Vector2f v = (v1.normalized() + v2.normalized()).normalized() * distance;

        // 计算向量内外方向
        PointT t1(p.x, p.y, 0);
        PointT t2(p.x + v[0], p.y + v[1], 0);
        float distance1 = pcl::geometry::squaredDistance(t1, center);
        float distance2 = pcl::geometry::squaredDistance(t2, center);
        if (distance1 > distance2) {
            c->push_back(PointT(p.x + v[0], p.y + v[1], p.z + offsetZ));
            Eigen::Vector3f normal = Eigen::Vector3f(v[0], v[1], offsetZ).normalized();
            n->push_back(pcl::Normal(normal[0], normal[1], normal[2]));
        }
        else {
            c->push_back(PointT(p.x - v[0], p.y - v[1], p.z + offsetZ));
            Eigen::Vector3f normal = Eigen::Vector3f(-v[0], -v[1], offsetZ).normalized();
            n->push_back(pcl::Normal(normal[0], normal[1], normal[2]));
        }
    }

    return std::make_tuple(c, n);
}
