#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include "common.h"
#include "edge.h"
#include "edge2d.h"
#include "filter.h"
#include "miscs.h"
#include "obb.h"
#include "roi.h"
#include "zoom.h"
#include "python_bridge.h"

void test()
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT>("shoes2.pcd", *cloud) < 0)
    {
        return;
    }

    // TODO: 进行XYZ滤波
    pcl::PointCloud<PointT>::Ptr inliers = StatisticalFilter(cloud);
    inliers = Downsampling(inliers, 1.0f);
    OBB inliersOBB = ComputeOBB(inliers);

    // 对齐
    pcl::PointCloud<PointT>::Ptr rotated(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*inliers, *rotated, inliersOBB.transformMatrix);
    pcl::PointCloud<pcl::Normal>::Ptr normals = ComputeNormals(rotated);
    OBB rotatedOBB = ComputeOBB(rotated);

    // 边缘检测
    pcl::PointCloud<PointT>::Ptr roi = ROI(rotated, pcl::PointXYZ(rotatedOBB.min.x, rotatedOBB.min.y, rotatedOBB.min.z + 1.0f), rotatedOBB.max);
    pcl::PointCloud<PointT>::Ptr edge = EdgeDetect2D(roi, rotatedOBB);

    // 匹配候选点云
    std::tie(edge, normals) = MatchCandidateCloud(rotated, normals, edge);

    // 点云排序
    std::tie(edge, normals) = Sort(edge, normals, rotatedOBB, "z");

    // 收缩点云
    pcl::PointCloud<PointT>::Ptr edge2;
    pcl::PointCloud<pcl::Normal>::Ptr normals2;
    // std::tie(edge2, normals2) = Shrink(edge, normals, 10.0);
    std::tie(edge2, normals2) = Shrink2D(edge, rotatedOBB, 10.0, 10.0);

    // 点云排序
    std::tie(edge2, normals2) = Sort(edge2, normals2, rotatedOBB, "z");

    // 绘制点云
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("boundary"));
    DrawCoordinateSystemAxis(viewer);

#if 0
    viewer->addPointCloud<PointT>(rotated, "rotated");
    viewer->addPointCloud<PointT>(edge, "edge");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "edge");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "edge");
    viewer->addPointCloud<PointT>(edge2, "edge2");
    viewer->addPointCloudNormals<PointT, pcl::Normal>(edge2, normals2, 1, 20, "normals");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "edge2");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "edge2");
    DrawOBB(viewer, rotatedOBB, "rotated");
#else
    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*edge, *normals, *pointNormals);
    pcl::transformPointCloudWithNormals(*pointNormals, *pointNormals, inliersOBB.inverseTransformMatrix);
    pcl::copyPointCloud(*pointNormals, *normals);
    pcl::transformPointCloud(*edge, *edge, inliersOBB.inverseTransformMatrix);

    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals2(new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*edge2, *normals2, *pointNormals2);
    pcl::transformPointCloudWithNormals(*pointNormals2, *pointNormals2, inliersOBB.inverseTransformMatrix);
    pcl::copyPointCloud(*pointNormals2, *normals2);
    pcl::transformPointCloud(*edge2, *edge2, inliersOBB.inverseTransformMatrix);

    viewer->addPointCloud<PointT>(inliers, "inliers");
    viewer->addPointCloud<PointT>(edge, "edge");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "edge");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "edge");
    viewer->addPointCloud<PointT>(edge2, "edge2");
    viewer->addPointCloudNormals<PointT, pcl::Normal>(edge2, normals2, 1, 20, "normals");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "edge2");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "edge2");
    DrawOBB(viewer, inliersOBB, "inliers");
#endif

    // 保存点云
    SavePointNormals("./output/points.txt", edge, normals2, inliersOBB);
    SavePointNormals2("./output/points2.txt", edge, normals2, inliersOBB);
    // 生成KUKA代码
    Invoke("generate_kuka_code", "execute");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}

int main(int argc, char **argv)
{
    test();
}
