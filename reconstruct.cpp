#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include "reconstruct.h"
#include "filter.h"

/**
 * @brief 曲面重建
 * 
 * @param cloud 点云
 * @param radius 邻近点半径
 * @return pcl::PolygonMesh::Ptr 曲面
 */
pcl::PolygonMesh::Ptr Reconstruct(pcl::PointCloud<PointT>::Ptr cloud, double radius)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals = ComputeNormals(cloud, radius);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
    tree->setInputCloud(cloud_with_normals);

#if 0
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setInputCloud(cloud_with_normals);
    poisson.setSearchMethod(tree);
    poisson.setConfidence(false);
    poisson.setManifold(false);
    poisson.setOutputPolygons(false);

    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh());
    poisson.reconstruct(*mesh);
#endif

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    gp3.setSearchRadius(radius);
    gp3.setMu(5.0f);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumAngle(2 * M_PI / 3);
    gp3.setMinimumAngle(M_PI / 18);
    gp3.setMaximumSurfaceAngle(M_PI / 4);
    gp3.setNormalConsistency(false);
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree);

    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh());
    gp3.reconstruct(*mesh);

    return mesh;
}