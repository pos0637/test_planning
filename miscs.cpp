#include <iostream>
#include <string>
#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "eulerAngle.h"
#include "miscs.h"

/**
 * @brief 绘制坐标系
 *
 * @param viewer 视图
 */
void DrawCoordinateSystemAxis(const pcl::visualization::PCLVisualizer::Ptr viewer)
{
    viewer->addArrow(pcl::PointXYZ(100.0, 0.0, 0.0), pcl::PointXYZ(0.0, 0.0, 0.0), 1.0f, 0.0f, 0.0f, false, "X");
    viewer->addArrow(pcl::PointXYZ(0.0, 100.0, 0.0), pcl::PointXYZ(0.0, 0.0, 0.0), 0.0f, 1.0f, 0.0f, false, "Y");
    viewer->addArrow(pcl::PointXYZ(0.0, 0.0, 100.0), pcl::PointXYZ(0.0, 0.0, 0.0), 0.0f, 0.0f, 1.0f, false, "Z");
}

/**
 * @brief 显示图像
 *
 * @param name 名称
 * @param image 图像
 */
void ShowImage(const std::string &name, const cv::Mat &image)
{
    cv::namedWindow(name, CV_WINDOW_NORMAL);
    cv::imshow(name, image);
    cv::waitKey(0);
}

/**
 * @brief 显示曲面
 * 
 * @param name 名称
 * @param mesh 曲面
 */
void ShowMesh(const std::string &name, pcl::PolygonMesh::Ptr mesh)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(name));
    viewer->addPolygonMesh(*mesh);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}

/**
 * @brief 显示点云
 * 
 * @param name 名称
 * @param cloud 点云
 */
void ShowPointCloud(const std::string &name, pcl::PointCloud<PointT>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(name));
    viewer->addPointCloud<PointT>(cloud, name);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}

/**
 * @brief 绘制轮廓点
 * 
 * @param image 图像
 * @param contours 轮廓
 */
void DrawContours(const cv::Mat &image, const std::vector<std::vector<cv::Point>> &contours)
{
    for (std::vector<cv::Point> contour : contours)
    {
        for (cv::Point point : contour)
        {
            cv::line(image, point, point, cv::Scalar(0, 255, 0));
        }
    }
}

/**
 * @brief 显示轮廓
 * 
 * @param name 名称
 * @param width 宽度
 * @param height 高度
 * @param contours 轮廓
 */
void ShowContours(const std::string &name, int width, int height, const std::vector<std::vector<cv::Point>> &contours)
{
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
    cv::drawContours(image, contours, -1, cv::Scalar(255, 0, 0));
    ShowImage(name, image);
}

/**
 * @brief 保存轮廓索引
 * 
 * @param name 名称
 * @param width 宽度
 * @param height 高度
 * @param contours 轮廓
 */
void SaveContoursId(const std::string &name, int width, int height, const std::vector<std::vector<cv::Point>> &contours)
{
    cv::Mat image = cv::Mat::zeros(height * 30, width * 30, CV_8UC3);
    for (int i = 0; i < contours.size(); ++i)
    {
        for (int j = 0; j < contours[i].size(); ++j)
        {
            cv::Point point = contours[i][j];
            cv::circle(image, cv::Point(point.x * 30, point.y * 30), 1, cv::Scalar(0, 0, 255));
            cv::putText(image, std::to_string(i) + ":" + std::to_string(j), cv::Point(point.x * 30 + 5, point.y * 30 + 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
        }
    }

    cv::imwrite(name, image);
}

/**
 * @brief 读取轮廓
 * 
 * @param name 文件名
 * @return std::vector<std::vector<cv::Point>> 轮廓
 */
std::vector<std::vector<cv::Point>> LoadContours(const std::string &name)
{
    std::ifstream file(name, std::ios::in);
    std::vector<std::vector<cv::Point>> result(1);

    std::string temp;
    while (std::getline(file, temp))
    {
        std::vector<std::string> array;
        boost::split(array, temp, boost::is_any_of(","));
        result[0].push_back(cv::Point(std::stoi(array[0]), std::stoi(array[1])));
    }

    file.close();

    return result;
}

/**
 * @brief 保存轮廓
 * 
 * @param name 文件名
 * @param contours 轮廓
 */
void SaveContours(const std::string &name, const std::vector<std::vector<cv::Point>> &contours)
{
    std::ofstream file(name, std::ios::out);
    if (!file)
    {
        return;
    }

    for (std::vector<cv::Point> contour : contours)
    {
        for (cv::Point point : contour)
        {
            file << point.x << "," << point.y << endl;
        }
    }

    file.close();
}

/**
 * @brief 保存图像
 * 
 * @param name 文件名
 * @param image 图像
 */
void SaveImage(const std::string &name, const cv::Mat &image)
{
    cv::FileStorage fs(name, cv::FileStorage::WRITE);
    fs << "image" << image;
    fs.release();
}

/**
 * @brief 保存图像值
 * 
 * @param name 名称
 * @param image 图像
 */
void SaveImageValue(const std::string &name, const cv::Mat &image)
{
    cv::Mat result = cv::Mat::zeros(image.rows * 30, image.cols * 30, CV_8UC3);
    char buffer[20];
    for (int y = 0; y < image.rows; ++y)
    {
        for (int x = 0; x < image.cols; ++x)
        {
            float value = image.at<float>(y, x);
            if (std::fabs(value) < FLT_EPSILON)
            {
                continue;
            }

            sprintf(buffer, "%.2f", value);
            cv::putText(result, buffer, cv::Point(x * 30 + 5, y * 30 + 5), cv::FONT_HERSHEY_PLAIN, 0.5, cv::Scalar(255, 255, 255));
        }
    }

    cv::imwrite(name, result);
}

/**
 * @brief 保存点云(欧拉角版本)
 * 
 * @param name 名称
 * @param cloud 点云
 * @param normals 法线
 * @param obb 最小包围盒
 */
void SavePointNormals(const std::string &name, pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, const OBB &obb)
{
    std::ofstream file(name, std::ios::out);
    if (!file)
    {
        return;
    }

    char buffer[256];
    int s = 2;
    int t = 34;
    for (int i = 0; i < cloud->points.size(); ++i)
    {
        PointT point = cloud->points[i];
        Eigen::Vector3f angle = ComputeEulerAngle(obb.minorVector, normals->points[i]);
        sprintf(buffer, "%f, %f, %f, %f, %f, %f, %d, %d", point.x, point.y, point.z, angle[0], angle[1], angle[2], s, t);
        file << buffer << endl;
    }

    file.close();
}

/**
 * @brief 保存点云(四元数版本)
 * 
 * @param name 名称
 * @param cloud 点云
 * @param normals 法线
 * @param obb 最小包围盒
 */
void SavePointNormals2(const std::string &name, pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, const OBB &obb)
{
    std::ofstream file(name, std::ios::out);
    if (!file)
    {
        return;
    }

    char buffer[256];
    for (int i = 0; i < cloud->points.size(); ++i)
    {
        PointT point = cloud->points[i];
        Eigen::Quaternionf quat = ComputeQuaternion(obb.minorVector, normals->points[i]);
        sprintf(buffer, "%f, %f, %f, %f, %f, %f, %f", point.x, point.y, point.z, quat.x(), quat.y(), quat.z(), quat.w());
        file << buffer << endl;
    }

    file.close();
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
        if (normals != NULL)
        {
            n->push_back(normals->points[info[i].id]);
        }
    }

    return std::make_tuple(c, n);
}
