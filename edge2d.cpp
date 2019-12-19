#include "edge2d.h"
#include <algorithm>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <tuple>
#include "resampling.h"
#include "miscs.h"
#include "python_bridge.h"

/**
 * @brief 投影
 *
 * @param cloud 点云
 * @param obb 最小包围盒
 * @param axis 主轴
 * @param padding 边距
 * @return cv::Mat 深度图
 */
cv::Mat Projection2D(pcl::PointCloud<PointT>::Ptr cloud, const OBB &obb, const std::string &axis, int padding)
{
    int width = ((axis == "x") ? std::fabs(obb.max.x - obb.min.x) : std::fabs(obb.max.y - obb.min.y)) + padding * 2;
    int height = ((axis == "x") ? std::fabs(obb.max.y - obb.min.y) : std::fabs(obb.max.x - obb.min.x)) + padding * 2;
    PointT min = obb.min;
    cv::Mat image = cv::Mat::zeros(height, width, CV_32FC1);

    // 投影
    for (PointT point : *cloud)
    {
        int x = ((axis == "x") ? point.x - min.x : point.y - min.y) + padding;
        int y = ((axis == "x") ? point.y - min.y : point.x - min.x) + padding;
        float z = point.z - min.z;
        if (image.at<float>(y, x) <= z)
        {
            image.at<float>(y, x) = z;
        }
    }

    return image;
}

/**
 * @brief 投影轮廓深度图
 *
 * @param image 深度图
 * @param contours 轮廓
 * @return cv::Mat 轮廓深度图
 */
cv::Mat Projection2D(const cv::Mat &image, const std::vector<std::vector<cv::Point>> &contours)
{
    // 提取边缘点云
    const int radius = 5;
    cv::Mat result = cv::Mat::zeros(image.size(), CV_32FC1);
    // 遍历所有边界
    for (std::vector<cv::Point> contour : contours)
    {
        for (cv::Point point : contour)
        {
            // 计算周边点云极值
            int xStart = std::max(point.x - radius, 0);
            int yStart = std::max(point.y - radius, 0);
            int xEnd = std::min(point.x + radius, image.cols);
            int yEnd = std::min(point.y + radius, image.rows);
            int xMax = -1;
            int yMax = -1;
            float zMax = 0.0f;

            for (int y = yStart; y < yEnd; y++)
            {
                for (int x = xStart; x < xEnd; x++)
                {
                    float z = image.at<float>(y, x);
                    if (z > zMax)
                    {
                        xMax = x;
                        yMax = y;
                        zMax = z;
                    }
                }
            }

            result.at<float>(point.y, point.x) = zMax;
#ifdef _DEBUG
            if (std::fabs(zMax) < FLT_EPSILON)
            {
                printf("error: %d, %d\n", point.x, point.y);
            }
#endif
        }
    }

    return result;
}

/**
 * @brief 投影
 *
 * @param image 深度图
 * @param axis 主轴
 * @param deltaX X补偿值
 * @param deltaY Y补偿值
 * @param deltaZ Z补偿值
 * @return pcl::PointCloud<PointT>::Ptr 点云
 */
pcl::PointCloud<PointT>::Ptr Projection3D(const cv::Mat &image, const std::string &axis, int deltaX, int deltaY, float deltaZ)
{
    pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>());
    for (int y = 0; y < image.rows; y++)
    {
        for (int x = 0; x < image.cols; x++)
        {
            float z = image.at<float>(y, x);
            if (z < FLT_EPSILON)
            {
                continue;
            }

            if (axis == "x")
            {
                result->push_back(PointT(x + deltaX, y + deltaY, z + deltaZ));
            }
            else
            {
                result->push_back(PointT(y + deltaY, x + deltaX, z + deltaZ));
            }
        }
    }

    return result;
}

/**
 * @brief 计算斜率
 *
 * @param image 深度图
 * @return cv::Mat 斜率
 */
cv::Mat ComputeK(const cv::Mat &image)
{
    const int offset = 2;
    cv::Mat k = cv::Mat::zeros(image.size(), CV_32FC1);
    for (int y = 0; y < image.rows; y++)
    {
        for (int x = offset; x < image.cols - offset; x++)
        {
            k.at<float>(y, x) = (image.at<float>(y, x + offset) - image.at<float>(y, x - offset)) / offset;
        }
    }

    return k;
}

/**
 * @brief 多边形逼近轮廓
 *
 * @param contours 轮廓
 * @return std::vector<std::vector<cv::Point>> 多边形轮廓
 */
std::vector<std::vector<cv::Point>> ApproxPoly(const std::vector<std::vector<cv::Point>> &contours)
{
    std::vector<std::vector<cv::Point>> result(1);
    std::vector<cv::Point> points;
    for (std::vector<cv::Point> contour : contours)
    {
        for (cv::Point point : contour)
        {
            points.push_back(point);
        }
    }

#if 1
    cv::approxPolyDP(cv::Mat(points), result[0], 1, true);
#else
    cv::convexHull(points, result[0], true, true);
#endif
    return result;
}

/**
 * @brief 提取边缘
 *
 * @param image 深度图
 * @param threshold 斜率阈值
 * @param width 检查宽度
 * @return cv::Mat 边缘点云
 */
cv::Mat ExtractEdge(const cv::Mat &image, float threshold, int width)
{
    cv::Mat kImage = ComputeK(image);
    cv::Mat edge = image.clone();
    for (int y = 0; y < kImage.rows; y++)
    {
        int first = -1;
        int last = -1;
        int count = 0;

        // 寻找大于阈值的点
        for (int x = 0; x < kImage.cols - 1; x++)
        {
            float k = kImage.at<float>(y, x);
            if (k >= threshold)
            {
                if (first == -1)
                {
                    first = x;
                }
                count++;
            }
            else if (k <= -threshold)
            {
                last = x;
                count++;
            }
        }

        // 检查是否符合条件
        if ((count < 2) || (first < 0) || (last < 0) || (first >= last))
        {
#ifdef _DEBUG
            printf("not found: %d\n", y);
            if (y == 192)
            {
                for (int x = 0; x < kImage.cols - 1; x++)
                {
                    printf("%f, ", kImage.at<float>(y, x));
                }

                printf("\n\n");
            }
#endif // _DEBUG
            continue;
        }

        // 针对线激光相机过滤异常边缘点云
        // 修正边界
        float zMax = image.at<float>(y, first);
        int newFirst = first;
        for (int x = first + 1; x < std::min(first + width, image.cols); x++)
        {
            float z = image.at<float>(y, x);
            if (z > zMax)
            {
                zMax = z;
                newFirst = x;
            }
        }
        first = newFirst;

        // 修正边界
        zMax = image.at<float>(y, last);
        int newLast = last;
        for (int x = last - 1; x >= std::max(last - width, 0); x--)
        {
            float z = image.at<float>(y, x);
            if (z > zMax)
            {
                zMax = z;
                newLast = x;
            }
        }
        last = newLast;

        // 修正深度图
        for (int x = 0; x < first; x++)
        {
            edge.at<float>(y, x) = 0;
        }
        for (int x = last + 1; x < image.cols; x++)
        {
            edge.at<float>(y, x) = 0;
        }
        for (int x = first + 1; x < last; x++)
        {
            edge.at<float>(y, x) = image.at<float>(y, x);
        }
    }

    return edge;
}

/**
 * @brief 查找边缘
 *
 * @param cloud 点云
 * @param obb 最小包围盒
 * @return pcl::PointCloud<PointT>::Ptr 边缘点云
 */
pcl::PointCloud<PointT>::Ptr EdgeDetect2D(pcl::PointCloud<PointT>::Ptr cloud, const OBB &obb)
{
    // 点云投影
    const int padding = 10;
    cv::Mat image = Projection2D(cloud, obb, "y", padding);

    // 中值滤波
    cv::Mat blur;
    cv::medianBlur(image, blur, 5);

    // 边界提取
    cv::Mat edge = ExtractEdge(blur, 3.0f, 10);
    // 高斯滤波
    cv::GaussianBlur(edge, edge, cv::Size(9, 9), 0, 0);

    // 归一化
    cv::Mat normalize = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::normalize(edge, normalize, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    // 形态学运算
    // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    // cv::morphologyEx(normalize, normalize, cv::MORPH_CLOSE, element);

    // 查找边缘
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(normalize, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    SaveImage("contours.yml", image);
    SaveImage("k.yml", ComputeK(image));
    SaveContours("contours.txt", contours);
    Invoke("interpolation", "execute");
    contours = LoadContours("inlier_contours.txt");

#ifndef _DEBUG
    ShowContours("contours", image.cols, image.rows, contours);

#if 0
    // 显示内部轮廓点
    for (int i = 1; i < 15; ++i) {
        cv::Mat output = cv::Mat::zeros(image.size(), CV_8UC3);
        std::vector<std::vector<cv::Point>> innerContours = ComputeInnerContours(contours, i);
        cv::drawContours(output, contours, -1, cv::Scalar(255, 0, 0));
        DrawContours(output, innerContours);
        ShowImage("contours", output);
    }
#endif

#if 0
    // 显示轮廓点顺序
    cv::Mat output = cv::Mat::zeros(image.size(), CV_8UC3);
    for (std::vector<cv::Point> contour : contours) {
        for (cv::Point point : contour) {
            cv::circle(output, point, 1, cv::Scalar(0, 255, 0));
            ShowImage("test", output);
        }
    }
#endif
#endif // _DEBUG

    // 投影轮廓深度图
    image = Projection2D(image, contours);

    return Projection3D(image, "y", obb.min.y - padding, obb.min.x - padding, obb.min.z);
}
