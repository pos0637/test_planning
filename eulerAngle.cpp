#include <algorithm>
#include <Eigen/Geometry>
#include <opencv2/calib3d/calib3d.hpp>
#include "eulerAngle.h"

/**
 * @brief 计算向量夹角
 * 
 * @param v1 向量
 * @param v2 向量
 * @return float 夹角
 */
float GetTheta(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2)
{
    float v = v1.dot(v2) / (v1.norm() * v2.norm());
    return std::acos(v > 1 ? 1 : v < -1 ? -1 : v);
}

/**
 * @brief 计算旋转矩阵
 * 
 * @param v1 向量
 * @param v2 向量
 * @return cv::Mat 旋转矩阵
 */
cv::Mat GetRotationMatrixFromVectors(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2)
{
    float norm = GetTheta(v1, v2);
    Eigen::Vector3f v = v1.cross(v2).normalized() * norm;

    cv::Mat vector = cv::Mat::zeros(3, 1, CV_32FC1);
    vector.at<float>(0, 0) = v[0];
    vector.at<float>(1, 0) = v[1];
    vector.at<float>(2, 0) = v[2];

    cv::Mat rotationMatrix;
    cv::Rodrigues(vector, rotationMatrix);

    return rotationMatrix;
}

/**
 * @brief 计算欧拉角
 *
 * @param R 旋转矩阵
 * @return Eigen::Vector3f 欧拉角
 */
Eigen::Vector3f ComputeEulerAngle(const cv::Mat &R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());
    if (cv::norm(I, shouldBeIdentity) > FLT_EPSILON)
    {
        return Eigen::Vector3f(0, 0, 0);
    }

    Eigen::Vector3f v(0, 0, 0);
    if (std::fabs(R.at<float>(2, 0) - 1) < FLT_EPSILON)
    {
        v[2] = 0;
        v[1] = -90;
        v[0] = -std::atan2(R.at<float>(0, 1), R.at<float>(1, 1)) * 180 / CV_PI;
    }
    else if (std::fabs(R.at<float>(2, 0) + 1) < FLT_EPSILON)
    {
        v[2] = 0;
        v[1] = 90;
        v[0] = std::atan2(R.at<float>(0, 1), R.at<float>(1, 1)) * 180 / CV_PI;
    }
    else
    {
        float b = std::atan2(-R.at<float>(2, 0), std::sqrt(R.at<float>(0, 0) * R.at<float>(0, 0) + R.at<float>(1, 0) * R.at<float>(1, 0)));
        v[1] = b * 180 / CV_PI;
        v[2] = std::atan2(R.at<float>(1, 0) / std::cos(b), R.at<float>(0, 0) / std::cos(b)) * 180 / CV_PI;
        v[0] = std::atan2(R.at<float>(2, 1) / std::cos(b), R.at<float>(2, 2) / std::cos(b)) * 180 / CV_PI;
    }

    return v;
}

/**
 * @brief 计算欧拉角
 * 
 * @param v1 向量
 * @param v2 向量
 * @return Eigen::Vector3f 欧拉角
 */
Eigen::Vector3f ComputeEulerAngle2(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2)
{
    Eigen::AngleAxisf rotationVector(90 * 180 / CV_PI, Eigen::Vector3f(0, 0, 1).normalized());
    Eigen::Quaternionf quat1(rotationVector);
    Eigen::Quaternionf quat2 = Eigen::Quaternionf::FromTwoVectors(v1, v2);

    // Eigen::Vector3f angles = (quat2 * quat1).toRotationMatrix().eulerAngles(2, 1, 0);
    Eigen::Vector3f angles = quat2.toRotationMatrix().eulerAngles(2, 1, 0);
    angles[0] = angles[0] * 180 / CV_PI;
    angles[1] = angles[1] * 180 / CV_PI;
    angles[2] = angles[2] * 180 / CV_PI;

    return Eigen::Vector3f(angles[0], angles[1], angles[2]);
}

/**
 * @brief 计算欧拉角
 * 
 * @param z Z轴向量
 * @param normal 法线
 * @return Eigen::Vector3f 欧拉角
 */
Eigen::Vector3f ComputeEulerAngle(const Eigen::Vector3f &z, const pcl::Normal &normal)
{
    Eigen::Vector3f v2(normal.normal_x, normal.normal_y, normal.normal_z);
    // 统一向量方向
    if (normal.normal_z > 0)
    {
        v2 *= -1;
    }

    return ComputeEulerAngle2(z, v2);
}

/**
 * @brief 计算四元数
 * 
 * @param z Z轴向量
 * @param normal 法线
 * @return Eigen::Quaternionf 四元数
 */
Eigen::Quaternionf ComputeQuaternion(const Eigen::Vector3f &z, const pcl::Normal &normal)
{
    Eigen::Vector3f v2(normal.normal_x, normal.normal_y, normal.normal_z);
    // 统一向量方向
    if (normal.normal_z > 0)
    {
        v2 *= -1;
    }

    return Eigen::Quaternionf::FromTwoVectors(z, v2);
}

/**
 * @brief 计算固定XY轴方向的四元数
 * 
 * @param normal 法线
 * @return Eigen::Quaternionf 四元数
 */
Eigen::Quaternionf ComputeFixedQuaternion(const pcl::Normal &normal)
{
    Eigen::Vector3f vx(0, 0, 0);
    Eigen::Vector3f vy(0, 0, 0);
    Eigen::Vector3f vz(normal.normal_x, normal.normal_y, normal.normal_z);
    Eigen::Vector3f v(normal.normal_x, normal.normal_y, 0);

    vz = vz.normalized();
    v = v.normalized();
    if ((std::fabs(v[0]) > FLT_EPSILON) && (std::fabs(v[1]) > FLT_EPSILON))
    {
        vy = Eigen::Vector3f(-v[1] / v[0], 1.0f, 0.0f).normalized();
    }
    else if ((std::fabs(v[0]) < FLT_EPSILON) && (std::fabs(v[1]) > FLT_EPSILON))
    {
        vy = Eigen::Vector3f(1.0f, -v[0] / v[1], 0.0f).normalized();
    }
    else if ((std::fabs(v[0]) > FLT_EPSILON) && (std::fabs(v[1]) < FLT_EPSILON))
    {
        vy = Eigen::Vector3f(-v[1] / v[0], 1.0f, 0.0f).normalized();
    }

    vx = vy.cross(vz).normalized();

    Eigen::Matrix3f rotationMatrix;
    rotationMatrix << vx[0], vx[1], vx[2], vy[0], vy[1], vy[2], vz[0], vz[1], vz[2];

    return Eigen::Quaternionf(rotationMatrix);
}

void test1()
{
    Eigen::Vector3f v1(0, 0, 1);
    Eigen::Vector3f v2(-28.53 - 42.50, 207.93 - 189.38, 0);

    Eigen::AngleAxisf rotationVector(90 * 180 / CV_PI, Eigen::Vector3f(0, 0, 1).normalized());
    Eigen::Quaternionf quat1(rotationVector);
    Eigen::Quaternionf quat2 = Eigen::Quaternionf::FromTwoVectors(v1, v2);
    Eigen::Vector3f angles = (quat1 * quat2).toRotationMatrix().eulerAngles(2, 1, 0);
    angles[0] = angles[0] * 180 / CV_PI;
    angles[1] = angles[1] * 180 / CV_PI;
    angles[2] = angles[2] * 180 / CV_PI;

    printf(">>>>>>>>> angles: %f, %f, %f\n", angles(0), angles(1), angles(2));
}
