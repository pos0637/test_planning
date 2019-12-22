#include <algorithm>
#include <Eigen/Geometry>
#include <opencv2/calib3d/calib3d.hpp>
#include "eulerAngle.h"

/**
 * @brief 获取四元数
 * 
 * @param transformMatrix 变换矩阵
 * @return Eigen::Quaternionf 四元数
 */
Eigen::Quaternionf GetQuaternion(const Eigen::Matrix4f &transformMatrix)
{
    Eigen::Matrix3f rotationMatrix;
    rotationMatrix << transformMatrix(0, 0), transformMatrix(0, 1), transformMatrix(0, 2),
        transformMatrix(1, 0), transformMatrix(1, 1), transformMatrix(1, 2),
        transformMatrix(2, 0), transformMatrix(2, 1), transformMatrix(2, 2);

    return Eigen::Quaternionf(rotationMatrix);
}

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
    Eigen::Vector3f angles = Eigen::Quaternionf::FromTwoVectors(v1, v2).toRotationMatrix().eulerAngles(2, 1, 0);
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
 * @brief 计算固定XY轴方向的欧拉角
 * 
 * @param quaternion 四元数
 * @return Eigen::Vector3f 欧拉角
 */
Eigen::Vector3f ComputeFixedEulerAngle(const Eigen::Quaternionf &quaternion)
{
    Eigen::Vector3f angles = quaternion.toRotationMatrix().eulerAngles(2, 1, 0);
    angles[0] = angles[0] * 180 / CV_PI;
    angles[1] = angles[1] * 180 / CV_PI;
    angles[2] = angles[2] * 180 / CV_PI;

    return Eigen::Vector3f(angles[0], angles[1], angles[2]);
}

/**
 * @brief 计算固定XY轴方向的欧拉角
 * 
 * @param normal 法线
 * @return Eigen::Vector3f 欧拉角
 */
Eigen::Vector3f ComputeFixedEulerAngle(const pcl::Normal &normal)
{
    return ComputeFixedEulerAngle(ComputeFixedQuaternion(normal));
}

/**
 * @brief 计算固定XY轴方向的四元数
 * 
 * @param normal 法线
 * @return Eigen::Quaternionf 四元数
 */
Eigen::Quaternionf ComputeFixedQuaternion(const pcl::Normal &normal)
{
    Eigen::Vector2f vy2(0.0f, 0.0f);
    Eigen::Vector2f v(normal.normal_x, normal.normal_y);

    // 计算固定XY轴方向的垂直法线
    v = v.normalized();
    if ((std::fabs(v[0]) > FLT_EPSILON) && (std::fabs(v[1]) > FLT_EPSILON))
    {
        vy2 = Eigen::Vector2f(-v[1] / v[0], 1.0f).normalized();
    }
    else if ((std::fabs(v[0]) < FLT_EPSILON) && (std::fabs(v[1]) > FLT_EPSILON))
    {
        vy2 = Eigen::Vector2f(1.0f, -v[0] / v[1]).normalized();
    }
    else if ((std::fabs(v[0]) > FLT_EPSILON) && (std::fabs(v[1]) < FLT_EPSILON))
    {
        vy2 = Eigen::Vector2f(-v[1] / v[0], 1.0f).normalized();
    }

    // 修正法线方向与坐标系平行且同向
    if ((v[0] > FLT_EPSILON) && (v[1] > FLT_EPSILON))
    {
        if (vy2[0] < FLT_EPSILON)
        {
            vy2 *= -1;
        }
    }
    else if ((v[0] < FLT_EPSILON) && (v[1] > FLT_EPSILON))
    {
        if (vy2[0] < FLT_EPSILON)
        {
            vy2 *= -1;
        }
    }
    else if ((v[0] < FLT_EPSILON) && (v[1] < FLT_EPSILON))
    {
        if (vy2[0] > FLT_EPSILON)
        {
            vy2 *= -1;
        }
    }
    else if ((v[0] > FLT_EPSILON) && (v[1] < FLT_EPSILON))
    {
        if (vy2[0] > FLT_EPSILON)
        {
            vy2 *= -1;
        }
    }

    Eigen::Vector3f vx(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f vy(vy2[0], vy2[1], 0.0f);
    Eigen::Vector3f vz(normal.normal_x, normal.normal_y, normal.normal_z);

    // 根据工具实际姿态调整轴方向
    vz = vz.normalized() * -1;
    vy = vy.normalized();
    vx = vy.cross(vz).normalized();

    Eigen::Matrix3f rotationMatrix;
    rotationMatrix << vx[0], vy[0], vz[0], vx[1], vy[1], vz[1], vx[2], vy[2], vz[2];

    return Eigen::Quaternionf(rotationMatrix);
}

/**
 * @brief 计算固定XY轴方向的四元数
 * 
 * @param normals 法线
 * @return std::vector<Eigen::Quaternionf> 四元数
 */
std::vector<Eigen::Quaternionf> ComputeFixedQuaternions(pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    std::vector<Eigen::Quaternionf> result;
    for (pcl::Normal normal : *normals)
    {
        result.push_back(ComputeFixedQuaternion(normal));
    }

    return result;
}

/**
 * @brief 变换四元数
 * 
 * @param quaternions 四元数
 * @param transformMatrix 变换矩阵
 * @return std::vector<Eigen::Quaternionf> 四元数
 */
std::vector<Eigen::Quaternionf> TransformQuaternions(const std::vector<Eigen::Quaternionf> &quaternions, const Eigen::Matrix4f &transformMatrix)
{
    std::vector<Eigen::Quaternionf> result;
    Eigen::Quaternionf transformQuat = GetQuaternion(transformMatrix);
    for (Eigen::Quaternionf quat : quaternions)
    {
        result.push_back(transformQuat * quat);
    }

    return result;
}
