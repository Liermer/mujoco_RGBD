// Copyright 2023 Nimrod Curtis
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Custom includes
#include "Plane.hpp"
#include "Utilities.hpp"

// Compute the convex hull of the projected plane
void Plane::computePlaneHull()
{ 
  // Project the plane to 2D
  projectPlaneTo2D();

  // Initialize the ConvexHull object
  pcl::ConvexHull<pcl::PointXYZ> hull;
  hull.setInputCloud(cloud_projected_);

  // Reconstruct the convex hull for the projected plane
  hull.reconstruct(*hull_);
}

// Project the plane to 2D using the plane coefficients
void Plane::projectPlaneTo2D()
{
  // Initialize the ProjectInliers object to project the points onto a model plane
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud_);
  proj.setModelCoefficients(plane_coefficients_);
  
  // Perform the projection of the plane to 2D
  proj.filter(*cloud_projected_);
}

// Calculate the slope of the plane using the first plane coefficient
void Plane::calcPlaneSlope()
{
  // Convert the first plane coefficient to degrees to get the slope
  slope_ = Utilities::rad2deg(plane_coefficients_->values[0]);
}

// Process the plane by setting its features and computing its convex hull
void Plane::processPlane()
{
    getCentroid();
    getPrincipalDirections();
    getMeasurements();
    computePlaneHull();
}

// Compute the centroid of the plane's points
void Plane::getCentroid() 
{
    Eigen::Vector4f vector_centroid;
    pcl::compute3DCentroid(*cloud_,vector_centroid);
    centroid_.x = vector_centroid[0];
    centroid_.y = vector_centroid[1];
    centroid_.z = vector_centroid[2];
}

// 计算平面的主方向，使用协方差矩阵
void Plane::getPrincipalDirections() 
{
    // 声明一个3x3的矩阵用于存储协方差
    Eigen::Matrix3f covariance;
    
    // 创建一个4D向量，存储平面的质心坐标（x,y,z）和一个额外的1（齐次坐标）
    Eigen::Vector4f eigen_centroid(centroid_.x, centroid_.y, centroid_.z, 1);
    
    // 计算点云的归一化协方差矩阵
    pcl::computeCovarianceMatrixNormalized(*cloud_, eigen_centroid, covariance);
    
    // 创建一个自伴随特征求解器，用于求解协方差矩阵的特征向量
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    
    // 获取特征向量，这些向量代表平面的主方向
    plane_dir_ = eigen_solver.eigenvectors();
    
    // 计算第三个主方向，使用前两个主方向的叉积
    // 注意：第0列对应最小特征值，因此其方向垂直于平面
    plane_dir_.col(2) = plane_dir_.col(0).cross(plane_dir_.col(1));
}

// Compute the plane's measurements (length, width, and center)
void Plane::getMeasurements() 
{
    // 如果平面方向向量为零，重新计算主方向
    if (plane_dir_.isZero(0))
        this->getPrincipalDirections();

    // 创建从相机投影坐标系到平面坐标系的变换矩阵
    Eigen::Matrix4f cp2p(Eigen::Matrix4f::Identity()); // cp2p = camera projected ==> plane
    cp2p.block<3,3>(0,0) = plane_dir_.transpose(); // 设置旋转部分为平面方向的转置
    cp2p.block<3,1>(0,3) = -1.f * (cp2p.block<3,3>(0,0) * centroid_.getVector3fMap().head<3>()); // 设置平移部分
    
    // 创建一个新的点云指针，用于存储变换后的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cPoints(new pcl::PointCloud<pcl::PointXYZ>);
    // 将原始点云变换到平面坐标系
    pcl::transformPointCloud(*cloud_, *cPoints, cp2p);

    // 声明用于存储最小和最大点的向量
    Eigen::Vector4f min_pt, max_pt;
    // 计算变换后点云的最小和最大边界点
    pcl::getMinMax3D(*cPoints, min_pt, max_pt);

    // 计算平面的长度（长边）
    length_ = max_pt[2] - min_pt[2]; // 使用第三个分量，因为主方向在第三列有较大的特征值
    // 计算平面的宽度（深度）
    width_ = max_pt[1] - min_pt[1];

    // 计算边界矩形的中心点，并将其变换回世界坐标系
    // 创建从平面坐标系到相机投影坐标系的仿射变换
    Eigen::Affine3d p2cp = Eigen::Translation3d(centroid_.getVector3fMap().cast<double>()) * Eigen::AngleAxisd(plane_dir_.cast<double>());
    // 计算边界框对角线的中点
    Eigen::Vector3f mean_diag = 0.5f*(max_pt.head<3>() + min_pt.head<3>());
    // 将中点从平面坐标系变换回世界坐标系
    pcl::transformPoint(mean_diag, mean_diag, p2cp.cast<float>());
    // 设置平面中心点的坐标
    center_.x = mean_diag(0);
    center_.y = mean_diag(1);
    center_.z = mean_diag(2);
} 