#include "Utilities.hpp"

// 根据内点提取点云的函数
void Utilities::getCloudByInliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
                                const pcl::PointIndices::Ptr &inliers,
                                bool negative, bool organized)
{   
    // 创建提取对象
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setNegative(negative); // 设置是否反转行为
    extract.setInputCloud(cloud_in); // 设置输入点云
    extract.setIndices(inliers); // 设置内点索引
    extract.setKeepOrganized(organized); // 设置是否保持点云结构
    extract.filter(*cloud_out); // 执行过滤操作
}

// 点云变换函数
void Utilities::transformCloud(Eigen::Affine3d c2cp,
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud)
{
    // 使用PCL的transformPointCloud函数进行点云变换
    pcl::transformPointCloud(*input_cloud, *output_cloud, c2cp);
}

// 体素化降采样方法
void Utilities::voxelizingDownsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                                    float leaf_xy, float leaf_z)
{
    // 创建体素网格滤波对象
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(input_cloud); // 设置输入点云
    filter.setLeafSize(leaf_xy, leaf_xy, leaf_z); // 设置体素大小
    filter.filter(*output_cloud); // 执行滤波
}

// 平滑处理方法
void Utilities::smoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud)
{
    // 创建移动最小二乘平滑对象
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
    filter.setInputCloud(input_cloud); // 设置输入点云
    filter.setSearchRadius(0.05); // 设置搜索半径为5cm
    filter.setComputeNormals(true); // 计算法线
    // 创建KD树对象用于搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    filter.setSearchMethod(kdtree); // 设置搜索方法
    filter.process(*output_cloud); // 执行平滑处理
}

// 点云裁剪函数
void Utilities::cropping(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                        double min_x, double max_x, double min_y, double max_y, double min_z, double max_z)
{
    // 定义裁剪盒尺寸
    double xmin = min_x, xmax = max_x, ymin = min_y, ymax = max_y, zmin = min_z, zmax = max_z;
    // 创建裁剪盒对象
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(input_cloud); // 设置输入点云
    Eigen::Vector4f min_point, max_point;
    min_point << xmin, ymin, zmin, 1.0;
    max_point << xmax, ymax, zmax, 1.0;
    crop.setMin(min_point);
    crop.setMax(max_point);
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    crop.filter(*output_cloud);
}

// 移除离群点函数
void Utilities::removeOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                               int mean_k, double stddev_mul_thresh)
{
    // 创建统计离群点移除对象
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud); // 设置输入点云
    sor.setMeanK(mean_k); // 设置平均值计算的邻近点数
    sor.setStddevMulThresh(stddev_mul_thresh); // 设置标准差乘数阈值
    sor.filter(*output_cloud); // 执行过滤
}

// 欧几里德聚类函数
void Utilities::euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud)
{
    // 创建KdTree对象，用于搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input_cloud);

    // 存储聚类结果的向量
    std::vector<pcl::PointIndices> cluster_indices;
    // 创建欧几里德聚类对象
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.05); // 设置聚类容差为5cm
    ec.setMinClusterSize(30); // 设置最小聚类点数
    ec.setMaxClusterSize(25000); // 设置最大聚类点数
    ec.setSearchMethod(tree); // 设置搜索方法
    ec.setInputCloud(input_cloud); // 设置输入点云
    ec.extract(cluster_indices); // 执行聚类

    // 仅保留最大聚类
    if (!cluster_indices.empty())
    {
        for (std::vector<int>::const_iterator pit = cluster_indices.front().indices.begin(); pit != cluster_indices.front().indices.end(); ++pit)
        {
            output_cloud->points.push_back(input_cloud->points[*pit]); //将每个点添加到输出点云中
        }
        output_cloud->width = output_cloud->points.size(); //设置输出点云的宽度 
        output_cloud->height = 1; //设置输出点云的高度
        output_cloud->is_dense = true; //设置输出点云是否密集
    }
}

// 查找低于指定Y阈值的点的平均X坐标
float Utilities::findAvgXForPointsBelowYThreshold(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                  double yThreshold, int x_neighbors, bool is_min)
{
    // 根据Y坐标阈值过滤点
    std::vector<pcl::PointXYZ> filteredPoints;
    for (const auto &point : cloud->points)
    {
        if (point.y < yThreshold)
        {
            filteredPoints.push_back(point);
        }
    }

    // 根据is_min参数对过滤后的点按X坐标升序或降序排序
    if (is_min)
    {
        std::sort(filteredPoints.begin(), filteredPoints.end(),
                  [](const pcl::PointXYZ &a, const pcl::PointXYZ &b)
                  {
                      return a.x < b.x;
                  });
    }
    else
    {
        std::sort(filteredPoints.begin(), filteredPoints.end(),
                  [](const pcl::PointXYZ &a, const pcl::PointXYZ &b)
                  {
                      return a.x > b.x;
                  });
    }

    // 取前x_neighbors个点（如果点数少于x_neighbors，则取所有点）
    int numPointsToAverage = std::min(static_cast<int>(filteredPoints.size()), x_neighbors);

    // 计算平均X坐标值
    double avgX = 0.0;
    for (int i = 0; i < numPointsToAverage; ++i)
    {
        avgX += filteredPoints[i].x;
    }

    if (numPointsToAverage > 0)
    {
        avgX /= numPointsToAverage;
    }

    return static_cast<float>(avgX);
}

// 角度转换函数：度到弧度
float Utilities::deg2rad(float deg_angle)
{
    return (deg_angle * 0.017453293f); // 0.017453293 是 π/180 的近似值
}

// 角度转换函数：弧度到度
float Utilities::rad2deg(float rad_angle)
{
    return (rad_angle / 0.017453293f); // 0.017453293 是 π/180 的近似值
}

// 点云着色函数
void Utilities::colorize(const pcl::PointCloud<pcl::PointXYZ> &pc,
                         pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
                         const std::vector<int> &color)
{
    int N = pc.points.size(); // 获取点云中点的数量

    // 清空彩色点云
    pc_colored.clear();

    pcl::PointXYZRGB pt_tmp;
    for (int i = 0; i < N; ++i)
    {
        const auto &pt = pc.points[i];

        // 为临时点赋值坐标和颜色
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];

        // 将着色后的点添加到彩色点云中
        pc_colored.points.emplace_back(pt_tmp);
    }
}

// 注释掉的函数：从像素坐标获取点云中的点
// bool getPointFromPixelCoordinates(pcl::PointXYZ& point, int pixel_x, int pixel_y, const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud)
// {
//     // 检查像素坐标是否在有效范围内
//     if (pixel_x >= 0 && pixel_x < pcl_cloud->width && pixel_y >= 0 && pixel_y < pcl_cloud->height) {
//         // 计算点云中对应像素坐标的索引
//         int index = pixel_y * pcl_cloud->width + pixel_x;
//         point = pcl_cloud->at(index);
//         return true;
//     } else {
//         return false;
//     }
// }
Utilities::Utilities(){} 