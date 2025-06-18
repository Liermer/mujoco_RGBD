#include "Pre_process.hpp"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <librealsense2/rs.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <chrono>

// Pre_process类的构造函数
Pre_process::Pre_process(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud) 
    : cloud_(cloud) {
    cloud_filtered_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud_projected_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud_hull_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud_centroid_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud_principal_directions_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud_measurements_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

// Pre_process类的析构函数
Pre_process::~Pre_process() {
    // 智能指针会自动清理
}

// Pre_process类的成员函数
void Pre_process::pre_process(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud) {
    // 如果stair_detector_未初始化，则创建它
    if (!stair_detector_) {
        StairDetectorConfig config; // 使用YAML文件中的配置
        config.debug_ = true; // 保持调试开启

        // Voxel Filter
        config.leaf_size_xy_ = 0.04;
        config.leaf_size_z_ = 0.08;

        // Crop Box
        config.min_x_ = -0.3;
        config.max_x_ = 0.3;
        config.min_y_ = -0.5;
        config.max_y_ = 0.5;
        config.min_z_ = -2.0;
        config.max_z_ = 0.5;

        // Segmentation
        config.distance_threshold_ = 0.02;
        config.max_iterations_ = 300;
        config.angle_threshold_ = 10.0;

        // Clustering
        config.cluster_tolerance_ = 0.035;
        config.min_cluster_size_ = 100;
        config.max_cluster_size_ = 25000;

        // Floor Finding
        config.k_neighbors_ = 30;

        // Avg X Calculation
        config.x_neighbors_ = 500;
        config.y_threshold_ = 0.055;

        // Stair Filter
        config.filter_min_limit_ = 3;
        config.filter_max_limit_ = 12;
        config.pos_err_thresh_ = 0.06;
        config.w_ = 0.5;

        stair_detector_ = std::make_unique<StairDetector>(config);
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> current_cloud = cloud;
    
    if (current_cloud) {
        std::cout << "\n--- Processing new cloud ---" << std::endl;
        stair_detector_->processCloud(current_cloud);
        
        Stair detected_stair;
        if (stair_detector_->isStairDetected(detected_stair)) {
            const auto& pose = detected_stair.getStairPose();
            const auto& translation = pose.translation();
            Eigen::Quaterniond orientation(pose.linear());

            std::cout << "Stair Detected!" << std::endl;
            std::cout << "  Translation: [" << translation.x() << ", " << translation.y() << ", " << translation.z() << "]" << std::endl;
            std::cout << "  Orientation: [" << orientation.x() << ", " << orientation.y() << ", "
                      << orientation.z() << ", " << orientation.w() << "]" << std::endl;
        } else {
            std::cout << "No stair was definitively detected in the current cloud." << std::endl;
        }
    }
}

bool Pre_process::is_stair_detected(Stair& detected_stair)
{
    return stair_detector_->isStairDetected(detected_stair);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Pre_process::get_stair_cloud()
{
    if (stair_detector_) {
        return stair_detector_->getDetectedStairCloud();
    }
    return nullptr;
}

std::vector<Plane> Pre_process::get_stair_planes()
{
    if (stair_detector_) {
        return stair_detector_->getDetectedStairPlanes();
    }
    return {};
}





