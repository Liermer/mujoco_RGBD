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
    StairDetectorConfig config; // Use default config
    config.debug_ = true;
    config.distance_threshold_ = 0.05; // 5cm
    config.angle_threshold_ = 5.0; // 5 degrees
    config.leaf_size_xy_ = 0.01; // 1cm
    config.leaf_size_z_ = 0.01; // 1cm
    StairDetector detector(config);

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> current_cloud = cloud;
    
    if (current_cloud) {
        std::cout << "\n--- Processing new cloud ---" << std::endl;
        detector.processCloud(current_cloud);
        
        Stair detected_stair;
        if (detector.isStairDetected(detected_stair)) {
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





