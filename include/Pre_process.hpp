#ifndef PRE_PROCESS_HPP
#define PRE_PROCESS_HPP

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

#include "StairDetector.hpp"

class Pre_process{
    public:
        Pre_process(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud);
        ~Pre_process();
        void pre_process(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud);

    private:
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_filtered_;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_projected_;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_hull_;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_centroid_;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_principal_directions_;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_measurements_;
};

#endif