#include "StairDetector.hpp"

#include <iostream>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>

StairDetector::StairDetector(const StairDetectorConfig& config) : config_(config) {
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    stair_detected_ = false;
    planes_empty_ = true;
    if (config_.debug_) {
        std::cout << "StairDetector initialized in debug mode." << std::endl;
    }
}

StairDetector::~StairDetector() {}

void StairDetector::processCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->empty()) {
        if (config_.debug_) {
            std::cout << "Input cloud is empty." << std::endl;
        }
        return;
    }

    if (config_.debug_) {
        std::cout << "Processing new cloud with " << cloud->size() << " points." << std::endl;
    }

    // --- Start of logic from original pclCallback ---

    // Reset state
    reset();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);

    // Cropping
    if (config_.debug_) {
        std::cout << "Cropping parameters: "
                  << "x=[" << config_.min_x_ << "," << config_.max_x_ << "], "
                  << "y=[" << config_.min_y_ << "," << config_.max_y_ << "], "
                  << "z=[" << config_.min_z_ << "," << config_.max_z_ << "]" << std::endl;
    }
    Utilities::cropping(cloud, cloud_cropped, config_.min_x_, config_.max_x_, config_.min_y_, config_.max_y_, config_.min_z_, config_.max_z_);
    if (cloud_cropped->empty()){
        if(config_.debug_){ std::cout << "Cloud empty after cropping" << std::endl; }
        return;
    }
    if (config_.debug_) {
        std::cout << "Cloud size after cropping: " << cloud_cropped->size() << std::endl;
    }
    
    // Downsampling
    Utilities::voxelizingDownsample(cloud_cropped, cloud_downsampled, config_.leaf_size_xy_, config_.leaf_size_z_);
    if (cloud_downsampled->empty()){
        if(config_.debug_){ std::cout << "Cloud empty after downsampling" << std::endl;}
        return;
    }
    if (config_.debug_) {
        std::cout << "Cloud size after downsampling: " << cloud_downsampled->size() << std::endl;
    }

    *cloud_ = *cloud_downsampled;

    // Get Planes
    getPlanes(cloud_);

    if(Planes_.empty()){
         if(config_.debug_) { std::cout << "No planes detected" << std::endl; }
        planes_empty_ = true;
    } else {
        planes_empty_ = false;
        if(config_.debug_){ std::cout << "Detected " << Planes_.size() << " planes" << std::endl; }

        Stair singlePlaneStair(Planes_);
        
        if(checkForValidCandidate(singlePlaneStair)){
            updateBuffers(stairs_arr_, stairs_counts_arr_, singlePlaneStair);
        }
    }
    
    decrementCounter(stairs_counts_arr_);
    removeBelowThresh(stairs_arr_, stairs_counts_arr_);
    stair_detected_ = isDetectedStair(stairs_arr_, stairs_counts_arr_, detected_stair_filtered_);

    if(stair_detected_){
        if(config_.debug_){
            std::cout << "Stair detected!" << std::endl;
            std::cout << "  - Type: " << (detected_stair_filtered_.type_ == 0 ? "Up" : "Down") << std::endl;
            std::cout << "  - Distance: " << detected_stair_filtered_.step_distance_ << std::endl;
            std::cout << "  - Height: " << detected_stair_filtered_.step_height_ << std::endl;
        }
    }
    // --- End of logic from original pclCallback ---
}

bool StairDetector::isStairDetected(Stair& detected_stair){
    if(stair_detected_){
        detected_stair = detected_stair_filtered_;
        return true;
    }
    return false;
}

void StairDetector::reset() {
    Planes_.clear();
}

void StairDetector::getPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    // ... (Implementation from StairModeling::getPlanes)
    // Remember to replace get_logger() with std::cout for debugging
    // and use config_ members for parameters.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>(*input_cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0)); // Temporarily disable axis constraint
    // seg.setEpsAngle(Utilities::deg2rad(config_.angle_threshold_)); // Temporarily disable angle constraint
    seg.setDistanceThreshold(config_.distance_threshold_);
    seg.setMaxIterations(config_.max_iterations_);

    int nr_points = cloud_filtered->size();

    while (cloud_filtered->size() > 0.3 * nr_points) {
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            break;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Utilities::getCloudByInliers(cloud_filtered, plane_cloud, inliers, false, false);
        
        Plane plane(plane_cloud, coefficients);
        if(plane.length_ > 0.2 || plane.width_ > 0.2){
             Planes_.push_back(plane);
        }

        Utilities::getCloudByInliers(cloud_filtered, cloud_filtered, inliers, true, false);
    }
}


void StairDetector::findFloor() {
    // ... (Implementation from StairModeling::findFloor)
    if (Planes_.empty()) return;

    double min_x_val = std::numeric_limits<double>::max();
    int floor_idx = -1;

    for(size_t i = 0; i < Planes_.size(); ++i) {
        float avg_x = Utilities::findAvgXForPointsBelowYThreshold(Planes_[i].cloud_, config_.y_threshold_, config_.x_neighbors_, true);
        if(avg_x < min_x_val) {
            min_x_val = avg_x;
            floor_idx = i;
        }
    }

    if(floor_idx != -1) {
        Planes_[floor_idx].type_ = 0; // 0 for floor
    }
}


bool StairDetector::checkForValidCandidate(Stair& stair) {
    // ... (Implementation from StairModeling::checkForValidCandidate)
    if(stair.Planes_.size() >= 2 && (stair.Planes_.back().length_>0.4 || stair.Planes_.back().width_>0.4)){
        return true;
    }
    return false;
}

void StairDetector::decrementCounter(std::vector<int>& counter_buffer) {
    // ... (Implementation from StairModeling::decrementCounter)
    for (int & i : counter_buffer){
        i > 0 ? i-- : i=0;
    }
}

void StairDetector::removeBelowThresh(std::vector<Stair>& stairs_buffer, std::vector<int>& counter_buffer) {
    // ... (Implementation from StairModeling::removeBelowThresh)
    std::vector<Stair> new_stairs_buffer;
    std::vector<int> new_counter_buffer;

    for (size_t i = 0; i < stairs_buffer.size(); ++i) {
        if (counter_buffer[i] >= config_.filter_min_limit_) {
            new_stairs_buffer.push_back(stairs_buffer[i]);
            new_counter_buffer.push_back(counter_buffer[i]);
        }
    }
    stairs_buffer = new_stairs_buffer;
    counter_buffer = new_counter_buffer;
}

void StairDetector::pushToBuffers(std::vector<Stair>& stairs_buffer, std::vector<int>& counter_buffer, Stair& stair) {
    // ... (Implementation from StairModeling::pushToBuffers)
    stairs_buffer.push_back(stair);
    counter_buffer.push_back(config_.filter_max_limit_);
}

void StairDetector::updateBuffers(std::vector<Stair>& stairs_buffer, std::vector<int>& counter_buffer, Stair& stair) {
    // ... (Implementation from StairModeling::updateBuffers)
    bool found_match = false;
    for (size_t i = 0; i < stairs_buffer.size(); ++i) {
        if (calculatePositionError(stairs_buffer[i], stair) < config_.pos_err_thresh_) {
            stairs_buffer[i] = updateStair(stairs_buffer[i], stair);
            counter_buffer[i] = config_.filter_max_limit_;
            found_match = true;
            break;
        }
    }

    if (!found_match) {
        pushToBuffers(stairs_buffer, counter_buffer, stair);
    }
}

bool StairDetector::isDetectedStair(std::vector<Stair>& stairs_buffer, std::vector<int>& counter_buffer, Stair& detect_stair) {
    // ... (Implementation from StairModeling::isDetectedStair)
    for (size_t i = 0; i < stairs_buffer.size(); ++i) {
        if (counter_buffer[i] == config_.filter_max_limit_) {
            detect_stair = stairs_buffer[i];
            return true;
        }
    }
    return false;
}

double StairDetector::calculatePositionError(const Stair& stair1, const Stair& stair2) {
    // ... (Implementation from StairModeling::calculatePositionError)
    auto pos1 = stair1.getStairPose().translation();
    auto pos2 = stair2.getStairPose().translation();
    return (pos1 - pos2).norm();
}

Stair StairDetector::updateStair(Stair& stair1, Stair& stair2) {
    // ... (Implementation from StairModeling::updateStair)
    double w = config_.w_;
    stair1.step_distance_ = w * stair1.step_distance_ + (1 - w) * stair2.step_distance_;
    stair1.step_height_ = w * stair1.step_height_ + (1 - w) * stair2.step_height_;
    stair1.step_length_ = w * stair1.step_length_ + (1 - w) * stair2.step_length_;
    stair1.step_width_ = w * stair1.step_width_ + (1 - w) * stair2.step_width_;
    stair1.setStairPose();
    return stair1;
}

void StairDetector::setStair() {
    // ... (Implementation from StairModeling::setStair)
    // This method seems to be about setting a global stair object.
    // It might need to be re-evaluated in the context of the new class.
    // For now, I'll leave it empty.
}

void StairDetector::calcPlaneSlope() {
    // ... (Implementation from StairModeling::calcPlaneSlope)
    for (auto& plane : Planes_) {
        plane.calcPlaneSlope();
    }
} 