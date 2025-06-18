#ifndef STAIR_DETECTOR_HPP
#define STAIR_DETECTOR_HPP

#include <vector>
#include <string>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

#include "Stair.hpp"
#include "Plane.hpp"
#include "Utilities.hpp"

struct StairDetectorConfig {
    double leaf_size_xy_ = 0.05;
    double leaf_size_z_ = 0.05;
    double min_x_ = 0.3;
    double max_x_ = 3.0;
    double min_y_ = -1.0;
    double max_y_ = 1.0;
    double min_z_ = -1.0;
    double max_z_ = 1.0;
    double distance_threshold_ = 0.02;
    int max_iterations_ = 100;
    double angle_threshold_ = 2.0;
    double cluster_tolerance_ = 0.2;
    int min_cluster_size_ = 50;
    int max_cluster_size_ = 1000;
    int mean_k_ = 50;
    double stddev_mul_thresh_ = 1.0;
    int k_neighbors_ = 10;
    int x_neighbors_ = 30;
    double y_threshold_ = 0.055;
    int filter_min_limit_ = 5;
    int filter_max_limit_ = 15;
    double pos_err_thresh_ = 0.2;
    double w_ = 0.5; 
    bool debug_ = false;
};

class StairDetector {
public:
    explicit StairDetector(const StairDetectorConfig& config);
    ~StairDetector();

    void processCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    bool isStairDetected(Stair& detected_stair);

private:
    void reset();
    void getPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
    void findFloor();
    bool checkForValidCandidate(Stair& stair);
    void decrementCounter(std::vector<int>& counter_buffer);
    void removeBelowThresh(std::vector<Stair>& stairs_buffer, std::vector<int>& counter_buffer);
    void pushToBuffers(std::vector<Stair>& stairs_buffer, std::vector<int>& counter_buffer, Stair& stair);
    void updateBuffers(std::vector<Stair>& stairs_buffer, std::vector<int>& counter_buffer, Stair& stair);
    bool isDetectedStair(std::vector<Stair>& stairs_buffer, std::vector<int>& counter_buffer, Stair& detect_stair);
    double calculatePositionError(const Stair& stair1, const Stair& stair2);
    Stair updateStair(Stair& stair1, Stair& stair2);
    void setStair();
    void calcPlaneSlope();
    
    StairDetectorConfig config_;

    Stair Stair_;
    std::vector<Plane> Planes_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    Stair detected_stair_filtered_;
    std::vector<Stair> stairs_arr_;
    std::vector<int> stairs_counts_arr_;
    bool stair_detected_;
    bool planes_empty_;
};

#endif // STAIR_DETECTOR_HPP 