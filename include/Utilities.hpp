#ifndef UTILITIES_HPP
#define UTILITIES_HPP

// Third party libraries
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/mls.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class Utilities
{
public:
    Utilities();

    /**
     * @brief Extracts a subset of a point cloud based on given inlier indices.
     * @param cloud_in Input cloud.
     * @param cloud_out Output cloud containing extracted points.
     * @param inliers Indices of points to extract.
     * @param negative Whether to extract inliers or outliers.
     * @param organized Whether to keep the spatial organization of the cloud.
     */
    static void getCloudByInliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
                                  const pcl::PointIndices::Ptr& inliers, bool negative, bool organized);

    /**
     * @brief Computes the convex hull of a given point cloud.
     * @param cloud_in Input cloud.
     * @param cloud_hull Output cloud representing the convex hull.
     */
    static void computeHull(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_hull);

    /**
     * @brief Projects the points of a cloud to a 2D plane using the given model coefficients.
     * @param coeff_in Coefficients of the model plane.
     * @param cloud_in Input cloud.
     * @param cloud_out Output cloud containing projected points.
     */
    static void projectCloudTo2D(const pcl::ModelCoefficients::Ptr& coeff_in,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

    /**
     * @brief Applies a transformation to the input cloud and stores the result in the output cloud.
     * @param c2cp Affine transformation to apply.
     * @param input_cloud Input cloud.
     * @param output_cloud Output cloud containing transformed points.
     */
    static void transformCloud(Eigen::Affine3d c2cp,
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud);


    /**
     * @brief Downsamples the input cloud using voxelization.
     * @param input_cloud Input cloud.
     * @param output_cloud Output cloud containing downsampled points.
     * @param leaf_xy Voxel grid size in the XY plane.
     * @param leaf_z Voxel grid size along the Z axis.
     */
    static void voxelizingDownsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                                     float leaf_xy, float leaf_z);

    /**
     * @brief Smoothes the input cloud using the MovingLeastSquares method.
     * @param input_cloud Input cloud.
     * @param output_cloud Output cloud containing smoothed points.
     */
    static void smoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud);

    /**
     * @brief Crops the input cloud within specified minimum and maximum coordinates.
     * @param input_cloud Input cloud.
     * @param output_cloud Output cloud containing cropped points.
     * @param min_x Minimum X coordinate.
     * @param max_x Maximum X coordinate.
     * @param min_y Minimum Y coordinate.
     * @param max_y Maximum Y coordinate.
     * @param min_z Minimum Z coordinate.
     * @param max_z Maximum Z coordinate.
     */
    static void cropping(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                         double min_x,double max_x,double min_y,double max_y,double min_z,double max_z);

    /**
     * @brief Removes statistical outliers from the input cloud.
     * @param input_cloud Input cloud.
     * @param output_cloud Output cloud containing filtered points.
     * @param mean_k Number of neighboring points to use for mean distance estimation.
     * @param stddev_mul_thresh Standard deviation multiplier for the distance threshold calculation.
     */
    static void removeOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                               int mean_k, double stddev_mul_thresh);

    /**
     * @brief Cluster cloud based on euclidean distance
     * @param input_cloud Input cloud.
     * @param output_cloud Output cloud containing filtered points.
     */
    static void euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud);

    /**
     * @brief Finds the average x-coordinate of points below a specified y-coordinate threshold.
     * @param cloud Input cloud.
     * @param yThreshold Y-coordinate threshold.
     * @param x_neighbors Number of neighbors to consider for averaging.
     * @param is_min Whether to sort in ascending or descending order.
     * @return Average x-coordinate of the considered points.
     */
    static float findAvgXForPointsBelowYThreshold(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                  double yThreshold, int x_neighbors, bool is_min);

    /**
     * @brief Converts degrees to radians.
     * @param deg_angle Angle in degrees.
     * @return Angle in radians.
     */
    static float deg2rad(float deg_angle);

    /**
     * @brief Converts radians to degrees.
     * @param rad_angle Angle in radians.
     * @return Angle in degrees.
     */
    static float rad2deg(float rad_angle);

    /**
     * @brief Colorizes a point cloud with a given color.
     * @param pc Input point cloud.
     * @param pc_colored Output colored point cloud.
     * @param color Vector containing RGB values.
     */
    static void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc,
                         pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
                         const std::vector<int> &color);
    
    /**
     * @brief Colorizes a point cloud with a given color.
     * @param pc Input point cloud.
     * @param pc_colored Output colored point cloud.
     * @param color Vector containing RGB values.
     */
    static bool getPointFromPixelCoordinates(pcl::PointXYZ& point, int pixel_x, int pixel_y, const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud);
};

#endif // UTILITIES_HPP 