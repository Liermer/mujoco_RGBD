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

#ifndef PLANE_HPP
#define PLANE_HPP

// Third party libraries
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <Eigen/Dense>
#include <pcl/common/common.h>
// Custom utilities for the Plane class
#include "Utilities.hpp"

/**
 * @class Plane
 * @brief Represents a geometric plane and provides utilities for computing plane characteristics.
 *
 * The Plane class encapsulates a point cloud representation of a plane, providing methods for 
 * computing various attributes such as the plane's convex hull, slope, centroid, and principal 
 * directions. The class also provides tools for projecting the plane to 2D and calculating 
 * measurements related to the plane.
 */
class Plane
{
    public:

        /**
         * @brief Default constructor. Initializes empty clouds and other members.
         */
        Plane(){
            cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
            cloud_projected_.reset(new pcl::PointCloud<pcl::PointXYZ>());
            hull_.reset(new pcl::PointCloud<pcl::PointXYZ>());
            plane_dir_.setZero();
            type_ = 1; // set default type to level
        }

        /**
         * @brief Construct a new Plane object and processes it.
         * @param cloud_in Input point cloud representing the plane.
         * @param plane_coeff Plane coefficients.
         */
        Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::ModelCoefficients::Ptr plane_coeff) {
            plane_coefficients_ = plane_coeff;
            cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
            cloud_projected_.reset(new pcl::PointCloud<pcl::PointXYZ>());
            hull_.reset(new pcl::PointCloud<pcl::PointXYZ>());
            plane_dir_.setZero();
            *cloud_ = *cloud_in;
            processPlane();
            type_ = 1; // set default type to level
        }

        /**
         * @brief Default destructor.
         */
        ~Plane(){}

        /**
         * @brief Get a reference to the current Plane object.
         * @return A reference to the current Plane object.
         */
        Plane& get(){
            return *this;
        }

        /**
         * @brief Processes the plane by setting its features and computing its convex hull.
         */
        void processPlane();

        /**
         * @brief Calculates the slope of the plane.
         */
        void calcPlaneSlope();

        // Public Members:

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_; // Original point cloud of the plane
        pcl::PointCloud<pcl::PointXYZ>::Ptr hull_;  // Convex hull of the projected plane
        pcl::PointXYZ centroid_;                    // Centroid of the plane
        pcl::PointXYZ center_;                      // Center of the bounding rectangle

        pcl::ModelCoefficients::Ptr plane_coefficients_;   // Plane coefficients
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected_;  // 2D projected cloud of the plane

        int type_;                // Type identifier (if needed for categorizing planes)
        float slope_;             // Slope of the plane in degrees
        float width_;             // Width measurement of the plane
        float length_;            // Length measurement of the plane
        Eigen::Matrix3f plane_dir_;   // PCA direction of the plane

    private:

        /**
         * @brief Computes the convex hull of the projected plane.
         */
        void computePlaneHull();

        /**
         * @brief Projects the plane to 2D using the plane coefficients.
         */
        void projectPlaneTo2D();

        /**
         * @brief Computes the centroid of the plane's points.
         */
        void getCentroid();

        /**
         * @brief Computes the principal directions of the plane using PCA.
         */
        void getPrincipalDirections();

        /**
         * @brief Computes measurements of the plane like width and length based on PCA directions.
         */
        void getMeasurements();
};

#endif // PLANE_HPP 