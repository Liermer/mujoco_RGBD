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

#ifndef STAIR_HPP
#define STAIR_HPP

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm> // for std::sort

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Plane.hpp"
#include "Utilities.hpp"

// Parameters for height of the stairs, given by the regulations:
const float k_height_min = 0.07f;  // Min height 
const float k_height_max = 1.f;    //1.f; 0.2f Max height 
const float k_area_min = 0.2f; // Min step area

class Stair
{   
public:
    Stair(const std::vector<Plane>& planes);
    Stair();
    ~Stair() {}

    Stair& get() { return *this; }

    Eigen::Quaterniond getStairOrientation() const;
    void setStairOrientation();
    Eigen::Affine3d getStairPose() const;
    void setStairPose();

    void TransformPoseToMap(Eigen::Affine3d& bp2m);
    void TransformPoseToBase(Eigen::Affine3d& m2bp);

    pcl::PointCloud<pcl::PointXYZ>::Ptr getCombinedCloud() const;

    // Members:
    std::vector<Plane> Planes_;
    int type_;
    float step_width_;
    float step_length_;
    float step_height_;
    float step_distance_;
    float step_angle_;

    Eigen::Affine3d stair_pose_;
    Eigen::Affine3d stair_pose_in_map_;
};

#endif // STAIR_HPP 