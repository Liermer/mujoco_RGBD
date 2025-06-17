#include "Stair.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

Stair::Stair(const std::vector<Plane>& planes)
{
    for (const Plane& plane : planes){
        Planes_.push_back(plane);
    } 

    auto compareByX = [](const Plane& plane1, const Plane& plane2) {
        return plane1.centroid_.x < plane2.centroid_.x;
    };
    std::sort(Planes_.begin(), Planes_.end(), compareByX);

    step_height_ = Planes_.front().plane_coefficients_->values[3] 
                        - Planes_.back().plane_coefficients_->values[3];
    auto step_height_abs = fabs(step_height_);                    

    if(step_height_ > 0){  
        type_ = 0; // upwards
        step_distance_ = Utilities::findAvgXForPointsBelowYThreshold(Planes_.front().cloud_,
                                                                0.055, 30, true);
        step_length_ = planes.front().length_;
        step_width_ = planes.front().width_;
    } else {  
        type_ = 1; // downwards
        step_distance_ = Utilities::findAvgXForPointsBelowYThreshold(Planes_.front().cloud_,
                                                                0.055, 30, false);
        step_length_ = planes.back().length_;
        step_width_ = planes.back().width_;
    }

    setStairPose();
    const auto& position = stair_pose_.translation();
    step_angle_ = Utilities::rad2deg(std::atan2(position.y(), position.x()));                                             
}

Stair::Stair(){
    step_distance_ = 0.;
    step_height_ = 0.;
    step_width_ = 0.;
    step_length_ = 0.;
    step_angle_ = 0.;
    stair_pose_.setIdentity();
    stair_pose_in_map_.setIdentity();
}

void Stair::setStairPose()
{   
    Plane pose_plane = Planes_.front().get();
    Eigen::Vector3d position(step_distance_, pose_plane.centroid_.y, pose_plane.centroid_.z);
    stair_pose_.translation() = position;
    setStairOrientation();
}

Eigen::Affine3d Stair::getStairPose() const
{
    return stair_pose_;
}

void Stair::setStairOrientation()
{
    Plane level_plane;
    if (type_ == 0){
        level_plane = Planes_.back().get();
    }
    else{
        level_plane = Planes_.front().get();
    }

    Eigen::Matrix3d rotation_matrix = level_plane.plane_dir_.cast<double>();
    Eigen::Quaterniond q_rotation(rotation_matrix);

    // Replicating the Euler angle logic from the original code.
    // This seems to correct the orientation based on the principal component analysis result.
    Eigen::Vector3d euler = q_rotation.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX order for yaw, pitch, roll
    double yaw = euler[0];
    double pitch = euler[1];
    
    if(pitch >= 0) {
        q_rotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                     Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(pitch - M_PI/2.0, Eigen::Vector3d::UnitZ());
    } else {
        q_rotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                     Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(pitch + M_PI/2.0, Eigen::Vector3d::UnitZ());
    }
    
    q_rotation.normalize();
    stair_pose_.linear() = q_rotation.toRotationMatrix();
}


Eigen::Quaterniond Stair::getStairOrientation() const
{      
    return Eigen::Quaterniond(stair_pose_.linear());
}

void Stair::TransformPoseToMap(Eigen::Affine3d& bp2m)
{
    stair_pose_in_map_ = bp2m * stair_pose_; 
}

void Stair::TransformPoseToBase(Eigen::Affine3d& m2bp)
{
    stair_pose_ = m2bp * stair_pose_in_map_; 
} 