// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "mimosa/lidar/utils.hpp"
#include "mimosa/utils.hpp"

namespace mimosa
{
namespace lidar
{

struct RegistrationConfig
{
  float source_voxel_grid_filter_leaf_size = 0.5;
  float source_voxel_grid_min_dist_in_voxel = 0.1;
  float target_ivox_map_leaf_size = 0.5;
  float target_ivox_map_min_dist_in_voxel = 0.1;
  size_t num_corres_points = 5;
  float max_corres_distance = 2.24;
  float plane_validity_distance = 0.04;
  float lidar_point_noise_std_dev = 0.02;
  bool use_huber = true;
  float huber_threshold = 1.345;
  bool reg_4_dof = false;
  bool project_on_degneneracy = true;
  float degen_thresh_rot = 10;
  float degen_thresh_trans = 15;
};

void declare_config(RegistrationConfig & config);

struct GeometricConfig
{
  std::string logs_directory = "/tmp/";
  bool enabled = true;
  std::string log_level = "info";

  std::string map_frame = "mimosa_map";
  std::string sensor_frame = "mimosa_lidar";
  std::string body_frame = "mimosa_body";
  gtsam::Pose3 T_B_L = gtsam::Pose3::Identity();
  // Output (base_link) frame. Empty => use body_frame (backward compatible). Must match the
  // graph/imu managers, which read the same top-level `output_frame` key.
  std::string output_frame = "";
  // Pose of the output/base frame in the body/IMU frame. Used to express the published keyframe
  // poses in the same output frame as the graph manager's path/odometry.
  gtsam::Pose3 T_B_Base = gtsam::Pose3::Identity();

  inline std::string outputFrame() const
  {
    return output_frame.empty() ? body_frame : output_frame;
  }

  int point_skip_divisor = 1;
  int ring_skip_divisor = 1;
  float map_keyframe_trans_thresh = 0.1;
  float map_keyframe_rot_thresh_deg = 10;
  size_t initial_clouds_to_force_map_update = 10;
  size_t lru_horizon = 100;
  size_t neighbor_voxel_mode = 7;

  RegistrationConfig scan_to_map;
};

void declare_config(GeometricConfig & config);
}  // namespace lidar
}  // namespace mimosa
