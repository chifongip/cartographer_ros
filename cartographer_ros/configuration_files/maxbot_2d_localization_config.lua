-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom", -- base_link
  odom_frame = "odom",
  provide_odom_frame = false, --true
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.05,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 50 -- 35

TRAJECTORY_BUILDER_2D.min_range = 0.1 -- 0.3
TRAJECTORY_BUILDER_2D.max_range = 50. -- 8, maximum 25
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- TRAJECTORY_BUILDER_2D.use_multi_resolution_ceres_scan_matching = false
-- TRAJECTORY_BUILDER_2D.use_accurate_imu = false

POSE_GRAPH.optimize_every_n_nodes = 20 -- 35
-- POSE_GRAPH.revert_scan = false
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65 -- 0.66
POSE_GRAPH.global_constraint_search_after_n_seconds = 120.
POSE_GRAPH.constraint_builder.min_score = 0.60 -- 0.65
POSE_GRAPH.constraint_builder.log_matches = false
POSE_GRAPH.constraint_builder.max_constraint_distance = 15 
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
-- POSE_GRAPH.constraint_builder.global_localization_min_score_in_elevator = 0.65 --判定电梯内重定位成功最小得分
-- POSE_GRAPH.constraint_builder.matches_well_score_in_elevator = 0.75 --电梯内重定位匹配得分较好阈值
-- POSE_GRAPH.match_score_elevator_rectangle = 0.67 --判定电梯框匹配成功得分阈值


TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(5.)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 5 -- 1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 30 -- 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1 -- 40

-- POSE_GRAPH.elevator_dis = -1.5

return options
