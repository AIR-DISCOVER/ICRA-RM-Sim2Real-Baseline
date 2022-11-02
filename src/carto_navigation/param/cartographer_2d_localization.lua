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
  tracking_frame = "base_link",  -- IMU_link
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1, 
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 1.0,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 1e-2,
  trajectory_publish_period_sec = 30e-2,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 8.0
--TRAJECTORY_BUILDER_2D.missing_data_ray_length = 10.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window=0.15
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window=math.rad(18.0)
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight=0.1
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight=0.1
--TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
--TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(10.0)
--TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 3600.0
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points=0.5*TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range=0.1*TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_rang
----TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points=0.5*TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points
--TRAJECTORY_BUILDER_2D.voxel_filter_size=2.0*TRAJECTORY_BUILDER_2D.voxel_filter_size
TRAJECTORY_BUILDER_2D.submaps.num_range_data=35
--TRAJECTORY_BUILDER.pure_localization = true
TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 3,
}
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight=8.0
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight=40.0
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight=15.0


--TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability=0.51
--TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability=0.49

POSE_GRAPH.constraint_builder.min_score = 0.60
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
--POSE_GRAPH.constraint_builder.max_constraint_distance = 7.0
--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window=7.0
--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.0)
--POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight=40.0
--POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight=15.0
--POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight=5.0
POSE_GRAPH.optimize_every_n_nodes = 1
MAP_BUILDER.num_background_threads = 12
POSE_GRAPH.constraint_builder.sampling_ratio = 0.15 
POSE_GRAPH.global_sampling_ratio = 0.01
POSE_GRAPH.max_num_final_iterations = 1

--POSE_GRAPH.constraint_builder.loop_closure_translation_weight
--POSE_GRAPH.constraint_builder.loop_closure_rotation_weight
--POSE_GRAPH.matcher_translation_weight
--POSE_GRAPH.matcher_rotation_weight

return options
