-- Cartographer 2D LIDAR configuration for SICK TiM561
-- NO odometry required - pure lidar SLAM!

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,  -- Cartographer publishes odom->base_link from scan matching
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,  -- No external odometry
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,  -- 200 Hz
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D SLAM configuration (no IMU, no odometry)
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Scan matching tuning for 20 Hz lidar - BALANCED FOR MOVING DRONE
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.  -- Good feature matching
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.     -- BALANCED - equal priority for movement
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.        -- BALANCED - equal priority for rotation  
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 40  -- Fast enough for real-time
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- Motion filter - BALANCED for flying drone (translation + rotation)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2   -- Process frequently during flight
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05  -- 5cm movement threshold (drone moving)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)  -- 1.0° rotation threshold

-- Submaps
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90  -- 90 scans = 4.5 seconds at 20 Hz
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 5 cm resolution
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49

-- Pose graph optimization - TUNED FOR BETTER LOOP CLOSURE
-- Pose graph optimization - BALANCED FOR MOVING DRONE
POSE_GRAPH.optimize_every_n_nodes = 60  -- Optimize every 3 seconds (60 scans at 20Hz)
POSE_GRAPH.constraint_builder.min_score = 0.55  -- Good quality matches for moving drone
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3  -- Standard sampling
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0  -- Standard search radius
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.  -- BALANCED - handle translation
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)  -- BALANCED - handle rotation
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.  -- Standard weight
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.     -- BALANCED - equal with rotation
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 10.        -- BALANCED - equal with translation  
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5

-- Optimization problem
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimization_problem.acceleration_weight = 1.1e2
POSE_GRAPH.optimization_problem.rotation_weight = 1.6e4
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0  -- Not using odometry
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e2
POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = false
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4  -- Jetson has 6 cores

-- Constraints
POSE_GRAPH.max_num_final_iterations = 200
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.log_residual_histograms = false
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.

return options
