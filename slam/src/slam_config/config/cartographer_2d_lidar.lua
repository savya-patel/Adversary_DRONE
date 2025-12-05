-- Cartographer 2D configuration for SICK TiM series LiDAR.
-- Odometry is not required; scan matching is used for pose estimation.
--
-- TUNING GUIDE:
-- This config balances real-time performance (Jetson CPU) with mapping accuracy.
-- Key adjustables are marked with [TUNE] below.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,  -- Cartographer publishes odom → base_link
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,  -- No external odometry required
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,  -- [TUNE] Lower for faster submap updates
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,  -- [TUNE] Use 0.5 to halve scan processing for speed
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- ============================================================================
-- 2D TRAJECTORY BUILDER: Local scan matching and submap creation
-- ============================================================================

-- Range limits for incoming laser scans
TRAJECTORY_BUILDER_2D.min_range = 0.1  -- Ignore measurements closer than 10 cm
TRAJECTORY_BUILDER_2D.max_range = 10.0  -- Ignore measurements farther than 10 m
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0  -- Extrapolate missing rays to 5 m

-- IMU: disabled for scan-only operation
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- Online correlative scan matching: real-time loop closure detection
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- ============================================================================
-- SCAN MATCHING: Local ICP-like optimization (Ceres solver)
-- ============================================================================
-- Ceres solver minimizes: (occupied_space_weight * occupied_cost)
--                        + (translation_weight * translation_cost)
--                        + (rotation_weight * rotation_cost)

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.  -- Penalize unmatched occupied cells
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.     -- [TUNE] Weight of XY movement cost
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.        -- [TUNE] Weight of rotation cost (higher = favor rotation over translation)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 40  -- [TUNE] More iterations = better fit but slower
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- ============================================================================
-- MOTION FILTER: Throttle scans for efficiency
-- ============================================================================
-- Only insert scans into submaps if motion exceeds thresholds.

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2   -- Insert at least every 200 ms
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05  -- [TUNE] Insert on 5 cm movement (↓ for denser map, ↑ for speed)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)  -- [TUNE] Insert on 1° rotation (degrees)


-- ============================================================================
-- SUBMAPS: Local grid representation
-- ============================================================================
-- Each submap is a small 2D occupancy grid. Multiple submaps form the full map.

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90  -- [TUNE] Scans per submap (90 at ~20 Hz = ~4.5 sec)
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- [TUNE] Grid cell size in meters (0.05 m = 5 cm)
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55  -- [TUNE] Confidence of occupied cells
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49

-- ============================================================================
-- POSE GRAPH: Global loop closure and optimization
-- ============================================================================

-- Constraint generation and scoring
POSE_GRAPH.optimize_every_n_nodes = 60  -- [TUNE] Optimize after every 60 scans (~3 sec at 20 Hz)
POSE_GRAPH.constraint_builder.min_score = 0.55  -- [TUNE] Minimum scan-to-submap match score (0.0–1.0)
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60  -- [TUNE] For loop-closure matches

-- Fast correlative scan matcher (coarse loop detection)
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3  -- Sample 30% of submap pairs
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0  -- [TUNE] Max search radius for loop closure (m)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.  -- [TUNE] XY search window (m)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)  -- [TUNE] Rotation search window (degrees)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- Ceres scan matcher (fine loop closure refinement)
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.  -- Penalize unmatched cells
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.     -- [TUNE] Weight of loop-closure XY correction
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 10.        -- [TUNE] Weight of loop-closure rotation correction
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4  -- [TUNE] Loop closure translation penalty
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5  -- [TUNE] Loop closure rotation penalty

-- Global optimization (Ceres)
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimization_problem.acceleration_weight = 1.1e2
POSE_GRAPH.optimization_problem.rotation_weight = 1.6e4  -- [TUNE] How much to trust rotation over position
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5  -- Strong constraint on local poses
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0  -- No odometry
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e2
POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = false
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50  -- [TUNE] More iterations = better global fit
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4  -- Jetson Orin Nano has 6 CPU cores

-- Final optimization at end of trajectory
POSE_GRAPH.max_num_final_iterations = 200  -- [TUNE] More iterations for final refinement
POSE_GRAPH.global_sampling_ratio = 0.003  -- Sample 0.3% of all pose pairs
POSE_GRAPH.log_residual_histograms = false
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.  -- Start global loop detection after 10 sec

return options
