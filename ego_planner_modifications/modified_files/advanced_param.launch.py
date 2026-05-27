import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # LaunchConfigurations
    map_size_x = LaunchConfiguration('map_size_x_', default=42.0)
    map_size_y = LaunchConfiguration('map_size_y_', default=30.0)
    map_size_z = LaunchConfiguration('map_size_z_', default=5.0)
    
    odometry_topic = LaunchConfiguration('odometry_topic', default='odom')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='camera_pose')
    depth_topic = LaunchConfiguration('depth_topic', default='depth_image')
    cloud_topic = LaunchConfiguration('cloud_topic', default='cloud')
    grid_occ_topic = LaunchConfiguration('grid_occ_topic', default='grid/grid_map/occupancy_inflate')
    grid_pose_type = LaunchConfiguration('grid_pose_type', default=2)
    grid_frame_id = LaunchConfiguration('grid_frame_id', default='world')
    grid_obstacles_inflation = LaunchConfiguration('grid_obstacles_inflation', default=0.40)
    grid_min_ray_length = LaunchConfiguration('grid_min_ray_length', default=0.20)
    grid_resolution = LaunchConfiguration('grid_resolution', default=0.15)
    sector_filter_enable = LaunchConfiguration('sector_filter_enable', default='false')
    sector_min_angle_deg = LaunchConfiguration('sector_min_angle_deg', default=-60.0)
    sector_max_angle_deg = LaunchConfiguration('sector_max_angle_deg', default=60.0)
    sector_use_stride = LaunchConfiguration('sector_use_stride', default='false')
    sector_stride = LaunchConfiguration('sector_stride', default=1)
    sector_use_voxel = LaunchConfiguration('sector_use_voxel', default='false')
    sector_voxel_leaf = LaunchConfiguration('sector_voxel_leaf', default=0.10)
    sector_adaptive_recovery = LaunchConfiguration('sector_adaptive_recovery', default='true')
    sector_auto_disable_on_front_block = LaunchConfiguration('sector_auto_disable_on_front_block', default='false')
    sector_full_view_hold_sec = LaunchConfiguration('sector_full_view_hold_sec', default=2.0)
    virtual_ceil_height = LaunchConfiguration('virtual_ceil_height', default=-0.1)
    z_max = LaunchConfiguration('z_max', default=2.0)
    optimization_dist0          = LaunchConfiguration('optimization_dist0',          default=0.32)
    optimization_lambda_collision= LaunchConfiguration('optimization_lambda_collision',default=2.0)
    optimization_lambda_smooth   = LaunchConfiguration('optimization_lambda_smooth',   default=1.0)
    optimization_lambda_feasibility= LaunchConfiguration('optimization_lambda_feasibility', default=0.1)
    local_update_range_x = LaunchConfiguration('local_update_range_x', default=8.0)
    local_update_range_y = LaunchConfiguration('local_update_range_y', default=8.0)
    local_update_range_z = LaunchConfiguration('local_update_range_z', default=4.5)
    thresh_replan_time    = LaunchConfiguration('thresh_replan_time',    default=1.0)
    thresh_no_replan_meter= LaunchConfiguration('thresh_no_replan_meter',default=1.0)
    cx = LaunchConfiguration('cx', default=321.04638671875)
    cy = LaunchConfiguration('cy', default=243.44969177246094)
    fx = LaunchConfiguration('fx', default=387.229248046875)
    fy = LaunchConfiguration('fy', default=387.229248046875)
    
    max_vel = LaunchConfiguration('max_vel', default=1.5)
    max_acc = LaunchConfiguration('max_acc', default=3.0)
    planning_horizon = LaunchConfiguration('planning_horizon', default=7.5)

    point_num = LaunchConfiguration('point_num', default=1)
    point0_x = LaunchConfiguration('point0_x', default=0.0)
    point0_y = LaunchConfiguration('point0_y', default=0.0)
    point0_z = LaunchConfiguration('point0_z', default=0.0)
    point1_x = LaunchConfiguration('point1_x', default=10.0)
    point1_y = LaunchConfiguration('point1_y', default=10.0)
    point1_z = LaunchConfiguration('point1_z', default=0.0)
    point2_x = LaunchConfiguration('point2_x', default=20.0)
    point2_y = LaunchConfiguration('point2_y', default=20.0)
    point2_z = LaunchConfiguration('point2_z', default=1.0)
    point3_x = LaunchConfiguration('point3_x', default=-10.0)
    point3_y = LaunchConfiguration('point3_y', default=-10.0)
    point3_z = LaunchConfiguration('point3_z', default=1.0)
    point4_x = LaunchConfiguration('point4_x', default=30.0)
    point4_y = LaunchConfiguration('point4_y', default=30.0)
    point4_z = LaunchConfiguration('point4_z', default=1.0)

    flight_type = LaunchConfiguration('flight_type', default=2)
    use_distinctive_trajs = LaunchConfiguration('use_distinctive_trajs', default=True)
    
    obj_num_set = LaunchConfiguration('obj_num_set', default=10)
    
    drone_id = LaunchConfiguration('drone_id', default=0)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # DeclareLaunchArguments
    map_size_x_arg = DeclareLaunchArgument('map_size_x_', default_value=map_size_x, description='Map size along X')
    map_size_y_arg = DeclareLaunchArgument('map_size_y_', default_value=map_size_y, description='Map size along Y')
    map_size_z_arg = DeclareLaunchArgument('map_size_z_', default_value=map_size_z, description='Map size along Z')
    odometry_topic_arg = DeclareLaunchArgument('odometry_topic', default_value=odometry_topic, description='Odometry topic')
    camera_pose_topic_arg = DeclareLaunchArgument('camera_pose_topic', default_value=camera_pose_topic, description='Camera pose topic')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value=depth_topic, description='Depth topic')
    cloud_topic_arg = DeclareLaunchArgument('cloud_topic', default_value=cloud_topic, description='Point cloud topic')
    grid_occ_topic_arg = DeclareLaunchArgument(
        'grid_occ_topic',
        default_value=grid_occ_topic,
        description='Remapped grid_map/occupancy_inflate topic'
    )
    grid_pose_type_arg = DeclareLaunchArgument(
        'grid_pose_type',
        default_value=grid_pose_type,
        description='grid_map pose_type: 1=PoseStamped, 2=Odometry'
    )
    grid_frame_id_arg = DeclareLaunchArgument(
        'grid_frame_id',
        default_value=grid_frame_id,
        description='Target frame id used by grid_map'
    )
    grid_obstacles_inflation_arg = DeclareLaunchArgument(
        'grid_obstacles_inflation',
        default_value=grid_obstacles_inflation,
        description='Obstacle inflation radius for grid map'
    )
    grid_min_ray_length_arg = DeclareLaunchArgument(
        'grid_min_ray_length',
        default_value=grid_min_ray_length,
        description='Minimum range kept from sensor origin'
    )
    grid_resolution_arg = DeclareLaunchArgument(
        'grid_resolution',
        default_value=grid_resolution,
        description='Grid map voxel resolution'
    )
    sector_filter_enable_arg = DeclareLaunchArgument(
        'sector_filter_enable',
        default_value=sector_filter_enable,
        description='Enable embedded sector filter inside ego_planner_node'
    )
    sector_min_angle_deg_arg = DeclareLaunchArgument(
        'sector_min_angle_deg',
        default_value=sector_min_angle_deg,
        description='Embedded sector filter min angle in degree'
    )
    sector_max_angle_deg_arg = DeclareLaunchArgument(
        'sector_max_angle_deg',
        default_value=sector_max_angle_deg,
        description='Embedded sector filter max angle in degree'
    )
    sector_use_stride_arg = DeclareLaunchArgument(
        'sector_use_stride',
        default_value=sector_use_stride,
        description='Embedded sector filter stride sampling on/off'
    )
    sector_stride_arg = DeclareLaunchArgument(
        'sector_stride',
        default_value=sector_stride,
        description='Embedded sector filter stride value'
    )
    sector_use_voxel_arg = DeclareLaunchArgument(
        'sector_use_voxel',
        default_value=sector_use_voxel,
        description='Embedded sector filter voxel downsample on/off'
    )
    sector_voxel_leaf_arg = DeclareLaunchArgument(
        'sector_voxel_leaf',
        default_value=sector_voxel_leaf,
        description='Embedded sector filter voxel leaf size'
    )
    sector_adaptive_recovery_arg = DeclareLaunchArgument(
        'sector_adaptive_recovery',
        default_value=sector_adaptive_recovery,
        description='Embedded sector filter adaptive full-view recovery on/off'
    )
    sector_auto_disable_on_front_block_arg = DeclareLaunchArgument(
        'sector_auto_disable_on_front_block',
        default_value=sector_auto_disable_on_front_block,
        description='Trigger full-view when front is blocked by obstacles'
    )
    sector_full_view_hold_sec_arg = DeclareLaunchArgument(
        'sector_full_view_hold_sec',
        default_value=sector_full_view_hold_sec,
        description='How long (sec) to hold full-view after recovery trigger'
    )
    virtual_ceil_height_arg = DeclareLaunchArgument(
        'virtual_ceil_height',
        default_value=virtual_ceil_height,
        description='Virtual ceiling height [m] (set -0.1 to disable)'
    )
    z_max_arg = DeclareLaunchArgument(
        'z_max',
        default_value=z_max,
        description='Max flight altitude [m] applied at FSM and traj_server (set -1 to disable)'
    )
    optimization_dist0_arg = DeclareLaunchArgument(
        'optimization_dist0',
        default_value=optimization_dist0,
        description='Optimizer clearance distance (smaller = narrower passages allowed)'
    )
    optimization_lambda_collision_arg = DeclareLaunchArgument(
        'optimization_lambda_collision',
        default_value=optimization_lambda_collision,
        description='Collision avoidance cost weight (higher = safer, may sacrifice smoothness)'
    )
    optimization_lambda_smooth_arg = DeclareLaunchArgument(
        'optimization_lambda_smooth',
        default_value=optimization_lambda_smooth,
        description='Smoothness cost weight'
    )
    optimization_lambda_feasibility_arg = DeclareLaunchArgument(
        'optimization_lambda_feasibility',
        default_value=optimization_lambda_feasibility,
        description='Feasibility (vel/acc limit) cost weight'
    )
    local_update_range_x_arg = DeclareLaunchArgument(
        'local_update_range_x', default_value=local_update_range_x,
        description='Grid map local update range in X [m]'
    )
    local_update_range_y_arg = DeclareLaunchArgument(
        'local_update_range_y', default_value=local_update_range_y,
        description='Grid map local update range in Y [m]'
    )
    local_update_range_z_arg = DeclareLaunchArgument(
        'local_update_range_z', default_value=local_update_range_z,
        description='Grid map local update range in Z [m]'
    )
    thresh_replan_time_arg = DeclareLaunchArgument(
        'thresh_replan_time', default_value=thresh_replan_time,
        description='FSM replan time threshold [s]'
    )
    thresh_no_replan_meter_arg = DeclareLaunchArgument(
        'thresh_no_replan_meter', default_value=thresh_no_replan_meter,
        description='FSM no-replan distance threshold [m]'
    )
    cx_arg = DeclareLaunchArgument('cx', default_value=cx, description='Camera intrinsic cx')
    cy_arg = DeclareLaunchArgument('cy', default_value=cy, description='Camera intrinsic cy')
    fx_arg = DeclareLaunchArgument('fx', default_value=fx, description='Camera intrinsic fx')
    fy_arg = DeclareLaunchArgument('fy', default_value=fy, description='Camera intrinsic fy')
    max_vel_arg = DeclareLaunchArgument('max_vel', default_value=max_vel, description='Maximum velocity')
    max_acc_arg = DeclareLaunchArgument('max_acc', default_value=max_acc, description='Maximum acceleration')
    planning_horizon_arg = DeclareLaunchArgument('planning_horizon', default_value=planning_horizon, description='Planning horizon')

    point_num_arg = DeclareLaunchArgument('point_num', default_value=point_num, description='Number of waypoints')
    point0_x_arg = DeclareLaunchArgument('point0_x', default_value=point0_x, description='Waypoint 0 X coordinate')
    point0_y_arg = DeclareLaunchArgument('point0_y', default_value=point0_y, description='Waypoint 0 Y coordinate')
    point0_z_arg = DeclareLaunchArgument('point0_z', default_value=point0_z, description='Waypoint 0 Z coordinate')
    point1_x_arg = DeclareLaunchArgument('point1_x', default_value=point1_x, description='Waypoint 1 X coordinate')
    point1_y_arg = DeclareLaunchArgument('point1_y', default_value=point1_y, description='Waypoint 1 Y coordinate')
    point1_z_arg = DeclareLaunchArgument('point1_z', default_value=point1_z, description='Waypoint 1 Z coordinate')
    point2_x_arg = DeclareLaunchArgument('point2_x', default_value=point2_x, description='Waypoint 2 X coordinate')
    point2_y_arg = DeclareLaunchArgument('point2_y', default_value=point2_y, description='Waypoint 2 Y coordinate')
    point2_z_arg = DeclareLaunchArgument('point2_z', default_value=point2_z, description='Waypoint 2 Z coordinate')
    point3_x_arg = DeclareLaunchArgument('point3_x', default_value=point3_x, description='Waypoint 3 X coordinate')
    point3_y_arg = DeclareLaunchArgument('point3_y', default_value=point3_y, description='Waypoint 3 Y coordinate')
    point3_z_arg = DeclareLaunchArgument('point3_z', default_value=point3_z, description='Waypoint 3 Z coordinate')
    point4_x_arg = DeclareLaunchArgument('point4_x', default_value=point4_x, description='Waypoint 4 X coordinate')
    point4_y_arg = DeclareLaunchArgument('point4_y', default_value=point4_y, description='Waypoint 4 Y coordinate')
    point4_z_arg = DeclareLaunchArgument('point4_z', default_value=point4_z, description='Waypoint 4 Z coordinate')
    
    flight_type_arg = DeclareLaunchArgument('flight_type', default_value=flight_type, description='flight_type')
    use_distinctive_trajs_arg = DeclareLaunchArgument('use_distinctive_trajs', default_value=use_distinctive_trajs, description='Use distinctive trajectories')
    obj_num_set_arg = DeclareLaunchArgument('obj_num_set', default_value=obj_num_set, description='Number of objects')
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation time')

    # Ego Planner Node
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name=['_ego_planner_node'],
        respawn=True,
        respawn_delay=2.0,
        output='screen',
        remappings=[
            ('odom_world', [odometry_topic]),
            ('planning/bspline', ['planning/bspline']),
            ('planning/data_display', ['planning/data_display']),
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),
            
            ('goal_point', ['plan_vis/goal_point']),
            ('global_list', ['plan_vis/global_list']),
            ('init_list', ['plan_vis/init_list']),
            ('optimal_list', ['plan_vis/optimal_list']),
            ('a_star_list', ['plan_vis/a_star_list']),
            
            ('grid_map/odom', [odometry_topic]),
            ('grid_map/cloud', [cloud_topic]),
            ('grid_map/pose', [camera_pose_topic]),
            ('grid_map/depth', [depth_topic]),
            ('grid_map/occupancy_inflate', [grid_occ_topic])
        ],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'fsm/flight_type': flight_type},
            {'fsm/thresh_replan_time': thresh_replan_time},
            {'fsm/thresh_no_replan_meter': thresh_no_replan_meter},
            {'fsm/planning_horizon': planning_horizon},
            {'fsm/planning_horizen_time': 3.0},
            {'fsm/emergency_time': 1.0},
            {'fsm/realworld_experiment': True},
            {'fsm/fail_safe': True},
            {'fsm/z_max': z_max},
            
            {'fsm/waypoint_num': point_num},
            {'fsm/waypoint0_x': point0_x},
            {'fsm/waypoint0_y': point0_y},
            {'fsm/waypoint0_z': point0_z},
            {'fsm/waypoint1_x': point1_x},
            {'fsm/waypoint1_y': point1_y},
            {'fsm/waypoint1_z': point1_z},
            {'fsm/waypoint2_x': point2_x},
            {'fsm/waypoint2_y': point2_y},
            {'fsm/waypoint2_z': point2_z},
            {'fsm/waypoint3_x': point3_x},
            {'fsm/waypoint3_y': point3_y},
            {'fsm/waypoint3_z': point3_z},
            {'fsm/waypoint4_x': point4_x},
            {'fsm/waypoint4_y': point4_y},
            {'fsm/waypoint4_z': point4_z},
            
            {'grid_map/resolution': grid_resolution},
            {'grid_map/map_size_x': map_size_x},
            {'grid_map/map_size_y': map_size_y},
            {'grid_map/map_size_z': map_size_z},
            {'grid_map/local_update_range_x': local_update_range_x},
            {'grid_map/local_update_range_y': local_update_range_y},
            {'grid_map/local_update_range_z': local_update_range_z},
            {'grid_map/obstacles_inflation': grid_obstacles_inflation},
            {'grid_map/local_map_margin': 10},
            {'grid_map/ground_height': 0.1},
            # camera parameter
            {'grid_map/cx': cx},
            {'grid_map/cy': cy},
            {'grid_map/fx': fx},
            {'grid_map/fy': fy},
            # depth filter
            {'grid_map/use_depth_filter': True},
            {'grid_map/depth_filter_tolerance': 0.15},
            {'grid_map/depth_filter_maxdist': 5.0},
            {'grid_map/depth_filter_mindist': 0.2},
            {'grid_map/depth_filter_margin': 2},
            {'grid_map/k_depth_scaling_factor': 1000.0},
            {'grid_map/skip_pixel': 2},
            # local fusion
            {'grid_map/p_hit': 0.65},
            {'grid_map/p_miss': 0.35},
            {'grid_map/p_min': 0.12},
            {'grid_map/p_max': 0.90},
            {'grid_map/p_occ': 0.80},
            {'grid_map/min_ray_length': grid_min_ray_length},
            {'grid_map/max_ray_length': 5.0},
            {'grid_map/sector_filter_enable': sector_filter_enable},
            {'grid_map/sector_min_angle_deg': sector_min_angle_deg},
            {'grid_map/sector_max_angle_deg': sector_max_angle_deg},
            {'grid_map/sector_use_stride': sector_use_stride},
            {'grid_map/sector_stride': sector_stride},
            {'grid_map/sector_use_voxel': sector_use_voxel},
            {'grid_map/sector_voxel_leaf': sector_voxel_leaf},
            {'grid_map/sector_adaptive_recovery': sector_adaptive_recovery},
            {'grid_map/sector_auto_disable_on_front_block': sector_auto_disable_on_front_block},
            {'grid_map/sector_full_view_hold_sec': sector_full_view_hold_sec},
            
            {'grid_map/virtual_ceil_height': virtual_ceil_height},
            {'grid_map/visualization_truncate_height': 1.8},
            {'grid_map/show_occ_time': False},
            {'grid_map/pose_type': grid_pose_type},
            {'grid_map/frame_id': grid_frame_id},
            # planner manager
            {'manager/max_vel': max_vel},
            {'manager/max_acc': max_acc},
            {'manager/max_jerk': 4.0},
            {'manager/control_points_distance': 0.4},
            {'manager/feasibility_tolerance': 0.05},
            {'manager/planning_horizon': planning_horizon},
            {'manager/use_distinctive_trajs': use_distinctive_trajs},
            {'manager/drone_id': drone_id},
            # Trajectory optimization parameters
            {'optimization/lambda_smooth': optimization_lambda_smooth},
            {'optimization/lambda_collision': optimization_lambda_collision},
            {'optimization/lambda_feasibility': optimization_lambda_feasibility},
            {'optimization/lambda_fitness': 1.0},
            {'optimization/dist0': optimization_dist0},
            {'optimization/swarm_clearance': 0.5},
            {'optimization/max_vel': max_vel},
            {'optimization/max_acc': max_acc},

            # B-Spline parameters
            {'bspline/limit_vel': max_vel},
            {'bspline/limit_acc': max_acc},
            {'bspline/limit_ratio': 1.1},

            # Object prediction parameters
            {'prediction/obj_num': obj_num_set},
            {'prediction/lambda': 1.0},
            {'prediction/predict_rate': 1.0}
        ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()

    # Add LaunchArguments
    ld.add_action(map_size_x_arg)
    ld.add_action(map_size_y_arg)
    ld.add_action(map_size_z_arg)
    ld.add_action(odometry_topic_arg)
    ld.add_action(camera_pose_topic_arg)
    ld.add_action(depth_topic_arg)
    ld.add_action(cloud_topic_arg)
    ld.add_action(grid_occ_topic_arg)
    ld.add_action(grid_pose_type_arg)
    ld.add_action(grid_frame_id_arg)
    ld.add_action(grid_obstacles_inflation_arg)
    ld.add_action(grid_min_ray_length_arg)
    ld.add_action(grid_resolution_arg)
    ld.add_action(sector_filter_enable_arg)
    ld.add_action(sector_min_angle_deg_arg)
    ld.add_action(sector_max_angle_deg_arg)
    ld.add_action(sector_use_stride_arg)
    ld.add_action(sector_stride_arg)
    ld.add_action(sector_use_voxel_arg)
    ld.add_action(sector_voxel_leaf_arg)
    ld.add_action(sector_adaptive_recovery_arg)
    ld.add_action(sector_auto_disable_on_front_block_arg)
    ld.add_action(sector_full_view_hold_sec_arg)
    ld.add_action(virtual_ceil_height_arg)
    ld.add_action(z_max_arg)
    ld.add_action(optimization_dist0_arg)
    ld.add_action(optimization_lambda_collision_arg)
    ld.add_action(optimization_lambda_smooth_arg)
    ld.add_action(optimization_lambda_feasibility_arg)
    ld.add_action(local_update_range_x_arg)
    ld.add_action(local_update_range_y_arg)
    ld.add_action(local_update_range_z_arg)
    ld.add_action(thresh_replan_time_arg)
    ld.add_action(thresh_no_replan_meter_arg)
    ld.add_action(cx_arg)
    ld.add_action(cy_arg)
    ld.add_action(fx_arg)
    ld.add_action(fy_arg)
    ld.add_action(max_vel_arg)
    ld.add_action(max_acc_arg)
    ld.add_action(planning_horizon_arg)

    ld.add_action(point_num_arg)
    ld.add_action(point0_x_arg)
    ld.add_action(point0_y_arg)
    ld.add_action(point0_z_arg)
    ld.add_action(point1_x_arg)
    ld.add_action(point1_y_arg)
    ld.add_action(point1_z_arg)
    ld.add_action(point2_x_arg)
    ld.add_action(point2_y_arg)
    ld.add_action(point2_z_arg)
    ld.add_action(point3_x_arg)
    ld.add_action(point3_y_arg)
    ld.add_action(point3_z_arg)
    ld.add_action(point4_x_arg)
    ld.add_action(point4_y_arg)
    ld.add_action(point4_z_arg)
    
    ld.add_action(flight_type_arg)
    ld.add_action(use_distinctive_trajs_arg)
    ld.add_action(obj_num_set_arg)
    ld.add_action(drone_id_arg)
    ld.add_action(use_sim_time_arg)


    # Add Node
    ld.add_action(ego_planner_node)

    return ld
