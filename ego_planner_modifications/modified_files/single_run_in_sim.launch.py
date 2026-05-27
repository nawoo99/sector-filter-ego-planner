import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # 定义参数的 LaunchConfiguration
    obj_num = LaunchConfiguration('obj_num', default=10)
    drone_id = LaunchConfiguration('drone_id', default=0)
    
    map_size_x = LaunchConfiguration('map_size_x', default = 100.0)
    map_size_y = LaunchConfiguration('map_size_y', default = 50.0)
    map_size_z = LaunchConfiguration('map_size_z', default = 10.0)
    odom_topic = LaunchConfiguration('odom_topic', default = '/odometry')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='nouse1')
    depth_topic = LaunchConfiguration('depth_topic', default='nouse2')
    cloud_topic = LaunchConfiguration('cloud_topic', default='/points')
    grid_occ_topic = LaunchConfiguration('grid_occ_topic', default='grid/grid_map/occupancy_inflate')
    grid_pose_type = LaunchConfiguration('grid_pose_type', default=2)
    grid_frame_id = LaunchConfiguration('grid_frame_id', default='map')
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
    z_fixed = LaunchConfiguration('z_fixed', default=-1.0)
    optimization_dist0            = LaunchConfiguration('optimization_dist0',            default=0.32)
    optimization_lambda_collision  = LaunchConfiguration('optimization_lambda_collision',  default=2.0)
    optimization_lambda_smooth     = LaunchConfiguration('optimization_lambda_smooth',     default=1.0)
    optimization_lambda_feasibility= LaunchConfiguration('optimization_lambda_feasibility',default=0.1)
    planning_horizon      = LaunchConfiguration('planning_horizon',        default=7.5)
    local_update_range_x  = LaunchConfiguration('local_update_range_x',   default=8.0)
    local_update_range_y  = LaunchConfiguration('local_update_range_y',   default=8.0)
    local_update_range_z  = LaunchConfiguration('local_update_range_z',   default=4.5)
    thresh_replan_time    = LaunchConfiguration('thresh_replan_time',     default=1.0)
    thresh_no_replan_meter= LaunchConfiguration('thresh_no_replan_meter', default=1.0)
    
    
    # 声明全局参数
    obj_num_cmd = DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of objects')
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')
    use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation time')
    camera_pose_topic_cmd = DeclareLaunchArgument(
        'camera_pose_topic', default_value=camera_pose_topic, description='Camera pose topic'
    )
    depth_topic_cmd = DeclareLaunchArgument(
        'depth_topic', default_value=depth_topic, description='Depth image topic'
    )
    cloud_topic_cmd = DeclareLaunchArgument(
        'cloud_topic', default_value=cloud_topic, description='Point cloud topic'
    )
    grid_occ_topic_cmd = DeclareLaunchArgument(
        'grid_occ_topic', default_value=grid_occ_topic, description='Grid map visualize topic'
    )
    grid_pose_type_cmd = DeclareLaunchArgument(
        'grid_pose_type', default_value=grid_pose_type, description='grid_map pose type: 1=pose,2=odom'
    )
    grid_frame_id_cmd = DeclareLaunchArgument(
        'grid_frame_id', default_value=grid_frame_id, description='grid_map target frame id'
    )
    grid_obstacles_inflation_cmd = DeclareLaunchArgument(
        'grid_obstacles_inflation', default_value=grid_obstacles_inflation,
        description='Grid map obstacle inflation radius'
    )
    grid_min_ray_length_cmd = DeclareLaunchArgument(
        'grid_min_ray_length', default_value=grid_min_ray_length,
        description='Grid map minimum ray length'
    )
    grid_resolution_cmd = DeclareLaunchArgument(
        'grid_resolution', default_value=grid_resolution, description='Grid map voxel resolution'
    )
    sector_filter_enable_cmd = DeclareLaunchArgument(
        'sector_filter_enable', default_value=sector_filter_enable,
        description='Enable embedded sector filter inside ego_planner_node'
    )
    sector_min_angle_deg_cmd = DeclareLaunchArgument(
        'sector_min_angle_deg', default_value=sector_min_angle_deg,
        description='Embedded sector filter min angle in degree'
    )
    sector_max_angle_deg_cmd = DeclareLaunchArgument(
        'sector_max_angle_deg', default_value=sector_max_angle_deg,
        description='Embedded sector filter max angle in degree'
    )
    sector_use_stride_cmd = DeclareLaunchArgument(
        'sector_use_stride', default_value=sector_use_stride,
        description='Embedded sector filter stride sampling on/off'
    )
    sector_stride_cmd = DeclareLaunchArgument(
        'sector_stride', default_value=sector_stride,
        description='Embedded sector filter stride value'
    )
    sector_use_voxel_cmd = DeclareLaunchArgument(
        'sector_use_voxel', default_value=sector_use_voxel,
        description='Embedded sector filter voxel downsample on/off'
    )
    sector_voxel_leaf_cmd = DeclareLaunchArgument(
        'sector_voxel_leaf', default_value=sector_voxel_leaf,
        description='Embedded sector filter voxel leaf size'
    )
    sector_adaptive_recovery_cmd = DeclareLaunchArgument(
        'sector_adaptive_recovery', default_value=sector_adaptive_recovery,
        description='Embedded sector filter adaptive full-view recovery on/off'
    )
    sector_auto_disable_on_front_block_cmd = DeclareLaunchArgument(
        'sector_auto_disable_on_front_block', default_value=sector_auto_disable_on_front_block,
        description='Trigger full-view when front is blocked by obstacles'
    )
    sector_full_view_hold_sec_cmd = DeclareLaunchArgument(
        'sector_full_view_hold_sec', default_value=sector_full_view_hold_sec,
        description='How long (sec) to hold full-view after recovery trigger'
    )
    virtual_ceil_height_cmd = DeclareLaunchArgument(
        'virtual_ceil_height', default_value=virtual_ceil_height,
        description='Virtual ceiling height [m] (set -0.1 to disable)'
    )
    z_max_cmd = DeclareLaunchArgument(
        'z_max', default_value=z_max,
        description='Max flight altitude [m] applied at FSM and traj_server (set -1 to disable)'
    )
    z_fixed_cmd = DeclareLaunchArgument(
        'z_fixed', default_value=z_fixed,
        description='Fix traj_server Z output to this value [m] (set -1 to disable)'
    )
    optimization_dist0_cmd = DeclareLaunchArgument(
        'optimization_dist0', default_value=optimization_dist0,
        description='Optimizer clearance distance'
    )
    optimization_lambda_collision_cmd = DeclareLaunchArgument(
        'optimization_lambda_collision', default_value=optimization_lambda_collision,
        description='Collision avoidance cost weight'
    )
    optimization_lambda_smooth_cmd = DeclareLaunchArgument(
        'optimization_lambda_smooth', default_value=optimization_lambda_smooth,
        description='Smoothness cost weight'
    )
    optimization_lambda_feasibility_cmd = DeclareLaunchArgument(
        'optimization_lambda_feasibility', default_value=optimization_lambda_feasibility,
        description='Feasibility cost weight'
    )
    planning_horizon_cmd = DeclareLaunchArgument(
        'planning_horizon', default_value=planning_horizon,
        description='FSM/manager planning horizon [m]'
    )
    local_update_range_x_cmd = DeclareLaunchArgument(
        'local_update_range_x', default_value=local_update_range_x,
        description='Grid map local update range in X [m]'
    )
    local_update_range_y_cmd = DeclareLaunchArgument(
        'local_update_range_y', default_value=local_update_range_y,
        description='Grid map local update range in Y [m]'
    )
    local_update_range_z_cmd = DeclareLaunchArgument(
        'local_update_range_z', default_value=local_update_range_z,
        description='Grid map local update range in Z [m]'
    )
    thresh_replan_time_cmd = DeclareLaunchArgument(
        'thresh_replan_time', default_value=thresh_replan_time,
        description='FSM replan time threshold [s]'
    )
    thresh_no_replan_meter_cmd = DeclareLaunchArgument(
        'thresh_no_replan_meter', default_value=thresh_no_replan_meter,
        description='FSM no-replan distance threshold [m]'
    )
    # 地图属性以及是否使用动力学仿真
    use_mockamap = LaunchConfiguration('use_mockamap', default=False) # map_generator or mockamap 
    
    use_mockamap_cmd = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap, description='Choose map type, map_generator or mockamap')
    
    use_dynamic = LaunchConfiguration('use_dynamic', default=False)  
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')
    
    # Include advanced parameters
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'use_sim_time': use_sim_time,
            'obj_num_set': obj_num,
            
            'camera_pose_topic': camera_pose_topic,
            'depth_topic': depth_topic,
            'cloud_topic': cloud_topic,
            'grid_occ_topic': grid_occ_topic,
            'grid_pose_type': grid_pose_type,
            'grid_frame_id': grid_frame_id,
            'grid_obstacles_inflation': grid_obstacles_inflation,
            'grid_min_ray_length': grid_min_ray_length,
            'grid_resolution': grid_resolution,
            'sector_filter_enable': sector_filter_enable,
            'sector_min_angle_deg': sector_min_angle_deg,
            'sector_max_angle_deg': sector_max_angle_deg,
            'sector_use_stride': sector_use_stride,
            'sector_stride': sector_stride,
            'sector_use_voxel': sector_use_voxel,
            'sector_voxel_leaf': sector_voxel_leaf,
            'sector_adaptive_recovery': sector_adaptive_recovery,
            'sector_auto_disable_on_front_block': sector_auto_disable_on_front_block,
            'sector_full_view_hold_sec': sector_full_view_hold_sec,
            'virtual_ceil_height': virtual_ceil_height,
            'z_max': z_max,
            'optimization_dist0': optimization_dist0,
            'optimization_lambda_collision': optimization_lambda_collision,
            'optimization_lambda_smooth': optimization_lambda_smooth,
            'optimization_lambda_feasibility': optimization_lambda_feasibility,
            'local_update_range_x': local_update_range_x,
            'local_update_range_y': local_update_range_y,
            'local_update_range_z': local_update_range_z,
            'thresh_replan_time': thresh_replan_time,
            'thresh_no_replan_meter': thresh_no_replan_meter,
            'cx': str(428.8812561035156),
            'cy': str(238.36671447753906),
            'fx': str(428.8812561035156),
            'fy': str(428.86175537109375),
            'max_vel': str(1.5),
            'max_acc': str(3.0),
            'planning_horizon': planning_horizon,
            'use_distinctive_trajs': 'False',
            'flight_type': str(1),
            'point_num': str(4),
            'point0_x': str(15.0),
            'point0_y': str(0.0),
            'point0_z': str(1.0),
            
            'point1_x': str(-15.0),
            'point1_y': str(0.0),
            'point1_z': str(1.0),
            
            'point2_x': str(15.0),
            'point2_y': str(0.0),
            'point2_z': str(1.0),
            
            'point3_x': str(-15.0),
            'point3_y': str(0.0),
            'point3_z': str(1.0),
            
            'point4_x': str(15.0),
            'point4_y': str(0.0),
            'point4_z': str(1.0),
        }.items()
    )
    
    yaw_to_goal = LaunchConfiguration('yaw_to_goal', default='true')
    yaw_to_goal_cmd = DeclareLaunchArgument(
        'yaw_to_goal', default_value=yaw_to_goal,
        description='Face final goal instead of path tangent for yaw'
    )

    # Trajectory server node
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['traj_server'],
        respawn=True,
        respawn_delay=2.0,
        output='screen',
        remappings=[
            ('position_cmd', ['planning/pos_cmd']),
            ('planning/bspline', ['planning/bspline'])
        ],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'traj_server/time_forward': 1.0},
            {'traj_server/yaw_to_goal': yaw_to_goal},
            {'traj_server/z_max': z_max},
            {'traj_server/z_fixed': z_fixed}
        ]
    )
    
    ld = LaunchDescription()
        
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(use_sim_time_cmd)
    ld.add_action(camera_pose_topic_cmd)
    ld.add_action(depth_topic_cmd)
    ld.add_action(cloud_topic_cmd)
    ld.add_action(grid_occ_topic_cmd)
    ld.add_action(grid_pose_type_cmd)
    ld.add_action(grid_frame_id_cmd)
    ld.add_action(grid_obstacles_inflation_cmd)
    ld.add_action(grid_min_ray_length_cmd)
    ld.add_action(grid_resolution_cmd)
    ld.add_action(sector_filter_enable_cmd)
    ld.add_action(sector_min_angle_deg_cmd)
    ld.add_action(sector_max_angle_deg_cmd)
    ld.add_action(sector_use_stride_cmd)
    ld.add_action(sector_stride_cmd)
    ld.add_action(sector_use_voxel_cmd)
    ld.add_action(sector_voxel_leaf_cmd)
    ld.add_action(sector_adaptive_recovery_cmd)
    ld.add_action(sector_auto_disable_on_front_block_cmd)
    ld.add_action(sector_full_view_hold_sec_cmd)
    ld.add_action(virtual_ceil_height_cmd)
    ld.add_action(z_max_cmd)
    ld.add_action(z_fixed_cmd)
    ld.add_action(optimization_dist0_cmd)
    ld.add_action(optimization_lambda_collision_cmd)
    ld.add_action(optimization_lambda_smooth_cmd)
    ld.add_action(optimization_lambda_feasibility_cmd)
    ld.add_action(planning_horizon_cmd)
    ld.add_action(local_update_range_x_cmd)
    ld.add_action(local_update_range_y_cmd)
    ld.add_action(local_update_range_z_cmd)
    ld.add_action(thresh_replan_time_cmd)
    ld.add_action(thresh_no_replan_meter_cmd)
    ld.add_action(yaw_to_goal_cmd)
    ld.add_action(obj_num_cmd)
    ld.add_action(drone_id_cmd)
    ld.add_action(use_dynamic_cmd)
    # ld.add_action(use_mockamap_cmd)

    # 添加 Map Generator 节点
    # ld.add_action(map_generator_node)
    # ld.add_action(mockamap_node)
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    # ld.add_action(simulator_include)

    return ld
