import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # --- Setup environment variables ---
    # 设置 Gazebo 资源路径环境变量
    MDL_ENV_VAR = "IGN_GAZEBO_RESOURCE_PATH"
    
    # 添加 auto_robot 包路径
    if MDL_ENV_VAR in os.environ:
        os.environ[MDL_ENV_VAR] += ":" + os.path.join(
            get_package_prefix("auto_robot"), "share"
        )
    else:
        os.environ[MDL_ENV_VAR] = os.path.join(
            get_package_prefix("auto_robot"), "share"
        )
    
    # 添加 z1_description 包路径
    if MDL_ENV_VAR in os.environ:
        os.environ[MDL_ENV_VAR] += ":" + os.path.join(
            get_package_prefix("z1_description"), "share"
        )
    else:
        os.environ[MDL_ENV_VAR] = os.path.join(
            get_package_prefix("z1_description"), "share"
        )

    # 设置 Gazebo 插件路径环境变量
    LIB_ENV_VAR = "IGN_GAZEBO_SYSTEM_PLUGIN_PATH"
    if LIB_ENV_VAR in os.environ:
        os.environ[LIB_ENV_VAR] += ":/opt/ros/humble/lib"
    else:
        os.environ[LIB_ENV_VAR] = "/opt/ros/humble/lib"

    # 打印环境变量用于调试
    print(f"设置 {MDL_ENV_VAR}: {os.environ.get(MDL_ENV_VAR, 'NOT SET')}")
    print(f"设置 {LIB_ENV_VAR}: {os.environ.get(LIB_ENV_VAR, 'NOT SET')}")

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='auto_robot' #<--- CHANGE ME
    urdf_tutorial_path = get_package_share_path('auto_robot')
    default_rviz_config_path = urdf_tutorial_path / 'rviz/urdf.rviz'

    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.5'
    spawn_yaw_val = '0.0'

    auto_robot = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','auto_robot.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #          )
    ignition_gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"
        ], ),
        launch_arguments={
            "gz_args": " -r -v 1 empty.sdf",
        }.items(),
        # condition=IfCondition(sim_ignition),
    )

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    # 启动关节状态广播器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 启动 Z1 位置控制器
    z1_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )

    chassis_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller", "--controller-manager", "/controller_manager"],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        # condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'auto_robot',
                   '-x', spawn_x_val,
                   '-y', spawn_y_val,
                   '-z', spawn_z_val,
                   '-Y', spawn_yaw_val],
        output='screen'
    )

    # 添加控制器管理器
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(get_package_share_directory(package_name), "config", "chassis_controllers.yaml"),
            os.path.join(get_package_share_directory(package_name), "config", "z1_controllers.yaml")
        ],
        output="screen",
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    rviz_delayed = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Launch them all!
    return LaunchDescription([
        rviz_arg,
        auto_robot,
        ignition_gazebo_node,  # 先启动 Gazebo
        spawn_entity,          # 然后生成机器人
        joint_state_broadcaster_spawner,
        z1_position_controller_spawner,
        # joint_state_publisher_gui_node,  # 简单的关节状态发布器
        # rviz_node,             # 直接启动 RViz，不用延迟
        rviz_delayed,
    ])
