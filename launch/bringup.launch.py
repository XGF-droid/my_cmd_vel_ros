import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'my_cmd_vel_ros'


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory(PKG)
    default_params = os.path.join(pkg_share, 'config', 'my_cmd_vel_ros.yaml')

    # ---------------- launch 参数 ----------------
    params_file = LaunchConfiguration('params_file')
    start_publisher = LaunchConfiguration('start_publisher')
    start_subscriber = LaunchConfiguration('start_subscriber')
    start_monitor = LaunchConfiguration('start_monitor')

    linear_x = LaunchConfiguration('linear_x')
    linear_y = LaunchConfiguration('linear_y')
    angular_z = LaunchConfiguration('angular_z')
    rate = LaunchConfiguration('rate')
    accel_limit = LaunchConfiguration('accel_limit')
    duration = LaunchConfiguration('duration')

    stale_timeout = LaunchConfiguration('stale_timeout')
    status_rate = LaunchConfiguration('status_rate')
    monitor_topic = LaunchConfiguration('monitor_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 类型转换：ParameterValue(..., value_type=...) 会把字符串转成正确的参数类型
    def as_float(sub): return ParameterValue(sub, value_type=float)
    def as_bool(sub): return ParameterValue(sub, value_type=bool)

    # ---------------- 1) 速度指令发布节点 ----------------
    publisher = Node(
        package=PKG,
        executable='my_cmd_vel_ros_publisher',
        name='cmd_vel_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,
            {
                # 命令行传了就以命令行为准（写在 YAML 后面，后写的生效）
                'linear_x': as_float(linear_x),
                'linear_y': as_float(linear_y),
                'angular_z': as_float(angular_z),
                'rate': as_float(rate),
                'accel_limit': as_float(accel_limit),
                'duration': as_float(duration),
                'use_sim_time': as_bool(use_sim_time),
            },
        ],
        condition=IfCondition(start_publisher),
    )

    # ---------------- 2) 速度订阅节点（看门狗 + 钳制，转发 safe_cmd_vel）----------------
    subscriber = Node(
        package=PKG,
        executable='cmd_vel_subscriber',
        name='sub',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,
            {'use_sim_time': as_bool(use_sim_time)},
            # 注意：节点是 sub，YAML 里的键必须叫 sub:，不能用可执行文件名
        ],
        condition=IfCondition(start_subscriber),
    )

    # ---------------- 3) 状态上报节点 ----------------
    # monitor_topic 同时写进 cmd_vel_topic，让状态上报盯住"链路里真正有效的那一段"
   # monitor = Node(
   #     package=PKG,
    #    executable='robot_status_publisher',
     #   name='robot_status_publisher',
      #  output='screen',
       # emulate_tty=True,
       # parameters=[
       #     params_file,
       #     {
        #        'cmd_vel_topic': ParameterValue(monitor_topic, value_type=str),
        #        'stale_timeout': as_float(stale_timeout),
         #       'status_rate': as_float(status_rate),
         #       'use_sim_time': as_bool(use_sim_time),
         #   },
        #],
       # condition=IfCondition(start_monitor),
    #)

    return LaunchDescription([
        # ---- 参数声明 ----
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='参数 YAML 路径（三个节点的默认值都在这）'),
        DeclareLaunchArgument('start_publisher', default_value='true',
                              description='是否启动速度指令发布节点'),
        DeclareLaunchArgument('start_subscriber', default_value='true',
                              description='是否启动速度订阅节点'),
        DeclareLaunchArgument('start_monitor', default_value='true',
                              description='是否启动 robot_status 状态上报节点'),

        DeclareLaunchArgument('linear_x', default_value='0.3',
                              description='前进速度 m/s（发布节点）'),
        DeclareLaunchArgument('linear_y', default_value='0.0',
                              description='横向速度 m/s（麦轮/全向底盘）'),
        DeclareLaunchArgument('angular_z', default_value='0.5',
                              description='自转速度 rad/s（发布节点）'),
        DeclareLaunchArgument('rate', default_value='20.0',
                              description='发布频率 Hz（发布节点）'),
        DeclareLaunchArgument('accel_limit', default_value='1.0',
                              description='加速度上限，做斜率限制（发布节点）'),
        DeclareLaunchArgument('duration', default_value='0.0',
                              description='发布节点运行秒数，0=一直跑；设 8.0 可自动演示 E_STOP'),

        DeclareLaunchArgument('stale_timeout', default_value='0.5',
                              description='多久没收到速度判定失联（秒，状态上报节点）'),
        DeclareLaunchArgument('status_rate', default_value='10.0',
                              description='/robot_status 发布频率 Hz'),
        DeclareLaunchArgument('monitor_topic', default_value='/safe_cmd_vel',
                              description='状态上报盯哪个话题：/safe_cmd_vel（下发底盘）或 /cmd_vel（原始指令）'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='仿真时间（Gazebo / bag 回放设 true）'),

        # ---- 启动顺序 ----
        # 发布与订阅立即启动；状态上报延迟 1 秒，避免启动瞬间没数据先报一轮 INIT/E_STOP
        publisher,
        subscriber,
        #TimerAction(period=1.0, actions=[monitor]),
    ])