# my_cmd_vel_ros(我的包名)
# cmd_vel  是一个速度话题，我认为外部系统：比如遥控器或者电脑键盘输入多个参数，即关于小车的速度，转速等参数会发布给topic（我理解的是外部系统把参数发布给topic，我理解的是传参，**不知道对不对**），然后sub（订阅者），也就是我写的subcriber.cpp这个节点，会收到topic传的参数，并进行处理。
# 系统结构：  外部系统给topic  cmd_vel发布参数指令，subcriber中的sub节点接受并处理，同时由subcribe中的发布节点safe_pub发给话题safe_cmd_vel  这是发布机器人的速度参数处理结果
#                                          然后由my_cmd_vel_ros_publisher中的cmd_sub订阅节点接受话题 cmd_vel的信息，，并进行安全处理，然后由发布节点satus_pub发布给话题robot_status。


# subcriber.cpp  节点   在这个程序中，我其实是写了两个节点，一个就是sub节点，它负责接收topic的参数，并对这些参数进行安全处理。
## sub节点的安全处理：  1.我设置了对非法参数的判断，即对0/0 未定式，正无穷和负无穷的判断，当topic传给sub节点这些值的时候，会直接将与小车速度有关的参数返回为0/
##                   2.我设置了对参数的处理，比如我设置了速度的最大变化量，这样在传入参数的时候，会先判断目标速度-默认速度与最大变化量的关系，进而让机器人的速度缓慢的变化，减速同理。
##                    3.我还设置了极小值为0的代码，如果参数过小，就不会改变底盘的速度参数，这样底盘就不会不停的抖动了。
##                    4.最后我还有个“看门狗”，他可以在一段时间没有接受到topic的指令后，自动将速度调为0,进而防止机器人一直行动

## subcriber.cpp中的safe_pub 节点，如果sub接受到的参数合法，在sub进行处理后，就会通过safe_pub 发布给另一个话题topic （safe_cmd_vel），**这就是发布机器人的速度参数的处理结果**




## my_cmd_vel_ros_publisher.cpp  在这个程序中，我写了一订阅节点（cmd_sub），它可以接受topiccmd_vel的参数并进行处理，方式和上面的订阅节点差不多
##                               同时，我还写了一个发布节点（status_pub），他会判断参数是否合法，并将参数信息发布给另一话题robot_status。

# topic cmd_vel   输入：外部系统   输出:subcriber中的sub节点     和    my_cmd_vel_ros_publisher的cmd_sub节点
# topic safe_cmd_vel  输入：safe_pub 节点
# topic robot_status  输入：status_pub 节点

# 关键参数：subccriber.cpp  :topic_ = declare_parameter<std::string>("topic", "/cmd_vel");
    -out_topic_ = declare_parameter<std::string>("out_topic", "safe_cmd_vel");
    -timeout_ = declare_parameter<double>("timeout", 0.5);          // 看门狗超时（秒），<=0 关闭
    -max_linear_ = declare_parameter<double>("max_linear", 1.0);    // 线速度上限 m/s
    -max_angular_ = declare_parameter<double>("max_angular", 2.0);  // 角速度上限 rad/s
    -verbose_ = declare_parameter<bool>("verbose", true);           // 是否每次打印收到的速度
    -deadband_ = declare_parameter<double>("deadband", 0.0);        // 小于该值视为 0，抑制抖动
    
    
    -std::string topic_;
    -std::string out_topic_;
    
    
    - double timeout_ = 0.5;
    -double max_linear_ = 1.0;
    -double max_angular_ = 2.0;
    -double deadband_ = 0.0;
    -bool verbose_ = true;
    -bool valid_ = true;

    -std::chrono::steady_clock::time_point last_msg_time_;
    -bool started_ = false;
    -bool stopped_by_watchdog_ = false;

    // 统计
    -size_t count_ = 0;
    -size_t clamped_count_ = 0;
    -size_t invalid_count_ = 0;
    -size_t timeout_count_ = 0;
#  my_cmd_vel_ros_publisher.cpp:  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    -status_topic_ = declare_parameter<std::string>("status_topic", "/robot_status");
    -status_rate_ = declare_parameter<double>("status_rate", 10.0);       // 状态发布频率 Hz
    -stale_timeout_ = declare_parameter<double>("stale_timeout", 0.5);    // 多久没指令算失联 s
    -stopped_speed_ = declare_parameter<double>("stopped_speed", 0.01);   // 小于此速度视为静止
    -deadband_ = declare_parameter<double>("deadband", 0.0);              // 死区，0 表示不启用
    -frame_id_ = declare_parameter<std::string>("frame_id", "base_link");  // 状态帧坐标系
    -verbose_ = declare_parameter<bool>("verbose", false);                // 是否打印状态切换日志
    -publish_before_first_cmd_ =
    - declare_parameter<bool>("publish_before_first_cmd", true);       // 未收到指令时是否也发状态
    -std::string cmd_vel_topic_, status_topic_, frame_id_;
    -double status_rate_ = 10.0;
    -double stale_timeout_ = 0.5;
    -double stopped_speed_ = 0.01;
    -double deadband_ = 0.0;
    -bool verbose_ = false;
    -bool publish_before_first_cmd_ = true;
    -bool valid_ = true;

    -double applied_rate_hz_ = 10.0;  // 已生效频率，用于判断是否需要重建定时器
    -bool rate_changed_ = false;

  // 最近一次指令与状态
    -double cmd_vx_ = 0.0, cmd_vy_ = 0.0, cmd_wz_ = 0.0;
    -double linear_speed_ = 0.0, angular_speed_ = 0.0;
    -bool ever_received_ = false;
    -bool online_ = false;
    -bool logged_first_ = false;
    -const char* motion_state_ = kStateInit;  // 指向常量字符串，无需管理内存

  // 计时（看门狗用单调时钟）
    -std::chrono::steady_clock::time_point last_cmd_time_;

  // 统计
    -uint64_t cmd_count_ = 0;
    -uint64_t timeout_count_ = 0;
    -uint64_t stop_count_ = 0;
    -uint64_t invalid_count_ = 0;
# 编译:cd ~/ros2_ws
#     colcon build  --packages-select my_cmd_vel_ros
#     source install/setup.bash
# 运行
## 启动节点：ros2 launch my_cmd_vel_ros bringup.launch.py
## 测试发送指令：ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
# 验证系统功能：1.外部指令正常转发功能：# 向原始话题发送线速度 0.3m/s
##             ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"   
##             ros2 topic echo /safe_cmd_vel
##             ros2 topic echo /system_status
#             2.看门狗验证： 在上一步之后按下ctrl+c ;
#             3外部系统恢复后，看门狗停止：ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
#             4可视化验证节点连接关系：rqt_graph
#              5查看话题频率：ros2 topic hz /safe_cmd_vel


## AI使用情况：1.两个发布点，两个订阅点，是由AI写的，使用AI生成了CMAKE包和xml.包 launch配置文件也是AI写的然后编译和运行的一些命令和上传Git的命令询问了AI，向AI询问了录屏和rosbag的用法，然后在AI写完代码后，逐行的看了代码，大致了解每这些代码是为了干什么，但是一些C++的写法不太了解，询问AI，然后自行创作README时都是自己写的，除了编译运行的指令和检查系统功能的指令
