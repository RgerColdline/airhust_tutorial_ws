#include <template.h>

// 全局变量定义
int mission_num = 0;

// add target publisher
ros::Publisher target_pub;
ros::Publisher obstacle_pub; // 障碍物信息发布

void print_param()
{
  std::cout << "=== 控制参数 ===" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
  std::cout << "if_debug: " << if_debug << std::endl;
  std::cout << "安全距离: " << SAFE_DISTANCE << "米" << std::endl;
  std::cout << "避障距离: " << OBSTACLE_AVOID_DISTANCE << "米" << std::endl;
  if (if_debug == 1)
    cout << "自动offboard" << std::endl;
  else
    cout << "遥控器offboard" << std::endl;
}

int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  // 初始化ROS节点
  ros::init(argc, argv, "template");
  ros::NodeHandle nh;

  // 订阅mavros相关话题
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

  // 订阅下视摄像头图像话题
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber down_camera_sub = it.subscribe("camera/image_raw", 1, downCameraCallback);

  // 订阅雷达话题
  ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 1, laserCallback);

  // 发布无人机多维控制话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

  // 初始化publisher - 这是修复的关键部分
  target_pub = nh.advertise<std_msgs::String>("/target", 10);
  obstacle_pub = nh.advertise<std_msgs::String>("/obstacle_info", 10);

  // 创建服务客户端
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  // 参数读取

  nh.param<float>("err_max", err_max, 0);
  nh.param<float>("if_debug", if_debug, 0);
  print_param();

  int choice = 0;
  std::cout << "1 to go on , else to quit" << std::endl;
  std::cin >> choice;
  if (choice != 1)
    return 0;
  ros::spinOnce();
  rate.sleep();

  // 等待连接到飞控
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  // 设置无人机的期望位置

  setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
  setpoint_raw.coordinate_frame = 1;
  setpoint_raw.position.x = 0;
  setpoint_raw.position.y = 0;
  setpoint_raw.position.z = ALTITUDE;
  setpoint_raw.yaw = 0;

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "ok" << std::endl;

  // 定义客户端变量，设置为offboard模式
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // 定义客户端变量，请求无人机解锁
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  // 记录当前时间，并赋值给变量last_request
  ros::Time last_request = ros::Time::now();

  while (ros::ok())
  {
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
    {
      if (if_debug == 1)
      {
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
          ROS_INFO("Offboard enabled");
        }
      }
      else
      {
        ROS_INFO("Waiting for OFFBOARD mode");
      }
      last_request = ros::Time::now();
    }
    else
    {
      if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }

    // 持续发布设置点，确保无人机起飞
    mavros_setpoint_pos_pub.publish(setpoint_raw);

    // 当无人机到达起飞点高度后，悬停3秒后进入任务模式，提高视觉效果
    if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.2)
    {
      if (ros::Time::now() - last_request > ros::Duration(1.0))
      {
        mission_num = 1;
        last_request = ros::Time::now();
        break;
      }
    }

    mission_pos_cruise(0, 0, ALTITUDE, 0, err_max);
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }

  while (ros::ok())
  {
    ROS_WARN("mission_num = %d", mission_num);

    // 发布障碍物信息（用于调试）
    if (laser_updated && obstacle_detected)
    {
      std_msgs::String obstacle_msg;
      obstacle_msg.data = "检测到障碍物，距离: " +
                          std::to_string(*std::min_element(obstacle_distances.begin(), obstacle_distances.end())) +
                          "米";
      obstacle_pub.publish(obstacle_msg);
    }

    switch (mission_num)
    {
    // mission1: 起飞并悬停10秒（满足任务要求）
    case 1:
      if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
      {
        static ros::Time hover_start_time = ros::Time::now();
        if (ros::Time::now() - hover_start_time > ros::Duration(3.0))
        { // zan shi xiu gai wei 3s fang bian tiao shi
          mission_num = 2;
          last_request = ros::Time::now();
        }
      }
      else if (ros::Time::now() - last_request >= ros::Duration(3.0))
      {
        mission_num = 2;
        last_request = ros::Time::now();
      }
      break;

    // mission2: 避障巡航到目标点(17.0, 2.5)
    case 2:
    {
        // 目标以相对坐标表示（相对于起飞点）
        float target_x = 17.0f; // 相对坐标，按需修改
        float target_y = 2.5f;  // 相对坐标
        if (!isReached(15.0f, local_pos.pose.pose.position.y, ALTITUDE, err_max))
        {
          // 使用新的扇区选择算法计算中间避障点并朝该点巡航
          geometry_msgs::Point avoid_p = calculateAvoidancePoint(target_x, target_y, ALTITUDE);
          mission_pos_cruise(static_cast<float>(avoid_p.x), static_cast<float>(avoid_p.y), ALTITUDE, 0, err_max);
        }
        else
        {
          // 目标点到达后，直接切换到 case 4（跳过 case 3）
          mission_num = 4;
          last_request = ros::Time::now();
        }
      break;
    }

    // mission4: chuan huan
    case 4:
    {
      static bool circle_done = false;
      static bool square_done = false;
      err_max=.1f;
      static int circle_flag=0;
      switch (circle_flag)
      {
        case 0:
        mission_pos_cruise(18.0f, 2.4f, ALTITUDE, 0, err_max);
        if(isReached(18.0f,2.4f,ALTITUDE,err_max))
        {
          circle_flag=1;
        }
        break;
        case 1:

      MAX_STEP=0.3;
        mission_pos_cruise(20.0f, 2.4f, ALTITUDE, 0, err_max);
        if(isReached(20.0f,2.4f,ALTITUDE,err_max))
        {
          circle_flag=2;
        }
        break;
        case 2:
      MAX_STEP=0.7;
        mission_pos_cruise(21.0f, 0.2f, ALTITUDE, 0, err_max);
        if(isReached(21.0f,0.2f,ALTITUDE,err_max))
        {
          circle_flag=3;
        }
        break;
        case 3:
      MAX_STEP=0.5;
        mission_pos_cruise(23.0f, 0.2f, ALTITUDE, 0, err_max);
        if(isReached(23.0f,0.2f,ALTITUDE,err_max))
        {
          circle_flag=4;
        }
        break;
        case 4:
      MAX_STEP=0.7;
      err_max=0.2f;
       
      
      // if (!circle_done)
      // {
      //   float ring_center_x = 19.0f; // 如果你需要用 15.0 或其他，请修改为相应值
      //   float ring_center_y = 2.4f;
      //   float ring_alt = ALTITUDE;
      //   // 圆环：半径 1.0m ，点数 12
      //   if (fly_through_circle_ring(ring_center_x, ring_center_y, ring_alt, 1.0f, 12, err_max))
      //   {
      //     circle_done = true;
      //     last_request = ros::Time::now();
      //     ROS_INFO("圆环完成");
      //   }
      // }
      // else if (!square_done)
      // {
      //   float ring_center_x = 22.0f; // 使用 mission2 目标 x 或你实际需要的中心
      //   float ring_center_y = 0.2f;
      //   float ring_center_z = ALTITUDE; // 垂直方环中心高度
      //   float ring_alt = ALTITUDE;
      //   // 垂直方环：center_x不变，在 y-z 平面穿越
      //   if (fly_through_vertical_square_ring(ring_center_x, ring_center_y, ring_center_z, 2.2f, err_max))
      //   {
      //     square_done = true;
      //     last_request = ros::Time::now();
      //     ROS_INFO("垂直方环完成");
      //   }
      // }
      // else
      // {
        // 两个环都完成，结束任务或进入下一个流程
        mission_num = -1;
        last_request = ros::Time::now();
       }
    }
    break;
    }
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();

    if (mission_num == -1)
    {
      ROS_INFO("所有任务完成！");
      exit(0);
    }
  }
  return 0;
}