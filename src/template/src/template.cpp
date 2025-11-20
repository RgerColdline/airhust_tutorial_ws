#include <template.h>

// 全局变量定义
int mission_num = 0;



// add target publisher
ros::Publisher target_pub;


void print_param()
{
  std::cout << "=== 控制参数 ===" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
  std::cout << "if_debug: " << if_debug << std::endl;
  if(if_debug == 1) cout << "自动offboard" << std::endl;
  else cout << "遥控器offboard" << std::endl;
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


  // 发布无人机多维控制话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

  // create target publisher
  target_pub = nh.advertise<std_msgs::String>("/target", 10);

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
  if (choice != 1) return 0;
  ros::spinOnce();
  rate.sleep();
  
  // 等待连接到飞控
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  //设置无人机的期望位置
 
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
  std::cout<<"ok"<<std::endl;

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
      if(if_debug == 1)
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
    
    switch (mission_num)
    {
      // mission1: 起飞并悬停10秒（满足任务要求）
      case 1:
        if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
        {
          static ros::Time hover_start_time = ros::Time::now();
          if (ros::Time::now() - hover_start_time > ros::Duration(10.0)) {
            mission_num = 2;
            last_request = ros::Time::now();
          }
        }
	    else if(ros::Time::now() - last_request >= ros::Duration(3.0))
        {
          mission_num = 2;
          last_request = ros::Time::now();
        }
        break;

      // mission2: 世界系前进（示例路径点）
      case 2:
        if (mission_pos_cruise(1.0, 0.0, ALTITUDE, 0.0 , err_max))
        {
          mission_num = 3;
          last_request = ros::Time::now();
        }
        break;

      // mission3: 识别任务区域巡航
      case 3:
        {
          static bool targets_recognized = false;
          static ros::Time search_start_time = ros::Time::now();
          
          // 在识别区域巡航搜索
          if (mission_pos_cruise(2.0, 0.0, ALTITUDE, 0.0, err_max) || 
              mission_pos_cruise(2.0, 1.0, ALTITUDE, 0.0, err_max) ||
              mission_pos_cruise(1.0, 1.0, ALTITUDE, 0.0, err_max))
          {
            // 尝试识别目标
            if (down_camera_updated) {
              std::vector<std::string> contents;
              std::vector<geometry_msgs::Point> positions;
              
              if (recognizeTargets(contents, positions)) {
                // 发布识别结果
                for (size_t i = 0; i < contents.size(); i++) {
                  std_msgs::String target_msg;
                  target_msg.data = contents[i] + " at (" + 
                                   std::to_string(positions[i].x) + ", " +
                                   std::to_string(positions[i].y) + ", " +
                                   std::to_string(positions[i].z) + ")";
                  target_pub.publish(target_msg);
                  ROS_INFO("识别到目标: %s", target_msg.data.c_str());
                }
                targets_recognized = true;
              }
            }
            
            if (targets_recognized || ros::Time::now() - search_start_time > ros::Duration(30.0)) {
              mission_num = 4;
              last_request = ros::Time::now();
            }
          }
        }
        break;

      // mission4: 基于颜色识别的精准降落
      case 4:
        if (precision_land_with_color_detection())
        {
          mission_num = -1; // 任务结束
          last_request = ros::Time::now();
        }
        break;
    }
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
    
    if(mission_num == -1) 
    {
      ROS_INFO("所有任务完成！");
      exit(0);
    }
  }
  return 0;
}