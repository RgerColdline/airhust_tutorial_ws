#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/PositionTarget.h>
#include <algorithm>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

// files about image processing
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// 雷达相关头文件
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>

using namespace std;

#define ALTITUDE 1.5f
#define SAFE_DISTANCE 1.0f
#define OBSTACLE_AVOID_DISTANCE 1.5f  // 缩短避障触发距离，适应狭小场地
// 最大单步移动距离（相对坐标，单位：米）
#define MAX_STEP 0.3f     // 水平每次最多移动 0.3m，可根据场地再调小
#define MAX_Z_STEP 0.2f   // 垂直每次最多移动 0.2m

float err_max = 0.2;
float if_debug = 0;

mavros_msgs::PositionTarget setpoint_raw;

/************************************************************************
函数 1：无人机状态回调函数
*************************************************************************/
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

/************************************************************************
函数 2：回调函数接收无人机的里程计信息
从里程计信息中提取无人机的位置信息和姿态信息
*************************************************************************/
tf::Quaternion quat;
nav_msgs::Odometry local_pos;
double roll, pitch, yaw;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    if (flag_init_position == false && (local_pos.pose.pose.position.z != 0))
    {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        init_yaw_take_off = yaw;
        flag_init_position = true;
    }
}

/************************************************************************
函数 3：下视摄像头图像回调函数
*************************************************************************/
cv::Mat down_camera_image;
bool down_camera_updated = false;
void downCameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        down_camera_image = cv_ptr->image;
        down_camera_updated = true;

        // 可选：显示图像（调试用）
        if (if_debug == 1)
        {
            cv::imshow("Downward Camera", down_camera_image);
            cv::waitKey(1);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

/************************************************************************
函数 3.4：雷达数据回调函数和处理
*************************************************************************/
sensor_msgs::LaserScan laser_scan;
bool laser_updated = false;
vector<float> obstacle_distances; // 存储各个方向的障碍物距离
bool obstacle_detected = false;
float obstacle_direction = 0.0; // 障碍物相对于无人机前进方向的角度

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser_scan = *msg;
    laser_updated = true;

    // 处理雷达数据，检测障碍物
    obstacle_detected = false;
    float min_distance = 100.0; // 初始化一个较大的值
    int min_index = -1;

    // 分析雷达数据，检测前方障碍物
    int num_readings = laser_scan.ranges.size();
    obstacle_distances.clear();
    obstacle_distances.resize(num_readings);

    for (int i = 0; i < num_readings; ++i)
    {
        float distance = laser_scan.ranges[i];
        obstacle_distances[i] = distance;

        // 检查是否在有效范围内且小于安全距离
        if (!std::isinf(distance) && !std::isnan(distance) &&
            distance > laser_scan.range_min && distance < laser_scan.range_max)
        {

            if (distance < OBSTACLE_AVOID_DISTANCE && distance < min_distance)
            {
                min_distance = distance;
                min_index = i;
                obstacle_detected = true;
            }
        }
    }

    if (obstacle_detected && min_index != -1)
    {
        // 计算障碍物相对于无人机前进方向的角度
        float angle = laser_scan.angle_min + min_index * laser_scan.angle_increment;
        obstacle_direction = angle;

        if (if_debug == 1)
        {
            ROS_INFO("检测到障碍物! 距离: %.2f米, 方向: %.2f弧度", min_distance, angle);
        }
    }
}

/************************************************************************
函数 3.5：避障决策函数
根据雷达数据决定避障方向
*************************************************************************/
geometry_msgs::Point calculateAvoidancePoint(float target_x, float target_y, float target_z)
{
    // 说明：函数输入/输出均使用相对坐标（以起飞点为原点）
    geometry_msgs::Point avoid_point;

    // 当前相对位置（world -> relative）
    double current_rel_x = local_pos.pose.pose.position.x - init_position_x_take_off;
    double current_rel_y = local_pos.pose.pose.position.y - init_position_y_take_off;
    avoid_point.z = target_z; // 高度保持为传入的相对高度

    // 初始化为当前相对位置
    avoid_point.x = current_rel_x;
    avoid_point.y = current_rel_y;

    if (!obstacle_detected)
    {
        // 没有障碍物，直接返回目标相对点
        avoid_point.x = target_x;
        avoid_point.y = target_y;
        return avoid_point;
    }

    // 以相对坐标计算到目标点的方向
    double dx = static_cast<double>(target_x) - current_rel_x;
    double dy = static_cast<double>(target_y) - current_rel_y;
    double target_direction = atan2(dy, dx);

    // 计算避障方向：优先小幅侧移（45度）而非90度大转向
    double side = (obstacle_direction >= 0.0f) ? 1.0 : -1.0; // 根据障碍相对角度选择侧向
    double avoid_direction = target_direction + side * (M_PI / 4.0); // 45度侧移

    // 计算较小的避障点（侧移距离减小）
    double avoid_distance = 0.6; // 小幅侧移，适合狭小空间
    avoid_point.x = current_rel_x + avoid_distance * cos(avoid_direction);
    avoid_point.y = current_rel_y + avoid_distance * sin(avoid_direction);

    // 检查避障方向是否安全（使用 laser_scan 中的角度）
    bool safe_direction = true;
    for (int i = 0; i < obstacle_distances.size(); ++i)
    {
        if (!std::isinf(obstacle_distances[i]) && !std::isnan(obstacle_distances[i]))
        {
            float check_angle = laser_scan.angle_min + i * laser_scan.angle_increment;
            double angle_diff = fabs(check_angle - avoid_direction);
            // 将角度阈值收紧为30度，避免因宽阈值误判触发大幅避让
            if (angle_diff < M_PI / 6.0 && obstacle_distances[i] < SAFE_DISTANCE)
            {
                safe_direction = false;
                break;
            }
        }
    }

    if (!safe_direction)
    {
        // 如果首选侧向不安全，尝试另一侧（同样小幅侧移）
        avoid_direction = target_direction - side * (M_PI / 4.0);
        avoid_point.x = current_rel_x + avoid_distance * cos(avoid_direction);
        avoid_point.y = current_rel_y + avoid_distance * sin(avoid_direction);

        if (if_debug == 1)
        {
            ROS_INFO("首选侧向不安全，尝试另一侧避障 (小幅 45°)");
        }
    }

    return avoid_point;
}

/************************************************************************
函数 4: 颜色识别函数 - 用于降落任务
识别靶标颜色并返回识别结果
*************************************************************************/
std::string recognizeColor(const cv::Mat &image)
{
    if (image.empty())
    {
        return "unknown";
    }

    // 转换为HSV颜色空间，便于颜色识别
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // 定义颜色范围（HSV空间）
    // 红色范围
    cv::Scalar red_lower1(0, 120, 70);
    cv::Scalar red_upper1(10, 255, 255);
    cv::Scalar red_lower2(170, 120, 70);
    cv::Scalar red_upper2(180, 255, 255);

    // 绿色范围
    cv::Scalar green_lower(35, 120, 70);
    cv::Scalar green_upper(85, 255, 255);

    // 蓝色范围
    cv::Scalar blue_lower(100, 120, 70);
    cv::Scalar blue_upper(130, 255, 255);

    // 创建各颜色的掩膜
    cv::Mat red_mask1, red_mask2, red_mask, green_mask, blue_mask;
    cv::inRange(hsv, red_lower1, red_upper1, red_mask1);
    cv::inRange(hsv, red_lower2, red_upper2, red_mask2);
    red_mask = red_mask1 | red_mask2;
    cv::inRange(hsv, green_lower, green_upper, green_mask);
    cv::inRange(hsv, blue_lower, blue_upper, blue_mask);

    // 计算各颜色的像素数量
    int red_pixels = cv::countNonZero(red_mask);
    int green_pixels = cv::countNonZero(green_mask);
    int blue_pixels = cv::countNonZero(blue_mask);

    // 判断主要颜色
    int max_pixels = std::max({red_pixels, green_pixels, blue_pixels});

    if (max_pixels < 100)
    { // 阈值，避免噪声
        return "unknown";
    }

    if (max_pixels == red_pixels)
    {
        return "red";
    }
    else if (max_pixels == green_pixels)
    {
        return "green";
    }
    else if (max_pixels == blue_pixels)
    {
        return "blue";
    }

    return "unknown";
}

/************************************************************************
函数 5: 目标识别函数 - 用于识别任务
识别数字、字母和二维码
*************************************************************************/
bool recognizeTargets(std::vector<std::string> &contents, std::vector<geometry_msgs::Point> &positions)
{
    // 这里需要实现具体的识别算法
    // 可以使用OpenCV的模板匹配、特征检测或二维码识别库

    // 示例：简单的二维码识别（需要安装ZBar或OpenCV的二维码识别模块）
    // cv::QRCodeDetector qrDetector;
    // std::string info = qrDetector.detectAndDecode(down_camera_image);

    // 暂时返回模拟数据
    contents.push_back("A123");
    positions.push_back(local_pos.pose.pose.position);

    return !contents.empty();
}

/************************************************************************
函数 6: 无人机位置控制
控制无人机飞向（x, y, z）位置，target_yaw为目标航向角，error_max为允许的误差范围
进入函数后开始控制无人机飞向目标点，返回值为bool型，表示是否到达目标点
*************************************************************************/
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
bool mission_pos_cruise_flag = false;
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max);
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max)
{
    if (mission_pos_cruise_flag == false)
    {
        mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
        mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
        mission_pos_cruise_flag = true;
    }

    // 当前相对位置（以起飞点为原点）
    double cur_rel_x = local_pos.pose.pose.position.x - init_position_x_take_off;
    double cur_rel_y = local_pos.pose.pose.position.y - init_position_y_take_off;
    double cur_rel_z = local_pos.pose.pose.position.z - init_position_z_take_off;

    // 目标也以相对坐标传入
    double dx = static_cast<double>(x) - cur_rel_x;
    double dy = static_cast<double>(y) - cur_rel_y;
    double dz = static_cast<double>(z) - cur_rel_z;

    // 限制每次移动步长，做到"一点点挪动"
    double step_x = std::max<double>(-MAX_STEP, std::min<double>(dx, MAX_STEP));
    double step_y = std::max<double>(-MAX_STEP, std::min<double>(dy, MAX_STEP));
    double step_z = std::max<double>(-MAX_Z_STEP, std::min<double>(dz, MAX_Z_STEP));

    double cmd_rel_x = cur_rel_x + step_x;
    double cmd_rel_y = cur_rel_y + step_y;
    double cmd_rel_z = cur_rel_z + step_z;

    // 转回世界坐标发布
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = static_cast<float>(cmd_rel_x + init_position_x_take_off);
    setpoint_raw.position.y = static_cast<float>(cmd_rel_y + init_position_y_take_off);
    setpoint_raw.position.z = static_cast<float>(cmd_rel_z + init_position_z_take_off);
    setpoint_raw.yaw = target_yaw;

    if (if_debug == 1)
    {
        ROS_INFO("mission_pos_cruise: cur_rel(%.2f,%.2f,%.2f) target_rel(%.2f,%.2f,%.2f) step(%.2f,%.2f,%.2f) cmd_world(%.2f,%.2f,%.2f)",
                 cur_rel_x, cur_rel_y, cur_rel_z, x, y, z, step_x, step_y, step_z,
                 setpoint_raw.position.x, setpoint_raw.position.y, setpoint_raw.position.z);
    }

    // 到达判定仍使用原来的误差范围（相比世界坐标）
    float world_target_x = x + init_position_x_take_off;
    float world_target_y = y + init_position_y_take_off;
    float world_target_z = z + init_position_z_take_off;

    if (fabs(local_pos.pose.pose.position.x - world_target_x) < error_max &&
        fabs(local_pos.pose.pose.position.y - world_target_y) < error_max &&
        fabs(local_pos.pose.pose.position.z - world_target_z) < error_max &&
        fabs(yaw - target_yaw) < 0.1)
    {
        ROS_INFO("到达目标点，巡航点任务完成");
        mission_pos_cruise_flag = false;
        return true;
    }

    return false;
}

/************************************************************************
函数 7: 精准降落函数
基于颜色识别进行精准降落
*************************************************************************/
bool precision_land_with_color_detection();
bool precision_land_with_color_detection()
{
    static ros::Time start_time = ros::Time::now();
    static std::string detected_color = "unknown";
    static bool color_detected = false;

    if (down_camera_updated)
    {
        detected_color = recognizeColor(down_camera_image);
        color_detected = true;
        ROS_INFO("检测到颜色: %s", detected_color.c_str());
    }

    // 根据检测到的颜色选择降落目标
    // 这里需要根据实际地图调整目标坐标
    float target_x = 0, target_y = 0;
    if (detected_color == "red")
    {
        target_x = 1.0; // 红色靶标位置
        target_y = 0.0;
    }
    else if (detected_color == "green")
    {
        target_x = 2.0; // 绿色靶标位置
        target_y = 0.0;
    }
    else if (detected_color == "blue")
    {
        target_x = 3.0; // 蓝色靶标位置
        target_y = 0.0;
    }
    else
    {
        // 未识别到颜色，使用默认降落点
        target_x = local_pos.pose.pose.position.x;
        target_y = local_pos.pose.pose.position.y;
    }

    // 控制无人机飞到目标上方
    if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max))
    {
        // 开始下降
        setpoint_raw.position.x = target_x;
        setpoint_raw.position.y = target_y;
        setpoint_raw.position.z = -0.15;
        setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
        setpoint_raw.coordinate_frame = 1;

        if (ros::Time::now() - start_time > ros::Duration(5.0))
        {
            ROS_INFO("精准降落完成，检测颜色: %s", detected_color.c_str());
            return true;
        }
    }

    return false;
}

/************************************************************************
函数 8: 降落（原函数保留）
无人机当前位置作为降落点，缓慢下降至地面
返回值为bool型，表示是否降落完成
*************************************************************************/
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
bool precision_land_init_position_flag = false;
ros::Time precision_land_last_time;
bool precision_land();
bool precision_land()
{
    if (!precision_land_init_position_flag)
    {
        precision_land_init_position_x = local_pos.pose.pose.position.x;
        precision_land_init_position_y = local_pos.pose.pose.position.y;
        precision_land_last_time = ros::Time::now();
        precision_land_init_position_flag = true;
    }
    setpoint_raw.position.x = precision_land_init_position_x;
    setpoint_raw.position.y = precision_land_init_position_y;
    setpoint_raw.position.z = -0.15;
    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    if (ros::Time::now() - precision_land_last_time > ros::Duration(5.0))
    {
        ROS_INFO("Precision landing complete.");
        precision_land_init_position_flag = false; // Reset for next landing
        return true;
    }
    return false;
}

//************************************************************************
// judge whether the drone reach the target point
bool isReached(float target_x, float target_y, float target_z, float error_max)
{
    float world_target_x = target_x + init_position_x_take_off;
    float world_target_y = target_y + init_position_y_take_off;
    float world_target_z = target_z + init_position_z_take_off;

    return (fabs(local_pos.pose.pose.position.x - world_target_x) < error_max &&
            fabs(local_pos.pose.pose.position.y - world_target_y) < error_max &&
            fabs(local_pos.pose.pose.position.z - world_target_z) < error_max);
}

/**************************************************************************
函数 9: 场地受限避障飞行函数
针对小场地优化的避障策略 (修复类型不匹配问题)
*************************************************************************/
bool avoid_to_point(float target_x, float target_y, float target_z, float target_yaw, float error_max)
{
    static bool avoiding = false;
    static ros::Time avoid_start_time;
    static geometry_msgs::Point avoid_target;
    static int failed_avoidance_attempts = 0; // 失败尝试计数
    static bool forced_navigation = false; // 强制导航模式
    
    // 场地边界限制 (根据您的实际场地修改这些值)
    const float FIELD_MIN_X = 0.0;
    const float FIELD_MAX_X = 15.0; // 假设场地X方向最大15米
    const float FIELD_MIN_Y = 0.0;
    const float FIELD_MAX_Y = 10.0; // 假设场地Y方向最大10米
    
    // 障碍物确认机制
    static bool confirmed_obstacle = false;
    static ros::Time last_obstacle_time;
    
    // 更新障碍物确认状态
    if (obstacle_detected) {
        if (!confirmed_obstacle) {
            ROS_WARN("检测到障碍物，开始评估避障策略");
        }
        confirmed_obstacle = true;
        last_obstacle_time = ros::Time::now();
    } else if (confirmed_obstacle && (ros::Time::now() - last_obstacle_time > ros::Duration(1.0))) {
        confirmed_obstacle = false;
        forced_navigation = false;
        failed_avoidance_attempts = 0;
    }
    
    // 检查是否需要进入避障模式
    if (confirmed_obstacle && !avoiding && !forced_navigation) {
        avoiding = true;
        avoid_start_time = ros::Time::now();
        
        // 计算避障点，但增加场地限制
        avoid_target = calculateAvoidancePoint(target_x, target_y, target_z);
        
        // 检查避障点是否在场地范围内
        if (avoid_target.x < FIELD_MIN_X || avoid_target.x > FIELD_MAX_X || 
            avoid_target.y < FIELD_MIN_Y || avoid_target.y > FIELD_MAX_Y) {
            ROS_WARN("计算的避障点(%.2f, %.2f)超出场地范围!", avoid_target.x, avoid_target.y);
            
            // 尝试替代避障策略 - 后退并上升
            // 使用相对当前位置作为基准（world -> relative）
            float current_rel_x = static_cast<float>(local_pos.pose.pose.position.x - init_position_x_take_off);
            float current_rel_y = static_cast<float>(local_pos.pose.pose.position.y - init_position_y_take_off);
            float current_z = static_cast<float>(local_pos.pose.pose.position.z);

            // 选择一个在场地范围内的安全点（相对坐标）
            avoid_target.x = current_rel_x - 1.0f; // 向后退1米（相对）
            avoid_target.y = current_rel_y;
            avoid_target.z = current_z + 0.5f; // 上升0.5米（高度仍使用world高度，mission_pos_cruise接受相对高度，后续调用使用target_z）

            // 确保安全点在场地内
            // 显式使用 double，避免 std::min/std::max 模板类型推导冲突
            avoid_target.x = std::max(static_cast<double>(FIELD_MIN_X + 0.5f),
                                      std::min<double>(avoid_target.x, static_cast<double>(FIELD_MAX_X - 0.5f)));
            avoid_target.y = std::max(static_cast<double>(FIELD_MIN_Y + 0.5f),
                                      std::min<double>(avoid_target.y, static_cast<double>(FIELD_MAX_Y - 0.5f)));
            avoid_target.z = std::min<double>(avoid_target.z, static_cast<double>(3.0));
             
            ROS_INFO("使用替代避障点: (%.2f, %.2f, %.2f)", avoid_target.x, avoid_target.y, avoid_target.z);
        }
    }
    
    // 避障模式
    if (avoiding && !forced_navigation) {
        // 使用mission_pos_cruise飞向避障点
        bool reached_avoid_point = mission_pos_cruise(
            avoid_target.x, 
            avoid_target.y, 
            avoid_target.z > 0 ? avoid_target.z : target_z, 
            target_yaw, 
            error_max * 1.5f // 放宽避障点的到达精度
        );
        
        // 检查是否到达避障点或超时
        bool timeout = (ros::Time::now() - avoid_start_time > ros::Duration(8.0));
        
        if (reached_avoid_point || timeout) {
            if (timeout) {
                ROS_WARN("避障点到达超时，尝试直接前往目标");
                failed_avoidance_attempts++;
            }
            
            // 检查是否仍然检测到障碍物
            if (obstacle_detected && failed_avoidance_attempts < 3) {
                // 重新计算避障点
                avoid_start_time = ros::Time::now();
                avoid_target = calculateAvoidancePoint(target_x, target_y, target_z);
                ROS_INFO("重新计算避障点: (%.2f, %.2f)", avoid_target.x, avoid_target.y);
            } else if (failed_avoidance_attempts >= 3) {
                // 多次失败后，尝试强制导航
                ROS_WARN("多次避障失败，切换到强制导航模式");
                forced_navigation = true;
                avoiding = false;
                failed_avoidance_attempts = 0;
            } else {
                // 成功绕过障碍物
                avoiding = false;
                ROS_INFO("成功绕过障碍物，继续前往目标点");
            }
        }
        
        // 打印状态
        ROS_INFO("避障模式: 当前(%.2f,%.2f,%.2f) -> 避障点(%.2f,%.2f,%.2f) [%d次失败]",
                 static_cast<float>(local_pos.pose.pose.position.x), 
                 static_cast<float>(local_pos.pose.pose.position.y), 
                 static_cast<float>(local_pos.pose.pose.position.z),
                 avoid_target.x, avoid_target.y, avoid_target.z > 0 ? avoid_target.z : target_z,
                 failed_avoidance_attempts);
                 
        return false;
    }
    
    // 强制导航模式 - 当避障多次失败时
    if (forced_navigation) {
        ROS_WARN_THROTTLE(1.0, "强制导航模式: 忽略障碍物，直接前往目标点");
        float current_z = static_cast<float>(local_pos.pose.pose.position.z);
        float safe_z = std::max(target_z, current_z + 0.3f);
        return mission_pos_cruise(target_x, target_y, safe_z, target_yaw, error_max * 2.0f);
    }
    
    // 正常飞行模式
    ROS_INFO("正常飞行模式: 前往目标点(%.2f, %.2f, %.2f)", target_x, target_y, target_z);
    return mission_pos_cruise(target_x, target_y, target_z, target_yaw, error_max);
}