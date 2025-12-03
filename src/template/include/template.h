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
#include <vector> // 新增: 使用std::vector

using namespace std;

#define ALTITUDE 1.5f
#define SAFE_DISTANCE 1.0f
#define OBSTACLE_AVOID_DISTANCE 1.5f  // 缩短避障触发距离，适应狭小场地
#define TOO_CLOSE_LATERAL_THRESHOLD 0.6f // 小于该值则触发"横飞"(90度侧向)策略
#define DOOR_FORWARD_MARGIN 0.8f // 前方需清空超过此距离则可尝试通过门/狭小路口
#define DOOR_FORWARD_ANGLE (M_PI / 18.0) // 10度：前方向锥角一侧（半角）用来判定是否“前方清晰”

// 新增：防抖/冷却阈值
#define AVOID_MIN_LATERAL_THRESHOLD 0.15f   // 小于该横移直接向前，不触发侧移/横飞
#define AVOID_SIDE_SWITCH_COOLDOWN 2.0      // 切换左右侧向的最小冷却时间（秒）
#define AVOID_REUSE_RADIUS 0.20f            // 如果新避障点与上一次接近，则复用上一次的避障点
#define AVOID_STUCK_CHECK_INTERVAL 3.0      // 秒：超时检测，发现卡住后尝试上升或后退

// 最大单步移动距离（相对坐标，单位：米）
#define MAX_STEP 0.3f     // 水平每次最多移动 0.3m，可根据场地再调小
#define MAX_Z_STEP 0.2f   // 垂直每次最多移动 0.2m

#define OBSTACLE_RECORD_DISTANCE 5.0f // 记录雷达点的最大距离，用于构成占据点云

// 新增：请把基本工具函数实现上移，确保其他函数可以使用（避免未声明错误）
static inline double normalizeAngle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// 小工具：弧度转度
static inline double radToDeg(double a)
{
    return a * 180.0 / M_PI;
}

// 小工具：角度左/右描述
static inline std::string angleSide(double ang)
{
    if (fabs(ang) < 1e-6) return std::string("front");
    if (ang > 0) return std::string("left");
    return std::string("right");
}

static inline double pointDist2(double ax, double ay, double bx, double by)
{
    double dx = ax - bx;
    double dy = ay - by;
    return dx * dx + dy * dy;
}

float err_max = 0.2;
float if_debug = 1;

mavros_msgs::PositionTarget setpoint_raw;

// 保留：雷达相关的最小状态（不含避障逻辑）
// laser_scan, laser_updated, obstacle_distances 用于上层处理或将来重写避障
sensor_msgs::LaserScan laser_scan;
bool laser_updated = false;
std::vector<float> obstacle_distances; // 存储各个方向的障碍物距离
bool obstacle_detected = false;

// 新增：垂直方环函数声明（非避障相关，保留）
bool fly_through_vertical_square_ring(float center_x, float center_y, float center_z, float side_length, float error_max);

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
    if (if_debug == 1) {
        ROS_INFO("local_pos_cb: pos=(%.3f, %.3f, %.3f), rel=(%.3f, %.3f), yaw=%.1f deg", 
                 local_pos.pose.pose.position.x, local_pos.pose.pose.position.y, local_pos.pose.pose.position.z,
                 local_pos.pose.pose.position.x - init_position_x_take_off, local_pos.pose.pose.position.y - init_position_y_take_off,
                 radToDeg(yaw));
    }
    if (flag_init_position == false && (local_pos.pose.pose.position.z != 0))
    {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        init_yaw_take_off = yaw;
        flag_init_position = true;
    }
    // NOTE: detailed laser debug logging moved to laserCallback to avoid using laser variables here
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
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    // 保持雷达订阅，但移除所有避障处理逻辑（仅保存原始 ranges 到 obstacle_distances）
    laser_scan = *msg;
    laser_updated = true;
    obstacle_distances.clear();
    int num_readings = laser_scan.ranges.size();
    obstacle_distances.resize(num_readings);
    for (int i = 0; i < num_readings; ++i) {
        float d = laser_scan.ranges[i];
        obstacle_distances[i] = d;
    }
    // 不进行聚类/最小距离计算，避免与重写版本冲突
    obstacle_detected = false;
    if (if_debug == 1) ROS_INFO("laserCallback: received %d ranges", num_readings);
}

// 移除：中值窗函数（避障相关），如需重写请在新实现中添加

// 新增：查找最大安全扇区（阈值表示判定为安全的最小距离）
// 移除：查找最大安全扇区函数（避障相关）。如需重写，请在新文件中实现。

// 新增实现：在安全扇区中选择角度最接近 target_direction 的扇区
// 移除：findClosestSafeSector 实现（避障相关）。如需重写，请单独实现。

/************************************************************************
函数 3.5：避障决策函数
根据雷达数据决定避障方向
*************************************************************************/
// 已移除复杂避障逻辑，提供一个最简单的占位实现：直接返回目标点（相对坐标）
geometry_msgs::Point calculateAvoidancePoint(float target_x, float target_y, float target_z)
{
    geometry_msgs::Point p;
    p.x = target_x;
    p.y = target_y;
    p.z = target_z;
    if (if_debug == 1) ROS_INFO("calculateAvoidancePoint: stub returning target (%.3f, %.3f, %.3f)", p.x, p.y, p.z);
    return p;
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
// 已移除复杂的避障流程：提供一个简单的包装器，直接调用 mission_pos_cruise
bool avoid_to_point(float target_x, float target_y, float target_z, float target_yaw, float error_max)
{
    if (if_debug == 1) ROS_INFO("avoid_to_point: stub forwarding to mission_pos_cruise (%.3f, %.3f, %.3f)", target_x, target_y, target_z);
    return mission_pos_cruise(target_x, target_y, target_z, target_yaw, error_max);
}

/************************************************************************
函数 10: 圆环飞行函数（直线穿环实现）
从环的一侧直线通过中心到另一侧，适合“直着传过去”需求
*************************************************************************/
bool fly_through_circle_ring(float center_x, float center_y, float z, float radius, int points, float error_max);
bool fly_through_circle_ring(float center_x, float center_y, float z, float radius, int points, float error_max)
{
    // 使用相对坐标（以起飞点为原点）
    static bool initialized = false;
    static std::vector<geometry_msgs::Point> waypoints;
    static int idx = 0;

    if (!initialized)
    {
        waypoints.clear();
        idx = 0;
        // 生成直线穿过圆心的路径点
        int n = std::max(2, points); // 至少两个点：起点和终点
        double approach = 1.0; // 在圆环外的进出缓冲距离（米），可根据需要调整
        double start_x = static_cast<double>(center_x) - (static_cast<double>(radius) + approach);
        double end_x = static_cast<double>(center_x) + (static_cast<double>(radius) + approach);
        double start_y = static_cast<double>(center_y);
        double end_y = static_cast<double>(center_y);

        for (int i = 0; i < n; ++i)
        {
            double t = static_cast<double>(i) / static_cast<double>(n - 1);
            geometry_msgs::Point p;
            p.x = static_cast<float>(start_x + t * (end_x - start_x));
            p.y = static_cast<float>(start_y + t * (end_y - start_y));
            p.z = z;
            waypoints.push_back(p);
        }

        initialized = true;
        if (if_debug == 1)
            ROS_INFO("fly_through_circle_ring (straight pass): initialized %d waypoints center(%.2f, %.2f) radius %.2f", (int)waypoints.size(), center_x, center_y, radius);
    }

    if (waypoints.empty())
    {
        initialized = false;
        return true;
    }

    // 顺序飞到每个路径点（直线穿越）
    geometry_msgs::Point cur_wp = waypoints[idx];
    if (mission_pos_cruise(cur_wp.x, cur_wp.y, cur_wp.z, 0.0f, error_max))
    {
        idx++;
        if (idx >= static_cast<int>(waypoints.size()))
        {
            // 完成直线穿环，清理并返回 true
            initialized = false;
            waypoints.clear();
            idx = 0;
            if (if_debug == 1)
                ROS_INFO("fly_through_circle_ring (straight pass): completed");
            return true;
        }
    }

    return false;
}

/************************************************************************
函数 11: 方环飞行函数
在指定边长和高度上，依次飞越一个正方形的四个角
*************************************************************************/
bool fly_through_square_ring(float center_x, float center_y, float z, float side_length, float error_max);
bool fly_through_square_ring(float center_x, float center_y, float z, float side_length, float error_max)
{
    static bool initialized = false;
    static std::vector<geometry_msgs::Point> corners;
    static int idx = 0;

    if (!initialized)
    {
        corners.clear();
        idx = 0;
        float half = side_length / 2.0f;
        geometry_msgs::Point p;
        // 顺时针或逆时针绕四角
        p.x = center_x - half;
        p.y = center_y - half;
        p.z = z;
        corners.push_back(p);
        p.x = center_x + half;
        p.y = center_y - half;
        corners.push_back(p);
        p.x = center_x + half;
        p.y = center_y + half;
        corners.push_back(p);
        p.x = center_x - half;
        p.y = center_y + half;
        corners.push_back(p);

        initialized = true;
        if (if_debug == 1) ROS_INFO("fly_through_square_ring: initialized 4 corners center(%.2f,%.2f) side %.2f", center_x, center_y, side_length);
    }

    if (corners.empty())
    {
        initialized = false;
        return true;
    }

    geometry_msgs::Point cur_c = corners[idx];
    if (mission_pos_cruise(cur_c.x, cur_c.y, cur_c.z, 0.0f, error_max))
    {
        idx++;
        if (idx >= (int)corners.size())
        {
            idx = 0; // 完成一圈
            // 完成后清理和返回
            initialized = false;
            corners.clear();
            idx = 0;
            return true;
        }
    }

    return false;
}

// 新增工具函数：角度归一化、点到点距离平方
// static inline double normalizeAngle(double a)
// {
//     while (a > M_PI) a -= 2.0 * M_PI;
//     while (a < -M_PI) a += 2.0 * M_PI;
//     return a;
// }

// static inline double pointDist2(double ax, double ay, double bx, double by) {
//     double dx = ax - bx;
//     double dy = ay - by;
//     return dx*dx + dy*dy;
// }

// 新增：判断给定方向（中心角） +/- half_angle 区域是否清晰（前方无障碍）
// forward declaration for use earlier in file
// 移除：isDirectionClear 实现（避障相关）。返回 true 作为占位行为。
bool isDirectionClear(double center_angle, double min_clear_dist, double half_angle)
{
    (void)center_angle; (void)min_clear_dist; (void)half_angle;
    return true;
}

// 新增：返回空间中任意点到最近障碍物的欧式距离（世界相对坐标）
// 移除：nearestObstacleDistance（避障相关）。提供简单占位实现返回大距离。
static inline double nearestObstacleDistance(double x, double y)
{
    (void)x; (void)y;
    return 1e6;
}

// 采样线段上的点检测沿线是否满足最小间距
// 移除：isPathClear（避障相关）。提供简单占位实现，始终返回 true。
bool isPathClear(double from_x, double from_y, double to_x, double to_y, double min_clear)
{
    (void)from_x; (void)from_y; (void)to_x; (void)to_y; (void)min_clear;
    return true;
}

// 新增：垂直方环函数实现
bool fly_through_vertical_square_ring(float center_x, float center_y, float center_z, float side_length, float error_max) {
    static bool initialized_v = false;
    static std::vector<geometry_msgs::Point> corners_v;
    static int idx_v = 0;
    if (!initialized_v) {
        corners_v.clear();
        idx_v = 0;
        float half = side_length / 2.0f;
        geometry_msgs::Point p;
        p.x = center_x; // constant x
        p.y = center_y - half; p.z = center_z - half; corners_v.push_back(p);
        p.x = center_x; p.y = center_y + half; p.z = center_z - half; corners_v.push_back(p);
        p.x = center_x; p.y = center_y + half; p.z = center_z + half; corners_v.push_back(p);
        p.x = center_x; p.y = center_y - half; p.z = center_z + half; corners_v.push_back(p);
        initialized_v = true;
        if (if_debug == 1) ROS_INFO("fly_through_vertical_square_ring: initialized 4 corners center(%.2f,%.2f,%.2f) side %.2f", center_x, center_y, center_z, side_length);
    }
    if (corners_v.empty()) {
        initialized_v = false; return true;
    }
    geometry_msgs::Point cur = corners_v[idx_v];
    if (mission_pos_cruise(cur.x, cur.y, cur.z, 0.0f, error_max)) {
        idx_v++;
        if (idx_v >= (int)corners_v.size()) {
            idx_v = 0; initialized_v = false; corners_v.clear(); return true;
        }
    }
    return false;
}

