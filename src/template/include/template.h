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
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

//files about image processing
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


using namespace std;

#define ALTITUDE 1.5f

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
void downCameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        down_camera_image = cv_ptr->image;
        down_camera_updated = true;
        
        // 可选：显示图像（调试用）
        if (if_debug == 1) {
            cv::imshow("Downward Camera", down_camera_image);
            cv::waitKey(1);
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

/************************************************************************
函数 4: 颜色识别函数 - 用于降落任务
识别靶标颜色并返回识别结果
*************************************************************************/
std::string recognizeColor(const cv::Mat& image)
{
    if (image.empty()) {
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
    
    if (max_pixels < 100) { // 阈值，避免噪声
        return "unknown";
    }
    
    if (max_pixels == red_pixels) {
        return "red";
    } else if (max_pixels == green_pixels) {
        return "green";
    } else if (max_pixels == blue_pixels) {
        return "blue";
    }
    
    return "unknown";
}

/************************************************************************
函数 5: 目标识别函数 - 用于识别任务
识别数字、字母和二维码
*************************************************************************/
bool recognizeTargets(std::vector<std::string>& contents, std::vector<geometry_msgs::Point>& positions)
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
	setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = x + init_position_x_take_off;
	setpoint_raw.position.y = y + init_position_y_take_off;
	setpoint_raw.position.z = z + init_position_z_take_off;
	setpoint_raw.yaw = target_yaw;
	ROS_INFO("now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f)", local_pos.pose.pose.position.x ,local_pos.pose.pose.position.y, local_pos.pose.pose.position.z, target_yaw * 180.0 / M_PI, x + init_position_x_take_off, y + init_position_y_take_off, z + init_position_z_take_off, target_yaw * 180.0 / M_PI );
	if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max && fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max && fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off) < error_max && fabs(yaw - target_yaw) < 0.1)
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
    
    if (down_camera_updated) {
        detected_color = recognizeColor(down_camera_image);
        color_detected = true;
        ROS_INFO("检测到颜色: %s", detected_color.c_str());
    }
    
    // 根据检测到的颜色选择降落目标
    // 这里需要根据实际地图调整目标坐标
    float target_x = 0, target_y = 0;
    if (detected_color == "red") {
        target_x = 1.0;  // 红色靶标位置
        target_y = 0.0;
    } else if (detected_color == "green") {
        target_x = 2.0;  // 绿色靶标位置
        target_y = 0.0;
    } else if (detected_color == "blue") {
        target_x = 3.0;  // 蓝色靶标位置
        target_y = 0.0;
    } else {
        // 未识别到颜色，使用默认降落点
        target_x = local_pos.pose.pose.position.x;
        target_y = local_pos.pose.pose.position.y;
    }
    
    // 控制无人机飞到目标上方
    if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max)) {
        // 开始下降
        setpoint_raw.position.x = target_x;
        setpoint_raw.position.y = target_y;
        setpoint_raw.position.z = -0.15;
        setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
        setpoint_raw.coordinate_frame = 1;
        
        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
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
    if(ros::Time::now() - precision_land_last_time > ros::Duration(5.0))
    {
        ROS_INFO("Precision landing complete.");
        precision_land_init_position_flag = false; // Reset for next landing
        return true;
    }
    return false;
}



//************************************************************************
//judge whether the drone reach the target point
bool isReached(float target_x, float target_y, float target_z, float error_max)
{
	if (fabs(local_pos.pose.pose.position.x - target_x) < error_max && fabs(local_pos.pose.pose.position.y - target_y) < error_max && fabs(local_pos.pose.pose.position.z - target_z) < error_max)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//avoid and get to the target point
void avoid_to_point(float target_x, float target_y, float target_z, float target_yaw, float error_max)
{
	setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
	setpoint_raw.coordinate_frame = 1;
    //dai wan cheng
	setpoint_raw.position.x = target_x;
	setpoint_raw.position.y = target_y;
	setpoint_raw.position.z = target_z;
	setpoint_raw.yaw = target_yaw;
	ROS_INFO("now (%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f)", local_pos.pose.pose.position.x ,local_pos.pose.pose.position.y, local_pos.pose.pose.position.z, target_x, target_y, target_z);
}