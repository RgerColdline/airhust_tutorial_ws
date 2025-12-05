#include <template.h>

// 将像素坐标转换为世界坐标（下视针孔模型）
geometry_msgs::Point pixelToWorld(int u, int v)
{
	geometry_msgs::Point p;
	if (!cam_info_received)
	{
		p.x = local_pos.pose.pose.position.x;
		p.y = local_pos.pose.pose.position.y;
		p.z = local_pos.pose.pose.position.z;
		return p;
	}

	double Z = local_pos.pose.pose.position.z;
	double Xc = (static_cast<double>(u) - cam_cx) / cam_fx * Z;
	// 图像 v 向下为正，取反以匹配机体/世界坐标系
	double Yc = - (static_cast<double>(v) - cam_cy) / cam_fy * Z;

	double rel_x = Xc;
	double rel_y = Yc;

	double world_rel_x = cos(yaw) * rel_x - sin(yaw) * rel_y;
	double world_rel_y = sin(yaw) * rel_x + cos(yaw) * rel_y;

	p.x = init_position_x_take_off + static_cast<float>(world_rel_x);
	p.y = init_position_y_take_off + static_cast<float>(world_rel_y);
	p.z = 0.0;
	return p;
}

// 摄像头内参回调
void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg)
{
	cam_info = *msg;
	cam_fx = cam_info.K[0];
	cam_fy = cam_info.K[4];
	cam_cx = cam_info.K[2];
	cam_cy = cam_info.K[5];
	cam_info_received = true;
	if (if_debug == 1)
		ROS_INFO("Camera info received fx=%.2f fy=%.2f cx=%.2f cy=%.2f", cam_fx, cam_fy, cam_cx, cam_cy);
}

// 生成字符模板（数字+大写字母）
void generateCharTemplates(int fontFace, double fontScale, int thickness)
{
	char_templates.clear();
	int tw = 60, th = 100;
	for (char c = '0'; c <= '9'; ++c)
	{
		cv::Mat img(th, tw, CV_8UC1, cv::Scalar(0));
		std::string s(1, c);
		int baseline = 0;
		cv::Size ts = cv::getTextSize(s, fontFace, fontScale, thickness, &baseline);
		cv::Point org((tw - ts.width) / 2, (th + ts.height) / 2);
		cv::putText(img, s, org, fontFace, fontScale, cv::Scalar(255), thickness, cv::LINE_AA);
		cv::threshold(img, img, 50, 255, cv::THRESH_BINARY);
		char_templates[c] = img;
	}
	for (char c = 'A'; c <= 'Z'; ++c)
	{
		cv::Mat img(th, tw, CV_8UC1, cv::Scalar(0));
		std::string s(1, c);
		int baseline = 0;
		cv::Size ts = cv::getTextSize(s, fontFace, fontScale, thickness, &baseline);
		cv::Point org((tw - ts.width) / 2, (th + ts.height) / 2);
		cv::putText(img, s, org, fontFace, fontScale, cv::Scalar(255), thickness, cv::LINE_AA);
		cv::threshold(img, img, 50, 255, cv::THRESH_BINARY);
		char_templates[c] = img;
	}
	if (if_debug == 1)
		ROS_INFO("Generated %zu character templates", char_templates.size());
}

// 识别函数：二维码优先，若非二维码用模板匹配数字/字母（返回世界坐标）
bool recognizeTargets(std::vector<std::string> &contents, std::vector<geometry_msgs::Point> &positions)
{
	contents.clear();
	positions.clear();

	if (down_camera_image.empty())
		return false;

	cv::Mat img = down_camera_image.clone();

	// QR 优先
	try
	{
		cv::QRCodeDetector qr;
		std::vector<cv::Point> bbox;
		std::string decoded = qr.detectAndDecode(img, bbox);
		if (!decoded.empty() && bbox.size() >= 4)
		{
			cv::Point2f center(0.0f, 0.0f);
			for (const auto &pt : bbox)
				center += cv::Point2f(static_cast<float>(pt.x), static_cast<float>(pt.y));
			center *= (1.0f / bbox.size());

			contents.push_back(decoded);
			positions.push_back(pixelToWorld(static_cast<int>(center.x), static_cast<int>(center.y)));
			if (if_debug == 1)
				ROS_INFO("QR decoded: %s at pixel (%.1f, %.1f)", decoded.c_str(), center.x, center.y);
		}
	}
	catch (const std::exception &e)
	{
		ROS_WARN("QR detection failed: %s", e.what());
	}

	// 模板匹配识别数字/字母
	if (char_templates.empty())
		generateCharTemplates();

	cv::Mat gray;
	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
	cv::Mat th;
	cv::adaptiveThreshold(gray, th, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 15, 8);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(th, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	for (const auto &cont : contours)
	{
		cv::Rect r = cv::boundingRect(cont);
		if (r.area() < 800) continue;
		cv::Mat roi = th(r);
		cv::Mat roi_resized;
		cv::resize(roi, roi_resized, cv::Size(60, 100));
		double best_score = -1.0;
		char best_char = '?';
		for (const auto &kv : char_templates)
		{
			cv::Mat result;
			cv::matchTemplate(roi_resized, kv.second, result, cv::TM_CCOEFF_NORMED);
			double minv, maxv;
			cv::minMaxLoc(result, &minv, &maxv);
			if (maxv > best_score)
			{
				best_score = maxv;
				best_char = kv.first;
			}
		}
		if (best_score > 0.55)
		{
			std::string s(1, best_char);
			int cx = r.x + r.width / 2;
			int cy = r.y + r.height / 2;
			contents.push_back(s);
			positions.push_back(pixelToWorld(cx, cy));
			if (if_debug == 1)
				ROS_INFO("Matched char '%c' score=%.2f at pixel(%d,%d)", best_char, best_score, cx, cy);
		}
		if (contents.size() >= 3) break;
	}
	return !contents.empty();
}

// 恢复全局 target publisher 定义，避免 "not declared in this scope" 错误
ros::Publisher target_pub;

// 简化版 print_param 实现，供 main 调用打印基本参数
void print_param()
{
    std::cout << "=== 控制参数 ===" << std::endl;
    std::cout << "err_max: " << err_max << std::endl;
    std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
    std::cout << "if_debug: " << if_debug << std::endl;
}

int main(int argc, char **argv)
{
	// 防止中文输出乱码
	setlocale(LC_ALL, "");

	ros::init(argc, argv, "template");
	ros::NodeHandle nh;

	// 订阅与发布
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber down_camera_sub = it.subscribe("camera/image_raw", 1, downCameraCallback);
	ros::Subscriber cam_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("camera/camera_info", 5, cameraInfoCallback);

	ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);
	target_pub = nh.advertise<std_msgs::String>("/target", 10);

	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	ros::Rate rate(20);

	nh.param<float>("err_max", err_max, 0);
	nh.param<float>("if_debug", if_debug, 0);
	print_param();

	// 等待飞控连接
	while (ros::ok() && !current_state.connected)
	{
		ros::spinOnce();
		rate.sleep();
	}

	// 初始 setpoint（保持 ALTITUDE 高度）
	setpoint_raw.type_mask = +64 + 128 + 256 + 512;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = init_position_x_take_off;
	setpoint_raw.position.y = init_position_y_take_off;
	setpoint_raw.position.z = init_position_z_take_off + ALTITUDE;
	setpoint_raw.yaw = 0.0;

	for (int i = 100 && ros::ok(); i > 0; --i)
	{
		mavros_setpoint_pos_pub.publish(setpoint_raw);
		ros::spinOnce();
		rate.sleep();
	}

	// OFFBOARD / ARM 尝试（简单方式）
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	ros::Time last_request = ros::Time::now();

	while (ros::ok() && !current_state.armed)
	{
		if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
		{
			set_mode_client.call(offb_set_mode);
			last_request = ros::Time::now();
		}
		if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
		{
			arming_client.call(arm_cmd);
			last_request = ros::Time::now();
		}
		mavros_setpoint_pos_pub.publish(setpoint_raw);
		ros::spinOnce();
		rate.sleep();
	}

	// 生成模板
	generateCharTemplates();

	// 完全替换的移动逻辑：起飞到 5m -> 前往 (28,4) -> 前往 (28,-4) -> 停留/降落
	const float takeoff_alt = 5.0f;
	const float cruise_alt = takeoff_alt;
	const float wp1_x = 28.0f, wp1_y = 4.0f;
	const float wp2_x = 28.0f, wp2_y = -4.0f;
	const float landing_descend_z = -0.15f;

	// 1) 升到 takeoff_alt（相对）
	ROS_INFO("移动逻辑：升至 %.2fm 并执行航线 (%.1f,%.1f)->(%.1f,%.1f)", takeoff_alt, wp1_x, wp1_y, wp2_x, wp2_y);
	while (ros::ok() && !mission_pos_cruise(0.0f, 0.0f, takeoff_alt, 0.0f, err_max))
	{
		mavros_setpoint_pos_pub.publish(setpoint_raw);
		ros::spinOnce();
		rate.sleep();
	}

	// 2) 飞到 (28, 4)
	while (ros::ok() && !mission_pos_cruise(wp1_x, wp1_y, cruise_alt, 0.0f, err_max))
	{
		mavros_setpoint_pos_pub.publish(setpoint_raw);
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("到达航点1 (%.1f, %.1f)", wp1_x, wp1_y);

	// 3) 飞到 (28, -4)
	while (ros::ok() && !mission_pos_cruise(wp2_x, wp2_y, cruise_alt, 0.0f, err_max))
	{
		mavros_setpoint_pos_pub.publish(setpoint_raw);
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("到达航点2 (%.1f, %.1f)", wp2_x, wp2_y);

	// 4) 在终点停留并执行降落命令（发布若干周期）
	float world_x = wp2_x + init_position_x_take_off;
	float world_y = wp2_y + init_position_y_take_off;
	setpoint_raw.position.x = world_x;
	setpoint_raw.position.y = world_y;
	setpoint_raw.position.z = landing_descend_z;
	setpoint_raw.type_mask = +64 + 128 + 256 + 512;
	setpoint_raw.coordinate_frame = 1;
	ros::Time t0 = ros::Time::now();
	while (ros::ok() && ros::Time::now() - t0 < ros::Duration(6.0))
	{
		mavros_setpoint_pos_pub.publish(setpoint_raw);
		ros::spinOnce();
		rate.sleep();
	}

	ROS_INFO("移动任务完成，节点退出");
	return 0;
}
