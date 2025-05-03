#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <offboard_run/SlidingWindowAverage.h>
#include <queue>
#include <ros/ros.h>
#include <vector>

Eigen::Vector3d p_lidar_body, p_enu;
Eigen::Quaterniond q_mav;
Eigen::Quaterniond q_px4_odom;
class SlidingWindowAverage
{
public:
  SlidingWindowAverage(int windowSize)
      : windowSize(windowSize), windowSum(0.0) {}

  double addData(double newData)
  {
    if (!dataQueue.empty() && fabs(newData - dataQueue.back()) > 0.01)
    {
      dataQueue = std::queue<double>();
      windowSum = 0.0;
      dataQueue.push(newData);
      windowSum += newData;
    }
    else
    {
      dataQueue.push(newData);
      windowSum += newData;
    }

    // 如果队列大小超过窗口大小，弹出队列头部元素并更新窗口和队列和
    if (dataQueue.size() > windowSize)
    {
      windowSum -= dataQueue.front();
      dataQueue.pop();
    }
    windowAvg = windowSum / dataQueue.size();
    // 返回当前窗口内的平均值
    return windowAvg;
  }

  int get_size() { return dataQueue.size(); }

  double get_avg() { return windowAvg; }

private:
  int windowSize;
  double windowSum;
  double windowAvg;
  std::queue<double> dataQueue;
};
int windowSize = 8;
SlidingWindowAverage swa = SlidingWindowAverage(windowSize);
double fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw =
      atan2(2 * (q.x() * q.y() + q.w() * q.z()),
            q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  return yaw;
}

// 建立一个订阅消息体类型的变量，用于存储订阅的信息
mavros_msgs::State current_state;
// 当前无人机位置
geometry_msgs::PoseStamped current_pose;
// 订阅时的回调函数，接受到该消息体的内容时执行里面的内容，内容是储存飞控当前的状态
void state_cb(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }
// 回调函数：更新当前无人机位置
void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  current_pose.pose = msg->pose.pose;
}
// 判断两个位置是否接近
bool is_close(const geometry_msgs::PoseStamped &pose1,
              const geometry_msgs::PoseStamped &pose2, double tolerance)
{
  double dx = pose1.pose.position.x - pose2.pose.position.x;
  double dy = pose1.pose.position.y - pose2.pose.position.y;
  double dz = pose1.pose.position.z - pose2.pose.position.z;
  return sqrt(dx * dx + dy * dy + dz * dz) < tolerance;
}

int main(int argc, char **argv)
{
  // ros系统的初始化，argc和argv在后期节点传值会使用，最后一个参数offb_node为节点名称
  ros::init(argc, argv, "offb_node");

  // 实例化ROS句柄，这个ros::NodeHandle类封装了ROS中的一些常用功能
  ros::NodeHandle nh;

  // 这是一个订阅者对象，可以订阅无人机的状态信息（状态信息来源为MAVROS发布），用来储存无人机的状态，在回调函数中会不断更新这个状态变量
  //<>里面为模板参数，传入的是订阅的消息体类型，（）里面传入三个参数，分别是该消息体的位置、缓存大小（通常为1000）、回调函数
  ros::Subscriber state_sub =
      nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

  // 订阅无人机位置信息
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
      "mavros/local_position/odom", 10, odom_cb);

  // 这是一个发布者对象，用来在本地坐标系下发布目标点，后面会以20Hz频率发布目标点
  //<>里面为模板参数，传入的是发布的消息体类型，（）里面传入两个参数，分别是该消息体的位置、缓存大小（通常为1000）
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 10);

  // 一个客户端，用来解锁无人机，这是因为无人机如果降落后一段时间没有收到信号输入，会自动上锁来保障安全
  // 启动服务的函数为nh下的serviceClient<>()函数，<>里面是该服务的类型，（）里面是该服务的路径
  ros::ServiceClient arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

  // 一个客户端，用来切换飞行模式
  // 启动服务的函数为nh下的serviceClient<>()函数，<>里面是该服务的类型，（）里面是该服务的路径
  ros::ServiceClient set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 10);

  // 官方要求local_pos_pub发布速率必须快于2Hz，这里设置为20Hz
  // PX4在两个Offboard命令之间有一个500ms的延时，如果超过此延时，系统会将回到无人机进入Offboard模式之前的最后一个模式。
  ros::Rate rate(20.0);

  // 等待飞控和MAVROS建立连接，current_state是我订阅的MAVROS的状态，在收到心跳包之后连接成功跳出循环
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // 添加目标位置到队列
  std::vector<geometry_msgs::PoseStamped> target_positions;
  geometry_msgs::PoseStamped pose1, pose2, pose3;

  pose1.pose.position.x = 0;
  pose1.pose.position.y = 0;
  pose1.pose.position.z = 1.5;
  target_positions.push_back(pose1);

  pose2.pose.position.x = 0;
  pose2.pose.position.y = 0;
  pose2.pose.position.z = 1;
  target_positions.push_back(pose2);

  pose3.pose.position.x = 0;
  pose3.pose.position.y = 0;
  pose3.pose.position.z = 0.7;
  target_positions.push_back(pose3);

  float init_yaw = 0.0;
  bool init_flag = 0;
  Eigen::Quaterniond init_q;
  if (swa.get_size() == windowSize && !init_flag)
  {
    init_yaw = swa.get_avg();
    init_flag = 1;
    init_q = Eigen::AngleAxisd(init_yaw, Eigen::Vector3d::UnitZ()) // des.yaw
             * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    // delete swa;
  }

  if (init_flag)
  {
    // 处理vector中的每个目标位置
    for (size_t i = 0; i < target_positions.size(); ++i)
    {
      geometry_msgs::PoseStamped current_pose = target_positions[i];

      // 将目标位置赋值给 p_lidar_body
      Eigen::Vector3d p_lidar_body(current_pose.pose.position.x,
                                   current_pose.pose.position.y,
                                   current_pose.pose.position.z);

      // 完成坐标系转换
      Eigen::Vector3d p_enu = init_q * p_lidar_body;

      // 更新目标位置
      current_pose.pose.position.x = p_enu[0];
      current_pose.pose.position.y = p_enu[1];
      current_pose.pose.position.z = p_enu[2];

      // 发布转换后的目标位置
      current_pose.header.stamp = ros::Time::now();
      vision_pub.publish(current_pose);

      ROS_INFO("\nposition in enu:\n   x: %.18f\n   y: %.18f\n   z: %.18f",
               p_enu[0], p_enu[1], p_enu[2]);
    }
  }
  // 在进入 OFFBOARD 模式之前，必须已经启动了 local_pos_pub 数据流
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    if (!target_positions.empty())
    {
      local_pos_pub.publish(target_positions.front());
    }
    ros::spinOnce();
    rate.sleep();
  }

  // 建立一个类型为SetMode的服务端offb_set_mode，并将其中的模式mode设为"OFFBOARD"，作用便是用于后面的客户端与服务端之间的通信（服务）
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // 建立一个类型为CommandBool的服务端arm_cmd，并将其中的是否解锁设为"true"，作用便是用于后面的客户端与服务端之间的通信（服务）
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  // 更新时间
  ros::Time last_request = ros::Time::now();

  //  大循环，只要节点还在ros::ok()的值就为正
  while (ros::ok())
  {

    if (current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      // 客户端set_mode_client向服务端offb_set_mode发起请求call，然后服务端回应response将模式返回，这就打开了offboard模式
      if (set_mode_client.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent)
      {
        // 打开Offboard模式后在终端打印信息
        ROS_INFO("Offboard enabled");
      }
      // 更新时间
      last_request = ros::Time::now();
    }

    // else指已经为offboard模式，则进入if语句内部
    else
    {
      // 判断当前状态是否解锁，如果没有解锁，则进入if语句内部
      // 这里是5秒钟进行一次判断，避免飞控被大量的请求阻塞
      if (!current_state.armed &&
          (ros::Time::now() - last_request > ros::Duration(5.0)))
      {
        // 客户端arming_client向服务端arm_cmd发起请求call，然后服务端回应response成功解锁，则解锁成功
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          // 解锁后在终端打印信息
          ROS_INFO("Vehicle armed");
        }
        // 更新时间
        last_request = ros::Time::now();
      }
    }

    // // 发布位置信息，所以综上飞机只有先打开offboard模式然后解锁才能飞起来
    // local_pos_pub.publish(pose);
    // 如果队列不为空，获取当前目标位置
    if (!target_positions.empty())
    {
      geometry_msgs::PoseStamped target_pose = target_positions[0];

      // 判断当前位置是否接近目标位置
      if (is_close(current_pose, target_pose, 0.2))
      {
        // 到达目标位置，移除当前目标位置
        target_positions.erase(target_positions.begin());
        ROS_INFO("Reached target position. Moving to next target.");
      }
      else
      {
        // 未到达目标位置，继续发布目标位置
        local_pos_pub.publish(target_pose);
      }
    }

    // 当spinOnce函数被调用时，会调用回调函数队列中第一个回调函数，这里回调函数是state_cb函数
    ros::spinOnce();

    // 根据前面ros::Rate rate(20.0);制定的发送频率自动休眠 休眠时间 = 1/频率
    rate.sleep();
  }

  return 0;
}
