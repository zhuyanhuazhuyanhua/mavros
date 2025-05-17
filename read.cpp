#include "geometry_msgs/Pose.h"
#include "ros/console.h"
#include "ros/init.h"
#include "ros/subscriber.h"
#include <ros/ros.h>
#include <string>
#include "tracker.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <queue>
#include <tf/transform_broadcaster.h>
#include <serial/serial.h>
#include <vector>
#include "PointsPlanner.hpp"

// #define POINTS 10 //从launch文件参数中获取的目标点数量
#define PIDS 18 // PID参数的数量 kp, ki, kd, 输出最小, 输出最大, 微分限幅

using namespace std;

string port;
int baudrate;
serial::Serial ser;
unsigned char send_data[19];
unsigned char recv_data[11];
typedef union
{
  int32_t data32i;
  unsigned char data8i[4];
} data32_i;

typedef union
{
  float data32;
  unsigned char data8[4];
} data32_u;
struct laser_data
{
  uint8_t head = 0xAA;
  ;
  uint8_t d_addr = 0xFF;
  uint8_t id;
  uint8_t len;
  float ctrl_POS_X;
  float ctrl_POS_Y;
  float ctrl_POS_Z;
  uint8_t sum_check;
  uint8_t add_check;
  uint8_t end;
};
struct laser_data data_send;
void set_data(uint8_t id, float POS_X, float POS_Y, float POS_Z, uint8_t end)
{
  // data_send.head = 0xAA;
  // data_send.d_addr = 0xFF;
  data_send.id = id;
  data_send.len = 0x0F;
  data_send.ctrl_POS_X = POS_X;
  data_send.ctrl_POS_Y = POS_Y;
  data_send.ctrl_POS_Z = POS_Z;
  data_send.sum_check = 0x00;
  data_send.add_check = 0x00;
  data_send.end = end;
}

void send_to_laser()
{
  uint8_t _cnt = 0;
  data32_u _temp;
  send_data[_cnt++] = data_send.head;
  data_send.sum_check += data_send.head;
  data_send.add_check += data_send.sum_check;
  send_data[_cnt++] = data_send.d_addr;
  data_send.sum_check += data_send.d_addr;
  data_send.add_check += data_send.sum_check;
  send_data[_cnt++] = data_send.id;
  data_send.sum_check += data_send.id;
  data_send.add_check += data_send.sum_check;
  send_data[_cnt++] = data_send.len;
  data_send.sum_check += data_send.len;
  data_send.add_check += data_send.sum_check;

  _temp.data32 = data_send.ctrl_POS_X;
  send_data[_cnt++] = _temp.data8[0];
  data_send.sum_check += _temp.data8[0];
  data_send.add_check += data_send.sum_check;
  send_data[_cnt++] = _temp.data8[1];
  data_send.sum_check += _temp.data8[1];
  data_send.add_check += data_send.sum_check;
  send_data[_cnt++] = _temp.data8[2];
  data_send.sum_check += _temp.data8[2];
  data_send.add_check += data_send.sum_check;
  send_data[_cnt++] = _temp.data8[3];
  data_send.sum_check += _temp.data8[3];
  data_send.add_check += data_send.sum_check;

  _temp.data32 = data_send.ctrl_POS_Y;
  send_data[_cnt++] = _temp.data8[0];
  data_send.sum_check += _temp.data8[0];
  data_send.add_check += data_send.sum_check;
  send_data[_cnt++] = _temp.data8[1];
  data_send.sum_check += _temp.data8[1];
  data_send.add_check += data_send.sum_check;
  send_data[_cnt++] = _temp.data8[2];
  data_send.sum_check += _temp.data8[2];
  data_send.add_check += data_send.sum_check;
  send_data[_cnt++] = _temp.data8[3];
  data_send.sum_check += _temp.data8[3];
  data_send.add_check += data_send.sum_check;

  _temp.data32 = data_send.ctrl_POS_Z;
  send_data[_cnt++] = _temp.data8[0];
  data_send.sum_check += _temp.data8[0];
  data_send.add_check += data_send.sum_check;
  send_data[_cnt++] = _temp.data8[1];
  data_send.sum_check += _temp.data8[1];
  data_send.add_check += data_send.sum_check;
  send_data[_cnt++] = _temp.data8[2];
  data_send.sum_check += _temp.data8[2];
  data_send.add_check += data_send.sum_check;
  send_data[_cnt++] = _temp.data8[3];
  data_send.sum_check += _temp.data8[3];
  data_send.add_check += data_send.sum_check;

  send_data[_cnt++] = data_send.sum_check;
  send_data[_cnt++] = data_send.add_check;
  send_data[_cnt++] = data_send.end;
  ser.write(send_data, _cnt);
}

bool judge_height()
{
  // ROS_INFO("Judge");
  if (sizeof(recv_data) != 11)
  {
    return 0;
  }
  if (recv_data[0] == 'h' && recv_data[1] == 'e' && recv_data[2] == 'i' && recv_data[3] == 'g' && recv_data[4] == 'h' && recv_data[5] == 't' && recv_data[6] == 0)
  {
    ROS_INFO("Judge");
    return 1;
  }
  else
    return 0;
}

void uart_recv_handle(int32_t *z)
{
  ROS_INFO("Begin Reading");
  int i = 0;
  data32_i _temp;
  _temp.data8i[3] = recv_data[7];
  _temp.data8i[2] = recv_data[8];
  _temp.data8i[1] = recv_data[9];
  _temp.data8i[0] = recv_data[10];
  *z = _temp.data32i;
}
std::mutex data_mutex;

void PoseMinus(geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2)
{
  pose1.position.x = pose1.position.x - pose2.position.x;
  pose1.position.y = pose1.position.y - pose2.position.y;
  pose1.position.z = pose1.position.z - pose2.position.z;
}

void callback(const boost::shared_ptr<const nav_msgs::Odometry> &input, geometry_msgs::Twist &twist, geometry_msgs::Pose &pose)
{
  // ROS_INFO("Frame id :%s\n", input->child_frame_id.c_str());

  std::lock_guard<std::mutex> lock(data_mutex); // 加进程锁，保护数据
  // 位置
  pose.position.x = input->pose.pose.position.x;
  pose.position.y = input->pose.pose.position.y;
  pose.position.z = input->pose.pose.position.z;

  // ROS_INFO("CallBack x=%f, y=%f, z=%f", input->pose.pose.position.x, input->pose.pose.position.y, input->pose.pose.position.z);

  // 旋转
  pose.orientation.x = input->pose.pose.orientation.x;
  pose.orientation.y = input->pose.pose.orientation.y;
  pose.orientation.z = input->pose.pose.orientation.z;
  pose.orientation.w = input->pose.pose.orientation.w;
  // // 线速度 m/s
  // twist.linear.x = input->twist.twist.linear.x;
  // twist.linear.y = input->twist.twist.linear.y;
  // twist.linear.z = input->twist.twist.linear.z;
  // // 角速度 rad/s
  // twist.angular.x = input->twist.twist.angular.x;
  // twist.angular.y = input->twist.twist.angular.y;
  // twist.angular.z = input->twist.twist.angular.z;
}

// 从参数服务器获取列表参数
bool getTargetPoints(ros::NodeHandle &nh, std::vector<geometry_msgs::Pose> &targetpoints)
{
  std::vector<float> target_list;

  if (nh.getParam("targetpoints", target_list))
  {
    if (target_list.size() % 3 != 0)
    { // 每个目标点有3个元素 (x, y, z, ox, oy, oz, ow) 四元数弃置
      ROS_ERROR("Invalid targetpoints size. Each target point should have 3 elements (x, y, z).");
      return false;
    }

    for (size_t i = 0; i < target_list.size(); i += 3)
    {
      geometry_msgs::Pose pose;
      pose.position.x = target_list[i];
      pose.position.y = target_list[i + 1];
      pose.position.z = target_list[i + 2];
      // 废除目标点的位姿功能
      // pose.orientation.x = target_list[i+3];
      // pose.orientation.y = target_list[i+4];
      // pose.orientation.z = target_list[i+5];
      // pose.orientation.w = target_list[i+6];
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
      targetpoints.push_back(pose);
    }
    // if(targetpoints.size() != POINTS){
    //     ROS_ERROR("Received %d targetpoints, expected %d.", (int)targetpoints.size(), POINTS);
    //     return false;
    // }else {
    ROS_INFO("Received %zu points from parameter server. SUCCESS.", targetpoints.size());
    return true;
    // }
  }
  else
  {
    ROS_ERROR("Failed to load targetpoints from parameter server.");
    return false;
  }
}

bool getPID(ros::NodeHandle &nh, std::vector<float> &pid)
{

  if (nh.getParam("PIDs", pid))
  {
    if (pid.size() == PIDS)
    {
      ROS_INFO("Get PID coefficients. SUCCESS");
      return true;
    }
    ROS_ERROR("Get %d PID coefficients, expecting %d", (int)pid.size(), PIDS);
  }
  ROS_ERROR("Failed t get PID");
  return false;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "follow_node");
  ros::NodeHandle nh;
  nh.param("port", port, port);
  nh.param("baudrate", baudrate, baudrate);
  // printf("%d",baudrate);
  //  ROS_INFO("I changed");
  try
  {
    ser.setPort(port);
    ser.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  }
  catch (serial::IOException &e)
  {
    ROS_ERROR_STREAM("Unable to open port");
    return -1;
  }
  if (ser.isOpen())
  {
    ROS_INFO("Serial Port initialized");
  }
  else
  {
    return -1;
  }

  geometry_msgs::Twist twist_current, twist_serial;
  geometry_msgs::Pose pose_current, pose_target;
  std::vector<geometry_msgs::Pose> waypoints, targetpoints, turningpoints; // 路径填充点，目标点，转弯点（转弯点第一个点、最后一个需要是起点/终点）
  std::queue<geometry_msgs::Pose> turningpoints_queue;

  // 从参数服务器获取目标点
  if (!getTargetPoints(nh, targetpoints))
  {
    ROS_ERROR("Failed to initialize target points. Exiting.");
    return -1;
  }

  string topic;
  if (nh.getParam("topic", topic))
  {
    ROS_INFO("Get topic %s", topic.c_str());
  }
  else
  {
    ROS_ERROR("Failed to get topic. Exiting.");
    return -1;
  }
  // 订阅里程计话题
  //     ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 2,
  // ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/livox_odometry_mapped", 2,
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(topic, 2,
                                                         [&twist_current, &pose_current](const boost::shared_ptr<const nav_msgs::Odometry> &input)
                                                         { callback(input, twist_current, pose_current); });

  // 获取起点
  // {
  //     std::lock_guard<std::mutex> lock(data_mutex); //加进程锁，保护数据
  //     // turningpoints.push_back(pose_current); //TODO:一定别忘了把起点加进去！
  //     turningpoints_queue.push(pose_current); //队列版本
  // }
  // 获取转弯点 （起点+target）
  turningpoints.insert(turningpoints.end(), targetpoints.begin(), targetpoints.end());
  // 加入目标点（队列）
  for (size_t i = 0; i < targetpoints.size(); i++)
  {
    turningpoints_queue.push(targetpoints[i]);
  }

  // 输出转弯点
  // for (size_t i = 0; i < turningpoints.size(); i++) {
  //     ROS_INFO("turningpoints %d: x=%f, y=%f, z=%f", (int)i, turningpoints[i].position.x, turningpoints[i].position.y, turningpoints[i].position.z);
  // }

  // 计算路径点
  float resolution_xy = 0.2, resolution_z = 0.1; // TODO:修改FillWaypoints的参数

  // FillWaypoints(waypoints, turningpoints, resolution_xy, resolution_z);

  // 路径点输入到队列中
  std::queue<geometry_msgs::Pose> waypoints_queue;
  // FillWaypoints(waypoints_queue, turningpoints, resolution_xy, resolution_z);
  // if (waypoints_queue.size() == 0){
  //     ROS_ERROR("Failed to fill waypoints. Exiting.");
  //     return -1;
  // }
  // for (auto elem : waypoints) {
  //     waypoints_queue.push(elem);
  // }

  // 输出路径点vector
  // for (size_t i = 0; i < waypoints.size(); i++) {
  //     ROS_INFO("Waypoint %d: x=%f, y=%f, z=%f", (int)i, waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z);
  // }

  // 输出路径点queue,调试用 会清空队列
  // while (!waypoints_queue.empty()) {
  //     ROS_INFO("Waypoint: x=%f, y=%f, z=%f", waypoints_queue.front().position.x, waypoints_queue.front().position.y, waypoints_queue.front().position.z);
  //     waypoints_queue.pop();
  // }
  // return -1;

  // std::vector<float> pid; //xPID, yPID, zPID的kp, ki, kd, 输出最小, 输出最大, 微分限幅
  // if (!getPID(nh, pid)) {
  //     ROS_ERROR("Failed to get PID. Exiting.");
  //     return -1;
  // }

  // 初始化tarcker，设置delta_time cd dsd
  // float delta_time = 1.0/20;
  // Tracker tracker(PIDCompute(1, 0, 0, delta_time), PIDCompute(1, 0, 0, delta_time), PIDCompute(1, 0, 0, delta_time)); //tracker PID功能废除

  float reach_thresh_xy, reach_thresh_z;
  if (nh.getParam("reach_thresh_xy", reach_thresh_xy) && nh.getParam("reach_thresh_z", reach_thresh_z))
  {
    ROS_INFO("get reach_thresh_xy=%f, reach_thresh_z=%f", reach_thresh_xy, reach_thresh_z);
  }
  else
  {
    ROS_ERROR("Failed to get reach_thresh_xy and reach_thresh_z. Exiting.");
    return -1;
  }
  Tracker tracker(reach_thresh_xy, reach_thresh_z);

  // pose_target = waypoints_queue.front();

  ROS_INFO("Tracker start.");
  float delta_x, delta_y, delta_z;
  bool ReachFinal = false;

  // std::queue<geometry_msgs::Pose> compute_queue  = waypoints_queue; //含填充点
  std::queue<geometry_msgs::Pose> compute_queue = turningpoints_queue; // 转折点

  // 是否到降落位置
  bool lock = false;
  int count = 0;

  // 运行频率
  ros::Rate rate(10);
  int32_t Z = 0;
  geometry_msgs::Pose pose_init;
  {
    std::lock_guard<std::mutex> lock(data_mutex); // 加进程锁，保护数据
    pose_init = pose_current;
  }
  while (ros::ok())
  {

    geometry_msgs::Pose pose_temp;
    {
      std::lock_guard<std::mutex> lock(data_mutex); // 加进程锁，保护数据
      pose_temp = pose_current;
    }
    // pose_temp = PoseMinus(pose_temp, pose_init);

    /*if(ser.available())
    {
     // cout<<ser.available()<<endl;
        ROS_INFO("Reading from serial port");
        ser.read(recv_data,11);
   }*/

    /*if(ser.available())
    {
        size_t num_available = ser.available();
        size_t num_to_read = std::min(num_available, sizeof(recv_data));
        ROS_INFO("Reading from serial port");
        size_t bytes_read = ser.read(recv_data, num_to_read);
        if(bytes_read != 11)
        {
            ROS_ERROR("Received incomplete data.");
            // 处理错误，可能需要继续读取剩余的数据
        }
    }*/

    /*if(!judge_height()){
        ROS_ERROR("Not height");
    }
    uart_recv_handle(&Z);
    ROS_INFO("Receive Z=%d",Z);*/
    float z_float = Z / 100.0 - 0.07;
    // pose_temp.position.z = z_float; //TODO:激光z

    if (compute_queue.size() == 1)
    {
      ROS_INFO("To Final Point.");
      ReachFinal = true;
      pose_target = compute_queue.front();
      // break;
    }

    if (!ReachFinal && compute_queue.size() > 0)
    {
      pose_target = compute_queue.front();
      if (tracker.ReachTarget(pose_temp, pose_target))
      {
        compute_queue.pop();
        pose_target = compute_queue.front();
      }
    }
    ROS_INFO("Tracker computing. Current %f %f %f, Target %f %f %f", pose_temp.position.x, pose_temp.position.y, pose_temp.position.z,
             pose_target.position.x, pose_target.position.y, pose_target.position.z);

    // if (tracker.ReachTarget(pose_temp, pose_target)){

    //     if (waypoints_queue.empty()){
    //         ROS_INFO("All waypoints reached.");
    //         // break;
    //     }
    //     waypoints_queue.pop();
    //     pose_target = waypoints_queue.front();
    // }
    // tracker.compute(pose_temp, pose_target, twist_serial);

    delta_x = pose_target.position.x - pose_temp.position.x;
    delta_y = pose_target.position.y - pose_temp.position.y;
    // delta_x = delta_y = 0.1;
    delta_z = pose_target.position.z - pose_temp.position.z;
    // 转换Δxyz到自己坐标系
    ROS_INFO(" before Convert delta_x=%f, delta_y=%f, delta_z=%f", delta_x, delta_y, delta_z);
    tracker.CovertDelta(delta_x, delta_y, delta_z, pose_temp); // TODO:别忘了开
    // ROS_INFO("quad frame delta_x=%f, delta_y=%f, delta_z=%f", delta_x, delta_y, delta_z);
    ROS_INFO("After Convertdelta_x=%f, delta_y=%f, delta_z=%f", delta_x, delta_y, delta_z);

    if (fabs(pose_target.position.z + 1) < 0.0001 && pose_current.position.z < 0.15)
    {
      count++;
      if (count > 10)
      {
        lock = true;
      }
    }
    else
    {
      count = 0;
    }

    // 三个数都不是nan
    if (!std::isnan(delta_x) && !std::isnan(delta_y) && !std::isnan(delta_z) && !lock)
    {
      set_data(0x80, delta_x, delta_y, delta_z, 0x00);
      send_to_laser();
      // ser.write(send_data,18);
    }
    else if (!std::isnan(delta_x) && !std::isnan(delta_y) && !std::isnan(delta_z) && lock)
    {
      set_data(0x80, delta_x, delta_y, delta_z, 0x01);
      send_to_laser();
      // ser.write(send_data,18);
    }
    else
    {
      ROS_ERROR("Delta is nan.");
    }

    ros::spinOnce();
    rate.sleep();
    ROS_INFO("Spin end.");
  }
  ROS_INFO("Tracker end.");
  // ros::spin();

  return 0;
}