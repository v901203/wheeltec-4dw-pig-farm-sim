#ifndef __WHEELTEC_ROBOT_H_
#define __WHEELTEC_ROBOT_H_

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <vector>
#include <memory>

// --- 序列埠：改用 ros-drivers/transport_drivers 的 serial_driver ---
// 原本 ROS1 用的 wjwwood/serial 在 ROS2 Humble 沒有對應套件，
// serial_driver 提供 open/close/is_open/send/receive(阻塞版本)，
// 跟原本逐位元組讀取的寫法相容，改動最小。
#include "serial_driver/serial_driver.hpp"
#include "io_context/io_context.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

// Macro definition
// 宏定义
#define SEND_DATA_CHECK   1          // Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK   0          // Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER      0X7B       // Frame head //帧头
#define FRAME_TAIL        0X7D       // Frame tail //帧尾
#define RECEIVE_DATA_SIZE 24         // The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE    11         // The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
#define PI                3.1415926f // PI //圆周率

// Relative to the range set by the IMU gyroscope, the range is ±500°, corresponding data range is ±32768
// 与IMU陀螺仪设置的量程有关，量程±500°，对应数据范围±32768
#define GYROSCOPE_RATIO   0.00026644f
// Relates to the range set by the IMU accelerometer, range is ±2g, corresponding data range is ±32768
// 与IMU加速度计设置的量程有关，量程±2g，对应数据范围±32768
#define ACCEl_RATIO       1671.84f

// External variables, IMU topic data //外部变量，IMU话题数据
extern sensor_msgs::msg::Imu Mpu6050;

// Covariance matrix for odometry topic data, used for robot_pose_ekf / robot_localization
// 协方差矩阵，用于里程计话题数据
const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0,
                                              0, 1e-3,    0,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0,
                                              0, 1e-3, 1e-9,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0,
                                              0, 1e-3,    0,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0,  1e3 };

const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0,
                                              0, 1e-3, 1e-9,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0, 1e-9 };

// Data structure for speed and position
// 速度、位置数据结构体
typedef struct __Vel_Pos_Data_
{
    float X;
    float Y;
    float Z;
} Vel_Pos_Data;

// IMU data structure
// IMU数据结构体
typedef struct __MPU6050_DATA_
{
    short accele_x_data;
    short accele_y_data;
    short accele_z_data;
    short gyros_x_data;
    short gyros_y_data;
    short gyros_z_data;
} MPU6050_DATA;

// The structure of the ROS to send data to the down machine
// ROS向下位机发送数据的结构体
typedef struct _SEND_DATA_
{
    uint8_t tx[SEND_DATA_SIZE];
    float X_speed;
    float Y_speed;
    float Z_speed;
    unsigned char Frame_Tail;
} SEND_DATA;

// The structure in which the lower computer sends data to the ROS
// 下位机向ROS发送数据的结构体
typedef struct _RECEIVE_DATA_
{
    uint8_t rx[RECEIVE_DATA_SIZE];
    uint8_t Flag_Stop;
    unsigned char Frame_Header;
    float X_speed;
    float Y_speed;
    float Z_speed;
    float Power_Voltage;
    unsigned char Frame_Tail;
} RECEIVE_DATA;

// The robot chassis class uses constructors to initialize data, publish topics, etc
// 机器人底盘类，使用构造函数初始化数据和发布话题等
class turn_on_robot : public rclcpp::Node
{
public:
    turn_on_robot();  // Constructor //构造函数
    ~turn_on_robot(); // Destructor //析构函数
    void Control();   // Loop control code //循环控制代码

private:
    rclcpp::Time _Now, _Last_Time; // Time dependent, used for integration to find displacement (mileage)
    float Sampling_Time;           // Sampling time, used for integration to find displacement (mileage)

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;
    // The speed topic subscribes to the callback function
    // 速度话题订阅回调函数
    void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher;

    void Publish_Odom();      // Pub the speedometer topic //发布里程计话题
    void Publish_ImuSensor(); // Pub the IMU sensor topic //发布IMU传感器话题
    void Publish_Voltage();   // Pub the power supply voltage topic //发布电源电压话题

    // 从串口(ttyUSB)读取运动底盘速度、IMU、电源电压数据
    bool Get_Sensor_Data();
    bool Get_Sensor_Data_New();
    unsigned char Check_Sum(unsigned char Count_Number, unsigned char mode); // BCC check function //BCC校验函数
    short IMU_Trans(uint8_t Data_High, uint8_t Data_Low);   // IMU data conversion read //IMU数据转化读取
    float Odom_Trans(uint8_t Data_High, uint8_t Data_Low);  // Odometer data is converted to read //里程计数据转化读取

    std::string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id;
    int serial_baud_rate;      // Serial communication baud rate //串口通信波特率
    RECEIVE_DATA Receive_Data; // The serial port receives the data structure //串口接收数据结构体
    SEND_DATA Send_Data;       // The serial port sends the data structure //串口发送数据结构体

    Vel_Pos_Data Robot_Pos;    // The position of the robot //机器人的位置
    Vel_Pos_Data Robot_Vel;    // The speed of the robot //机器人的速度
    MPU6050_DATA Mpu6050_Data; // IMU data //IMU数据
    float Power_voltage;       // Power supply voltage //电源电压

    // serial_driver 相關物件：需要一個 IoContext（asio 的 io_service 包裝）
    // 和一組序列埠設定，SerialPort 建構時就要吃這兩個，所以宣告成成員變數保留生命週期
    std::shared_ptr<drivers::common::IoContext> owned_ctx;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config;
    std::unique_ptr<drivers::serial_driver::SerialPort> Stm32_Serial;
};
#endif
