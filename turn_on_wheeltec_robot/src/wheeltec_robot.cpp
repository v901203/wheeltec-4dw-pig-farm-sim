#include "turn_on_wheeltec_robot/wheeltec_robot.h"
#include "turn_on_wheeltec_robot/Quaternion_Solution.h"

sensor_msgs::msg::Imu Mpu6050; // Instantiate an IMU object //实例化IMU对象

/**************************************
Function: The main function, ROS initialization, creates the Robot_control object through the Turn_on_robot class and automatically calls the constructor initialization
功能: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);                       // ROS2 initializes //ROS2初始化
  auto Robot_Control = std::make_shared<turn_on_robot>(); // Instantiate an object //实例化一个对象
  Robot_Control->Control();                        // Loop through data collection and publish the topic //循环执行数据采集和发布话题等操作
  rclcpp::shutdown();
  return 0;
}

/**************************************
Function: Data conversion function
功能: 数据转换函数
***************************************/
short turn_on_robot::IMU_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  short transition_16;
  transition_16 = 0;
  transition_16 |= Data_High << 8;
  transition_16 |= Data_Low;
  return transition_16;
}
float turn_on_robot::Odom_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  float data_return;
  short transition_16;
  transition_16 = 0;
  transition_16 |= Data_High << 8; // Get the high 8 bits of data   //获取数据的高8位
  transition_16 |= Data_Low;       // Get the lowest 8 bits of data //获取数据的低8位
  data_return = (transition_16 / 1000) + (transition_16 % 1000) * 0.001; // The speed unit is changed from mm/s to m/s //速度单位从mm/s转换为m/s
  return data_return;
}
/**************************************
Function: The speed topic subscription Callback function, according to the subscribed instructions through the serial port command control of the lower computer
功能: 速度话题订阅回调函数Callback，根据订阅的指令通过串口发指令控制下位机
***************************************/
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
  short transition; // intermediate variable //中间变量

  Send_Data.tx[0] = FRAME_HEADER; // frame head 0x7B //帧头0X7B
  Send_Data.tx[1] = 0;            // set aside //预留位
  Send_Data.tx[2] = 0;            // set aside //预留位

  // The target velocity of the X-axis of the robot
  // 机器人x轴的目标线速度
  transition = 0;
  transition = twist_aux->linear.x * 1000; // 将浮点数放大一千倍，简化传输
  Send_Data.tx[4] = transition;            // 取数据的低8位
  Send_Data.tx[3] = transition >> 8;       // 取数据的高8位

  // The target velocity of the Y-axis of the robot
  // 机器人y轴的目标线速度
  transition = 0;
  transition = twist_aux->linear.y * 1000;
  Send_Data.tx[6] = transition;
  Send_Data.tx[5] = transition >> 8;

  // The target angular velocity of the robot's Z axis
  // 机器人z轴的目标角速度
  transition = 0;
  transition = twist_aux->angular.z * 1000;
  Send_Data.tx[8] = transition;
  Send_Data.tx[7] = transition >> 8;

  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK); // For the BCC check bits, see the Check_Sum function //BCC校验位，规则参见Check_Sum函数
  Send_Data.tx[10] = FRAME_TAIL;                    // frame tail 0x7D //帧尾0X7D
  try
  {
    std::vector<uint8_t> tx(Send_Data.tx, Send_Data.tx + SEND_DATA_SIZE);
    Stm32_Serial->send(tx); // Sends data to the downloader via serial port //通过串口向下位机发送数据
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to send data through serial port: " << e.what()); // 如果发送数据失败，打印错误信息
  }
}
/**************************************
Function: Publish the IMU data topic
功能: 发布IMU数据话题
***************************************/
void turn_on_robot::Publish_ImuSensor()
{
  sensor_msgs::msg::Imu Imu_Data_Pub; // Instantiate IMU topic data //实例化IMU话题数据
  Imu_Data_Pub.header.stamp = this->now();
  Imu_Data_Pub.header.frame_id = gyro_frame_id; // IMU对应TF坐标，使用robot_pose_ekf/robot_localization功能包需要设置此项
  Imu_Data_Pub.orientation.x = Mpu6050.orientation.x; // A quaternion represents a three-axis attitude //四元数表达三轴姿态
  Imu_Data_Pub.orientation.y = Mpu6050.orientation.y;
  Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
  Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;
  Imu_Data_Pub.orientation_covariance[0] = 1e6; // Three-axis attitude covariance matrix //三轴姿态协方差矩阵
  Imu_Data_Pub.orientation_covariance[4] = 1e6;
  Imu_Data_Pub.orientation_covariance[8] = 1e-6;
  Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x; // Triaxial angular velocity //三轴角速度
  Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
  Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
  Imu_Data_Pub.angular_velocity_covariance[0] = 1e6; // Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
  Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
  Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x; // Triaxial acceleration //三轴线性加速度
  Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y;
  Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;
  imu_publisher->publish(Imu_Data_Pub); // Pub IMU topic //发布IMU话题
}
/**************************************
Function: Publish the odometer topic, Contains position, attitude, triaxial velocity, angular velocity about triaxial, TF parent-child coordinates, and covariance matrix
功能: 发布里程计话题，包含位置、姿态、三轴速度、绕三轴角速度、TF父子坐标、协方差矩阵
***************************************/
void turn_on_robot::Publish_Odom()
{
  // Convert the Z-axis rotation Angle into a quaternion for expression
  // 把Z轴转角转换为四元数进行表达（tf::createQuaternionMsgFromYaw 的 ROS2 等价寫法）
  tf2::Quaternion q;
  q.setRPY(0, 0, Robot_Pos.Z);
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

  nav_msgs::msg::Odometry odom; // Instance the odometer topic data //实例化里程计话题数据
  odom.header.stamp = this->now();
  odom.header.frame_id = odom_frame_id; // Odometer TF parent coordinates //里程计TF父坐标
  odom.pose.pose.position.x = Robot_Pos.X; // Position //位置
  odom.pose.pose.position.y = Robot_Pos.Y;
  odom.pose.pose.position.z = Robot_Pos.Z;
  odom.pose.pose.orientation = odom_quat; // Posture, Quaternion converted by Z-axis rotation //姿态，通过Z轴转角转换的四元数

  odom.child_frame_id = robot_frame_id; // Odometer TF subcoordinates //里程计TF子坐标
  odom.twist.twist.linear.x = Robot_Vel.X;  // Speed in the X direction //X方向速度
  odom.twist.twist.linear.y = Robot_Vel.Y;  // Speed in the Y direction //Y方向速度
  odom.twist.twist.angular.z = Robot_Vel.Z; // Angular velocity around the Z axis //绕Z轴角速度

  // There are two types of this matrix, which are used when the robot is at rest and when it is moving.
  // 这个矩阵有两种，分别在机器人静止和运动的时候使用
  if (Robot_Vel.X == 0 && Robot_Vel.Y == 0 && Robot_Vel.Z == 0)
  {
    memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2));
    memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
  }
  else
  {
    memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance));
    memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
  }
  odom_publisher->publish(odom); // Pub odometer topic //发布里程计话题
}
/**************************************
Function: Publish voltage-related information
功能: 发布电压相关信息
***************************************/
void turn_on_robot::Publish_Voltage()
{
  std_msgs::msg::Float32 voltage_msgs; // 定义电源电压发布话题的数据类型
  static float Count_Voltage_Pub = 0;
  if (Count_Voltage_Pub++ > 10)
  {
    Count_Voltage_Pub = 0;
    voltage_msgs.data = Power_voltage;     // 电源供电的电压获取
    voltage_publisher->publish(voltage_msgs); // 发布电源电压话题单位：V、伏特
  }
}
/**************************************
Function: Serial port communication check function (BCC check)
功能: 串口通讯校验函数，BCC校验
***************************************/
unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number, unsigned char mode)
{
  unsigned char check_sum = 0, k;

  if (mode == 0) // Receive data mode //接收数据模式
  {
    for (k = 0; k < Count_Number; k++)
    {
      check_sum = check_sum ^ Receive_Data.rx[k]; // By bit or by bit //按位异或
    }
  }
  if (mode == 1) // Send data mode //发送数据模式
  {
    for (k = 0; k < Count_Number; k++)
    {
      check_sum = check_sum ^ Send_Data.tx[k]; // By bit or by bit //按位异或
    }
  }
  return check_sum; // Returns the bitwise XOR result //返回按位异或结果
}
/**************************************
Function: The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
Update Note: This checking method can lead to read error data or correct data not to be processed. Kept for reference only; Control() uses Get_Sensor_Data_New() instead.
功能: 通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位（保留供参考，Control() 實際使用逐帧校验版本 Get_Sensor_Data_New()）
***************************************/
bool turn_on_robot::Get_Sensor_Data()
{
  short transition_16 = 0, j = 0, Header_Pos = 0, Tail_Pos = 0; // Intermediate variable //中间变量
  static int flag_error = 0, temp = 1;                          // Static variable that records the error flag and location //静态变量，用于记录出错标志位和出错位置
  std::vector<uint8_t> Receive_Data_Pr(RECEIVE_DATA_SIZE, 0);
  std::vector<uint8_t> Receive_Data_Tr(temp, 0);
  if (flag_error == 0) // Normal condition detected //检测到正常情况
    Stm32_Serial->receive(Receive_Data_Pr);
  else if (flag_error == 1) // Error condition detected //检测到错误情况
  {
    Receive_Data_Tr.resize(temp, 0);
    Stm32_Serial->receive(Receive_Data_Tr);
    flag_error = 0; // Error flag position 0 //错误标志位置0
  }

  // Record the position of the head and tail of the frame //记录帧头帧尾位置
  for (j = 0; j < 24; j++)
  {
    if (Receive_Data_Pr[j] == FRAME_HEADER)
      Header_Pos = j;
    else if (Receive_Data_Pr[j] == FRAME_TAIL)
      Tail_Pos = j;
  }

  if (Tail_Pos == (Header_Pos + 23))
  {
    memcpy(Receive_Data.rx, Receive_Data_Pr.data(), RECEIVE_DATA_SIZE);
    flag_error = 0; // Error flag position 0 for next reading //错误标志位置0，便于下次读取
  }
  else if (Header_Pos == (1 + Tail_Pos))
  {
    temp = Header_Pos; // 记录下一次读取的长度，经计算正好为帧头的位置
    flag_error = 1;     // 错误标志位置1，让下一次读取出错位数组
    return false;
  }
  else
  {
    // 其它情况则认为数据包有错误
    return false;
  }

  Receive_Data.Frame_Header = Receive_Data.rx[0]; // 数据的第一位是帧头0X7B
  Receive_Data.Frame_Tail = Receive_Data.rx[23];  // 数据的最后一位是帧尾0X7D

  if (Receive_Data.Frame_Header == FRAME_HEADER) // 判断帧头
  {
    if (Receive_Data.Frame_Tail == FRAME_TAIL) // 判断帧尾
    {
      if (Receive_Data.rx[22] == Check_Sum(22, READ_DATA_CHECK)) // BCC校验通过或者两组数据包交错
      {
        Receive_Data.Flag_Stop = Receive_Data.rx[1]; // 预留位
        Robot_Vel.X = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]); // 获取运动底盘X方向速度
        Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]); // 获取运动底盘Y方向速度
        Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7]); // 获取运动底盘Z方向速度

        Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8], Receive_Data.rx[9]);
        Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10], Receive_Data.rx[11]);
        Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12], Receive_Data.rx[13]);
        Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14], Receive_Data.rx[15]);
        Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16], Receive_Data.rx[17]);
        Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18], Receive_Data.rx[19]);

        Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;
        Mpu6050.angular_velocity.x = Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.y = Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.z = Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;

        transition_16 = 0;
        transition_16 |= Receive_Data.rx[20] << 8;
        transition_16 |= Receive_Data.rx[21];
        Power_voltage = transition_16 / 1000 + (transition_16 % 1000) * 0.001; // 毫伏(mv)->伏(v)

        return true;
      }
    }
  }
  return false;
}
/**************************************
Function: Read and verify the data sent by the lower computer frame by frame through the serial port, and then convert the data into international units
功能: 通过串口读取并逐帧校验下位机发送过来的数据，然后数据转换为国际单位
***************************************/
bool turn_on_robot::Get_Sensor_Data_New()
{
  short transition_16 = 0; // Intermediate variable //中间变量
  uint8_t check = 0, error = 1;
  static int count; // Static variable for counting //静态变量，用于计数

  std::vector<uint8_t> Receive_Data_Pr(1, 0);
  Stm32_Serial->receive(Receive_Data_Pr); // 通过串口读取下位机发送过来的数据（阻塞，一次一个 byte，行为對齊原本 serial::Serial::read）

  Receive_Data.rx[count] = Receive_Data_Pr[0]; // 串口数据填入数组

  Receive_Data.Frame_Header = Receive_Data.rx[0]; // 数据的第一位是帧头0X7B
  Receive_Data.Frame_Tail = Receive_Data.rx[23];  // 数据的最后一位是帧尾0X7D

  if (Receive_Data_Pr[0] == FRAME_HEADER || count > 0) // 确保数组第一个数据为FRAME_HEADER
    count++;
  else
    count = 0;
  if (count == 24) // 验证数据包的长度
  {
    count = 0; // 为串口数据重新填入数组做准备
    if (Receive_Data.Frame_Tail == FRAME_TAIL) // 验证数据包的帧尾
    {
      check = Check_Sum(22, READ_DATA_CHECK); // BCC校验通过或者两组数据包交错

      if (check == Receive_Data.rx[22])
      {
        error = 0; // 异或位校验成功
      }
      if (error == 0)
      {
        Receive_Data.Flag_Stop = Receive_Data.rx[1]; // 预留位
        Robot_Vel.X = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]); // 获取运动底盘X方向速度
        Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]); // 获取运动底盘Y方向速度
        Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7]); // 获取运动底盘Z方向速度

        Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8], Receive_Data.rx[9]);
        Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10], Receive_Data.rx[11]);
        Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12], Receive_Data.rx[13]);
        Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14], Receive_Data.rx[15]);
        Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16], Receive_Data.rx[17]);
        Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18], Receive_Data.rx[19]);

        Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;
        Mpu6050.angular_velocity.x = Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.y = Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.z = Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;

        transition_16 = 0;
        transition_16 |= Receive_Data.rx[20] << 8;
        transition_16 |= Receive_Data.rx[21];
        Power_voltage = transition_16 / 1000 + (transition_16 % 1000) * 0.001; // 毫伏(mv)->伏(v)

        return true;
      }
    }
  }
  return false;
}
/**************************************
Function: Loop access to the lower computer data and issue topics
功能: 循环获取下位机数据与发布话题
***************************************/
void turn_on_robot::Control()
{
  _Last_Time = this->now();
  while (rclcpp::ok())
  {
    Sampling_Time = (_Now - _Last_Time).seconds(); // 获取时间间隔，用于积分速度获得位移(里程)
    _Now = this->now();
    if (true == Get_Sensor_Data_New()) // 通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
    {
      Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time; // 计算X方向的位移，单位：m
      Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time; // 计算Y方向的位移，单位：m
      Robot_Pos.Z += Robot_Vel.Z * Sampling_Time; // 绕Z轴的角位移，单位：rad

      // 通过IMU绕三轴角速度与三轴加速度计算三轴姿态
      Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,
                           Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);

      Publish_Odom();      // 发布里程计话题
      Publish_ImuSensor();  // 发布IMU话题
      Publish_Voltage();    // 发布电源电压话题

      _Last_Time = _Now; // 记录时间，用于计算时间间隔
    }

    // ROS1 版用 ros::spinOnce() 讓 subscriber callback 有機會被觸發；
    // ROS2 沒有 NodeHandle 層級的 spinOnce，改用 rclcpp::spin_some() 對這個 node 的 base interface 處理一輪待處理的 callback
    rclcpp::spin_some(this->get_node_base_interface());
  }
}
/**************************************
Function: Constructor, executed only once, for initialization
功能: 构造函数, 只执行一次，用于初始化
***************************************/
turn_on_robot::turn_on_robot() : Node("wheeltec_robot"), Sampling_Time(0), Power_voltage(0)
{
  // Clear the data
  // 清空数据
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data));
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

  // ROS1 的 private_nh.param() 對應 ROS2 的 declare_parameter + get_parameter
  this->declare_parameter<std::string>("usart_port_name", "/dev/wheeltec_controller"); // 固定串口号
  this->declare_parameter<int>("serial_baud_rate", 115200);                            // 和下位机通信波特率115200
  this->declare_parameter<std::string>("odom_frame_id", "odom_combined");              // 里程计话题对应父TF坐标
  this->declare_parameter<std::string>("robot_frame_id", "base_footprint");            // 里程计话题对应子TF坐标
  this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");                  // IMU话题对应TF坐标

  this->get_parameter("usart_port_name", usart_port_name);
  this->get_parameter("serial_baud_rate", serial_baud_rate);
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->get_parameter("robot_frame_id", robot_frame_id);
  this->get_parameter("gyro_frame_id", gyro_frame_id);

  voltage_publisher = this->create_publisher<std_msgs::msg::Float32>("PowerVoltage", 10); // 创建电池电压话题发布者
  odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);           // 创建里程计话题发布者
  imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu", 20);               // 创建IMU话题发布者

  // 速度控制命令订阅回调函数设置
  Cmd_Vel_Sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 100,
      std::bind(&turn_on_robot::Cmd_Vel_Callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Data ready"); // 提示信息

  // 尝试初始化与开启串口（serial_driver 版本）
  try
  {
    owned_ctx = std::make_shared<drivers::common::IoContext>(1);
    device_config = std::make_unique<drivers::serial_driver::SerialPortConfig>(
        static_cast<uint32_t>(serial_baud_rate),
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE);
    Stm32_Serial = std::make_unique<drivers::serial_driver::SerialPort>(
        *owned_ctx, usart_port_name, *device_config);
    Stm32_Serial->open(); // 开启串口
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                         "wheeltec_robot can not open serial port, Please check the serial port cable! " << e.what());
  }
  if (Stm32_Serial && Stm32_Serial->is_open())
  {
    RCLCPP_INFO(this->get_logger(), "wheeltec_robot serial port opened"); // 串口开启成功提示
  }
}
/**************************************
Function: Destructor, executed only once and called by the system when an object ends its life cycle
功能: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
turn_on_robot::~turn_on_robot()
{
  // 对象turn_on_robot结束前向下位机发送停止运动命令
  Send_Data.tx[0] = FRAME_HEADER;
  Send_Data.tx[1] = 0;
  Send_Data.tx[2] = 0;

  Send_Data.tx[4] = 0; // 机器人X轴的目标线速度
  Send_Data.tx[3] = 0;

  Send_Data.tx[6] = 0; // 机器人Y轴的目标线速度
  Send_Data.tx[5] = 0;

  Send_Data.tx[8] = 0; // 机器人Z轴的目标角速度
  Send_Data.tx[7] = 0;
  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK); // 校验位，规则参见Check_Sum函数
  Send_Data.tx[10] = FRAME_TAIL;

  try
  {
    if (Stm32_Serial && Stm32_Serial->is_open())
    {
      std::vector<uint8_t> tx(Send_Data.tx, Send_Data.tx + SEND_DATA_SIZE);
      Stm32_Serial->send(tx); // 向串口发数据
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to send data through serial port: " << e.what()); // 如果发送数据失败,打印错误信息
  }
  if (Stm32_Serial && Stm32_Serial->is_open())
  {
    Stm32_Serial->close(); // 关闭串口
  }
  RCLCPP_INFO(this->get_logger(), "Shutting down"); // 提示信息
}
