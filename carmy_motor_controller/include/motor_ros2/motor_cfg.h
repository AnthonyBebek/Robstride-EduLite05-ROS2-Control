#include <atomic>
#include <bitset>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <map>
#include <net/if.h>
#include <optional>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#define Set_mode 'j'      
#define Set_parameter 'p' 

// Control Modes
#define move_control_mode 0   // Motion Control Mode
#define PosPP_control_mode 1  // Position Mode (PP)
#define Speed_control_mode 2  // Speed Mode
#define Elect_control_mode 3  // Current Mode
#define Set_Zero_mode 4       // Zero Position Mode
#define PosCSP_control_mode 5 // Position Mode (CSP)

#define SC_MAX 23.0f          // Maximum current for the motor
#define SC_MIN 0.0f           // Minimum current for the motor
#define SV_MAX 20.0f          // Maximum speed for the motor
#define SV_MIN -20.0f         // Minimum speed for the motor
#define SCIQ_MIN -23.0f       // Minimum current for the motor

// Communication Types
// -----------------------------
// This data is put within the can header to indicate the 
// instruction you want to send to the motor
// -----------------------------
#define Communication_Type_Get_ID 0x00             // Get the ID of the motor
#define Communication_Type_MotionControl 0x01      // Send motion control commands to the motor
#define Communication_Type_MotorRequest 0x02       // Request motor data (position, velocity, torque, temperature)
#define Communication_Type_MotorEnable 0x03        // Enable the motor
#define Communication_Type_MotorStop 0x04          // Stop the motor
#define Communication_Type_SetPosZero 0x06         // Set the current position as the zero position
#define Communication_Type_Can_ID 0x07             // Set the CAN ID of the motor
#define Communication_Type_Control_Mode 0x12       // Set the control mode of the motor (motion control, position control, speed control, current control, zero position)
#define Communication_Type_GetSingleParameter 0x11 // Get a single parameter from the motor
#define Communication_Type_SetSingleParameter 0x12 // Set a single parameter on the motor
#define Communication_Type_ErrorFeedback 0x15      // Get error feedback from the motor

using ReceiveResult =
    std::optional<std::tuple<uint8_t, uint16_t, uint8_t, std::vector<uint8_t>>>;

typedef union {
  float f;
  unsigned char c[4];
} float2uchar;

typedef struct {
  int set_motor_mode;
  float set_current;
  float set_speed;
  float set_Torque;
  float set_angle;
  float set_limit_cur;
  float set_Kp;
  float set_Ki;
  float set_Kd;
  float set_iq;
  float set_id;
  float set_acc;
} Motor_Set;

class data_read_write_one {
public:
  uint16_t index;
  float data;
};

//---------------------------------------------
// Motor types enumeration
//---------------------------------------------
enum class ActuatorType {
  ROBSTRIDE_00 = 0,
  ROBSTRIDE_01 = 1,
  ROBSTRIDE_02 = 2,
  ROBSTRIDE_03 = 3,
  ROBSTRIDE_04 = 4,
  ROBSTRIDE_05 = 5,
  ROBSTRIDE_06 = 6
};

//---------------------------------------------
// Motor operation parameters structure
//---------------------------------------------
struct ActuatorOperation {
  double position; // rad
  double velocity; // rad/s
  double torque;   // Nm
  double kp;
  double kd;
};

//---------------------------------------------
// Mapping of actuator types to their operation parameters
//---------------------------------------------
// position(rad)
// velocity(rad/s)
// torque(Nm)
// kp (proportional gain)
// kd (derivative gain)
//--------------------------------------------
static const std::map<ActuatorType, ActuatorOperation>
    ACTUATOR_OPERATION_MAPPING = {
        {ActuatorType::ROBSTRIDE_00,
         {
             4 * M_PI,
             50,
             17,
             500.0,
             5.0,
         }},
        {ActuatorType::ROBSTRIDE_01,
         {
             4 * M_PI,
             44,
             17,
             500.0,
             5.0,
         }},
        {ActuatorType::ROBSTRIDE_02,
         {
             4 * M_PI,
             44,
             17,
             500.0,
             5.0,
         }},
        {ActuatorType::ROBSTRIDE_03,
         {
             4 * M_PI,
             50,
             60,
             5000.0,
             100.0,
         }},
        {ActuatorType::ROBSTRIDE_04,
         {
             18.20 * M_PI, // 16-Bit percision
             15,
             120,
             5000.0,
             100.0,
         }},
        {ActuatorType::ROBSTRIDE_05,
         {
             18.20 * M_PI, // 16-Bit percision  
             33,
             17,
             500.0,
             5.0,
         }},
        {ActuatorType::ROBSTRIDE_06,
         {
             4 * M_PI,
             20,
             36,
             5000.0,
             100.0,
         }},
};

static const uint16_t Index_List[] = {0X7005, 0X7006, 0X700A, 0X700B, 0X7010,
                                      0X7011, 0X7014, 0X7016, 0X7017, 0X7018,
                                      0x7019, 0x701A, 0x701B, 0x701C, 0x701D};


// List of indices for motor parameters, used for reading and writing motor settings
// Parameter variable name, address, description, type, size(bytes), unit/notes
class data_read_write {
public:
  data_read_write_one
      run_mode; // 0:运控模式 1:位置模式（PP） 2:速度模式 3:电流模式 4:零点模式
                // 5:位置模式（CSP） uint8  1byte
  data_read_write_one iq_ref; // 电流模式Iq指令  float 	4byte 	-23~23A
  data_read_write_one
      spd_ref; // 转速模式转速指令  float 	4byte 	-30~30rad/s
  data_read_write_one imit_torque; // 转矩限制  float 	4byte 	0~12Nm
  data_read_write_one cur_kp; // 电流的 Kp  float 	4byte 	默认值 0.125
  data_read_write_one cur_ki; // 电流的 Ki  float 	4byte 	默认值 0.0158
  data_read_write_one
      cur_filt_gain; // 电流滤波系数filt_gain  float 	4byte 	0~1.0，默认值0.1
  data_read_write_one loc_ref; // 位置模式角度指令  float 	4byte 	rad
  data_read_write_one limit_spd; // 位置模式速度设置  float 	4byte 0~30rad/s
  data_read_write_one
      limit_cur; // 速度位置模式电流设置  float 	4byte 	0~23A
  // 以下只可读
  data_read_write_one mechPos; // 负载端计圈机械角度  float 	4byte 	rad
  data_read_write_one iqf;     // iq 滤波值  float 	4byte 	-23~23A
  data_read_write_one mechVel; // 负载端转速  float 	4byte 	-30~30rad/s
  data_read_write_one VBUS;    // 母线电压  float 	4byte 	V
  data_read_write_one rotation; // 圈数  int16 	2byte   圈数
  data_read_write(const uint16_t *index_list = Index_List) {
    run_mode.index = index_list[0];
    iq_ref.index = index_list[1];
    spd_ref.index = index_list[2];
    imit_torque.index = index_list[3];
    cur_kp.index = index_list[4];
    cur_ki.index = index_list[5];
    cur_filt_gain.index = index_list[6];
    loc_ref.index = index_list[7];
    limit_spd.index = index_list[8];
    limit_cur.index = index_list[9];
    mechPos.index = index_list[10];
    iqf.index = index_list[11];
    mechVel.index = index_list[12];
    VBUS.index = index_list[13];
    rotation.index = index_list[14];
  }
};

class RobStrideMotor {
public:
  RobStrideMotor(const std::string can_interface, uint8_t master_id,
                 uint8_t motor_id, int actuator_type)
      : iface(can_interface), master_id(master_id), motor_id(motor_id),
        actuator_type(actuator_type) {
    init_socket();
  }

  ~RobStrideMotor() {
    if (socket_fd >= 0)
      close(socket_fd);
  }

  ReceiveResult receive(double timeout_sec = 0) {
    // 设置超时时间
    if (timeout_sec > 0) {
      struct timeval timeout;
      timeout.tv_sec = static_cast<int>(timeout_sec);
      timeout.tv_usec = static_cast<int>((timeout_sec - timeout.tv_sec) * 1e6);
      setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    }

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));

    ssize_t nbytes = recv(socket_fd, &frame, sizeof(struct can_frame), 0);

    if (nbytes <= 0) {
      return std::nullopt; // 超时或失败
    }

    // 检查是否是扩展帧
    if (!(frame.can_id & CAN_EFF_FLAG)) {
      throw std::runtime_error("Frame is not extended ID");
    }

    uint32_t can_id = frame.can_id & CAN_EFF_MASK;

    uint8_t communication_type = (can_id >> 24) & 0x1F;
    uint16_t extra_data = (can_id >> 8) & 0xFFFF;
    uint8_t host_id = can_id & 0xFF;

    error_code = uint8_t((can_id >> 16) & 0x3F);
    pattern = uint8_t((can_id >> 22) & 0x03);

    // std::cout << "canid bits = " << std::bitset<32>(can_id) << std::endl;
    // std::cout << "data bits = ";
    // for (int i = 7; i >= 0; --i) { // 从高字节到低字节打印
    //     std::cout << std::bitset<8>(frame.data[i]) << " ";
    // }
    std::cout << "canid = 0x" << std::hex << std::uppercase << can_id
              << std::dec << std::endl;

    // std::cout << "data = ";
    for (int i = 0; i < 8; ++i) {
      std::cout << "data[" << i << "] = 0x" << std::setw(2) << std::setfill('0')
                << std::hex << std::uppercase << static_cast<int>(frame.data[i])
                << "  ";
    }
    std::cout << std::dec << std::endl; // 恢复为十进制输出

    std::vector<uint8_t> data(frame.data, frame.data + frame.can_dlc);

    return std::make_tuple(communication_type, extra_data, host_id, data);
  }

  std::tuple<float, float, float, float> return_data_pvtt() {
    std::cout << "-----position_feedback: " << position_ << std::endl;
    std::cout << "-----velocity_feedback: " << velocity_ << std::endl;
    std::cout << "-----torque: " << torque_ << std::endl;
    std::cout << "-----temperature: " << temperature_ << std::endl;

    return std::make_tuple(position_, velocity_, torque_, temperature_);
  }

  void receive_status_frame();

  void Set_RobStrite_Motor_parameter(uint16_t Index, float Value,
                                     char Value_mode);

  std::tuple<float, float, float, float>
  send_velocity_mode_command(float velocity_rad_s);

  // 发送使能指令（通信类型3）
  std::tuple<float, float, float, float> enable_motor();

  float read_initial_position();

  void init_socket();

  uint16_t float_to_uint(float x, float x_min, float x_max, int bits);
  // 发送运控模式（控制角度 + 速度 + KP + KD）
  std::tuple<float, float, float, float>
  send_motion_command(float torque, float position_rad, float velocity_rad_s,
                      float kp = 0.5f, float kd = 0.1f);

  float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    return ((float)x_int) * span / ((1 << bits) - 1) + x_min;
  }

  void Get_RobStrite_Motor_parameter(uint16_t Index);
  std::tuple<float, float, float, float>
  RobStrite_Motor_PosPP_control(float Speed, float Acceleration, float Angle);
  std::tuple<float, float, float, float>
  RobStrite_Motor_PosCSP_control(float Speed, float Angle);
  std::tuple<float, float, float, float>
  RobStrite_Motor_Current_control(float IqCommand);
  void RobStrite_Motor_Set_Zero_control();
  void Disenable_Motor(uint8_t clear_error);
  void Set_CAN_ID(uint8_t Set_CAN_ID);
  void Set_ZeroPos();

  float Byte_to_float(uint8_t *bytedata) {
    uint32_t data =
        bytedata[7] << 24 | bytedata[6] << 16 | bytedata[5] << 8 | bytedata[4];
    float data_float = *(float *)(&data);
    return data_float;
  }

  float Byte_to_float(const std::vector<uint8_t> &bytedata) {
    if (bytedata.size() < 8)
      return 0.0f; // 防止越界
    uint32_t data = (bytedata[7] << 24) | (bytedata[6] << 16) |
                    (bytedata[5] << 8) | bytedata[4];
    float data_float;
    std::memcpy(&data_float, &data, sizeof(float)); // 比 *(float*)&data 更安全
    return data_float;
  }

public:
  std::string iface;
  uint8_t master_id;
  uint8_t motor_id;
  int socket_fd = -1;
  Motor_Set Motor_Set_All; // 设定值
  data_read_write_one params;
  data_read_write drw;

  float position_ = 0.0;
  float velocity_ = 0.0;
  float torque_ = 0.0;
  float temperature_ = 0.0;

  uint8_t error_code;
  uint8_t pattern;
  std::atomic<bool> is_move_control_first = true;
  int actuator_type;
};
