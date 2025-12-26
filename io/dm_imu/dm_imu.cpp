#include "dm_imu.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/termios.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <memory>
#include "tools/yaml.hpp"
#include "io/message/include/srm/message/info.hpp"
#include "io/message/include/srm/message/message-base.hpp"
#include "io/message/include/srm/message/packet.hpp"
#include "io/message/include/srm/common/include/srm/common.hpp"
namespace srm::message {

#if defined(__APPLE__)
const std::string kSystem = "macos";
#elif defined(__linux__)
const std::string kSystem = "linux";
#endif

/// 串口收发
class Serial3 {
 public:
  Serial3() = default;
  virtual ~Serial3() = default;
  virtual bool Initialize() = 0;
  virtual bool Read(Packet REF_OUT data, short size) = 0;
  virtual bool Write(Packet REF_IN data, short size) = 0;
};

/// 物理串口收发
class PhySerial3 final : public Serial3 {
 public:
  PhySerial3() = default;
  ~PhySerial3() override;
  bool Initialize() override;
  bool Read(srm::message::Packet REF_OUT data, short size) override;
  bool Write(Packet REF_IN data, short size) override;

 private:
  std::string serial_port_;
  int serial_fd_{};
};

/// 虚拟串口收发
class SimSerial3 final : public Serial3 {
 public:
  SimSerial3() = default;
  ~SimSerial3() override = default;
  bool Initialize() override;
  bool Read(Packet REF_OUT data, short size) override;
  bool Write(Packet REF_IN data, short size) override;
};

/// 上下位机通信类
class StmMessage3 final : public BaseMessage {
  inline static auto registry = RegistrySub<BaseMessage, StmMessage3>("stm323");

 public:
  StmMessage3() = default;
  ~StmMessage3() override = default;
  bool Initialize(const std::string&config_path) override;
  bool Connect(bool flag) override;
  bool Send() override;
  bool Receive() override;

 private:
  std::unique_ptr<Serial3> serial_;
};

PhySerial3::~PhySerial3() { close(serial_fd_); }

bool PhySerial3::Initialize() {
  FILE *ls = (kSystem == "macos" ? popen("ls --color=never /dev/cu.usb*", "r")
                                 : popen("ls --color=never /dev/ttyACM*", "r"));
  char name[127];
  auto ret = fscanf(ls, "%s", name);
  pclose(ls);
  if (ret == -1) {
    tools::logger()->error("No UART device found.");
    return false;
  } else {
    serial_port_ = name;
  }

  if (kSystem == "linux" && chmod(serial_port_.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) == -1) {
    tools::logger()->error("Running in user mode, manually setting permission is required.");
    tools::logger()->error("$ sudo chmod 777 {}", serial_port_);
  }

  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ == -1) {
    tools::logger()->error("Failed to open serial port {}", serial_port_);
    return false;
  }

  termios termios{};
  tcgetattr(serial_fd_, &termios);
  cfmakeraw(&termios);
  cfsetispeed(&termios, B115200);
  tcsetattr(serial_fd_, TCSANOW, &termios);
  tools::logger()->info("Serial port {} is open." , serial_port_);
  return true;
}

bool PhySerial3::Read(Packet REF_OUT data, short size) {
  using namespace std::chrono_literals;
  std::vector<char> buffer(sizeof(short) + size);
  auto start_time = std::chrono::high_resolution_clock::now();

  char *ptr = buffer.data();
  int cnt = 0;

  while (cnt < sizeof(short)) {
    if (std::chrono::high_resolution_clock::now() - start_time >= 3s) {
      return false;
    }
    int bytes_read = read(serial_fd_, ptr + cnt, sizeof(short) - cnt);
    if (bytes_read > 0) cnt += bytes_read;
    else if (bytes_read <= 0) std::this_thread::sleep_for(1ms);
  }

  short received_size = *(short *)ptr;
  if (size != received_size) {
    tcflush(serial_fd_, TCIFLUSH);
    return false;
  }

  while (cnt < buffer.size()) {
    if (std::chrono::high_resolution_clock::now() - start_time >= 3s) {
      return false;
    }
    int bytes_read = read(serial_fd_, ptr + cnt, buffer.size() - cnt);
    if (bytes_read > 0) cnt += bytes_read;
    else if (bytes_read <= 0) std::this_thread::sleep_for(1ms);
  }

  data.Resize(size);
  memcpy(data.Ptr(), ptr + sizeof(short), size);
  return true;
}

bool PhySerial3::Write(Packet REF_IN data, short size) {
  std::vector<char> buffer(sizeof(short) + size);
  memcpy(buffer.data(), &size, sizeof(short));
  memcpy(buffer.data() + sizeof(short), data.Ptr(), data.Size());

  int cnt = write(serial_fd_, buffer.data(), buffer.size());
  return cnt == buffer.size();
}

extern "C" {
void InitParam3();
static int8_t CdcReceiveFs(uint8_t *Buf, uint32_t *Len);
char buffer3[256];
short buffer_size3 = 0;

// Vision通信使用的数据结构
typedef struct {
  float yaw;
  float pitch;
  float roll;
  int mode;
  int color;
} VisionGimbalReceive;

typedef struct {
  float bullet_speed;
} VisionShootReceive;

typedef struct {
  float yaw;
  float pitch;
} VisionGimbalSend;

typedef struct {
  int fire_flag;
} VisionShootSend;

typedef struct {
  void *ptr_list[32];
  short size_list[32];
} Message;

VisionGimbalReceive vision_gimbal_recv3;
VisionShootReceive vision_shoot_recv3;
VisionGimbalSend vision_gimbal_send3;
VisionShootSend vision_shoot_send3;

Message receive3;
Message send3;
short receive_size3 = 0;
short send_size3 = 0;
short send_id_list3[32];
short send_id_num3 = 0;

void InitParam3(void) {
#define RIGISTER_ID(data, id, packet) \
    data.ptr_list[id] = &(packet);    \
    data.size_list[id] = sizeof(packet);

    // STM32接收vision发来的数据
    RIGISTER_ID(receive3, 1, vision_gimbal_send3);
    RIGISTER_ID(receive3, 2, vision_shoot_send3);

    // STM32发送给vision的数据
    RIGISTER_ID(send3, 1, vision_gimbal_recv3);
    RIGISTER_ID(send3, 2, vision_shoot_recv3);
}

static int8_t CdcReceiveFs(uint8_t *Buf, uint32_t *Len) {
  if (!Buf) return -1;

  short buf_pos = 0;
  char *ptr;

  if (receive_size3 == 0) {
    if (buf_pos + sizeof(short) > 64) return -1;
    memcpy(&receive_size3, Buf + buf_pos, sizeof(short));
    buf_pos += sizeof(short);
    buffer_size3 = 0;
    if (receive_size3 <= 0 || receive_size3 > 1024) {
      receive_size3 = 0;
      return -1;
    }
  }

  short remain_size = receive_size3 - buffer_size3;
  if (remain_size > 0) {
    short copy_size = (remain_size > 64 - buf_pos) ? (64 - buf_pos) : remain_size;
    if (buffer_size3 + copy_size > sizeof(buffer3)) {
      receive_size3 = 0;
      buffer_size3 = 0;
      return -1;
    }
    memcpy(buffer3 + buffer_size3, Buf + buf_pos, copy_size);
    buffer_size3 += copy_size;
  }

  if (receive_size3 != buffer_size3) return 0;

  ptr = buffer3;
  while (ptr < buffer3 + buffer_size3) {
    if (ptr + sizeof(short) > buffer3 + buffer_size3) break;

    short id;
    memcpy(&id, ptr, sizeof(short));
    ptr += sizeof(short);

    if (id == -1) {
      receive_size3 = 0;
      return 0;
    } else if (id == 0) {
      send_id_num3 = 0;
      send_size3 = 0;
      while (ptr < buffer3 + buffer_size3) {
        if (ptr + sizeof(short) > buffer3 + buffer_size3) break;
        memcpy(&id, ptr, sizeof(short));
        ptr += sizeof(short);
        if (id < 0 || id >= 32) continue;
        send_id_list3[send_id_num3++] = id;
        send_size3 += send3.size_list[id] + sizeof(short);
        if (send_id_num3 >= 32) break;
      }
    } else {
      if (id < 0 || id >= 32) break;
      if (ptr + receive3.size_list[id] > buffer3 + buffer_size3) break;
      memcpy(receive3.ptr_list[id], ptr, receive3.size_list[id]);
      ptr += receive3.size_list[id];
    }
  }

  buffer_size3 = sizeof(short) + send_size3;
  if (buffer_size3 > sizeof(buffer3)) {
    receive_size3 = 0;
    return -1;
  }

  ptr = buffer3;
  memcpy(ptr, &send_size3, sizeof(short));
  ptr += sizeof(short);

  for (int i = 0; i < send_id_num3 && i < 32; i++) {
    short id = send_id_list3[i];
    if (id < 0 || id >= 32 || !send3.ptr_list[id]) continue;
    if (ptr + sizeof(short) + send3.size_list[id] > buffer3 + sizeof(buffer3)) break;
    memcpy(ptr, &id, sizeof(short));
    ptr += sizeof(short);
    memcpy(ptr, send3.ptr_list[id], send3.size_list[id]);
    ptr += send3.size_list[id];
  }

  receive_size3 = 0;
  return 0;
}
}

bool SimSerial3::Initialize() {
  InitParam3();
  return true;
}

bool SimSerial3::Read(Packet REF_OUT data, short size) {
  if (buffer_size3 < sizeof(short)) return false;
  short received_size = *(short *)buffer3;
  if (received_size != size) return false;
  if (buffer_size3 < sizeof(short) + size) return false;

  data.Resize(size);
  memcpy(data.Ptr(), buffer3 + sizeof(short), size);
  return true;
}

bool SimSerial3::Write(Packet REF_IN data, short size) {
  std::vector<char> buffer;
  buffer.insert(buffer.begin(), (char *)&size, (char *)&size + sizeof(short));

  auto data_clone = data;
  while (!data_clone.Empty()) {
    char c;
    if (!data_clone.Read(c)) break;
    buffer.push_back(c);
    if (buffer.size() >= 64) {
      CdcReceiveFs((uint8_t *)buffer.data(), nullptr);
      buffer.clear();
    }
  }
  if (!buffer.empty()) {
    CdcReceiveFs((uint8_t *)buffer.data(), nullptr);
  }
  return true;
}

bool StmMessage3::Initialize(const std::string & config_path) {
  auto yaml = tools::load(config_path);
  auto type = tools::read<std::string>(yaml,"serial_type");
  type == "physical" ? serial_ = std::make_unique<PhySerial3>() : serial_ = std::make_unique<SimSerial3>();
  if (!serial_ || !serial_->Initialize()) {
    tools::logger()->error("Failed to initialize serial.");
    return false;
  }
  return true;
}

bool StmMessage3::Connect(bool flag) {
  if (flag) {
    send_buffer_.Write((short)0);
    for (auto &[_, it] : receive_registry_) {
      send_buffer_.Write(it.first);
    }
  } else {
    send_buffer_.Write((short)-1);
  }
  if (!serial_->Write(send_buffer_, send_buffer_.Size())) {
    send_buffer_.Clear();
    tools::logger()->error("Failed to establish connection.");
    return false;
  }
  send_buffer_.Clear();
  return true;
}

bool StmMessage3::Send() {
  if (!serial_->Write(send_buffer_, send_size_)) {
    tools::logger()->error("Failed to send data to serial.");
    send_buffer_.Clear();
    return false;
  }
  send_buffer_.Clear();
  return true;
}

bool StmMessage3::Receive() {
  if (!serial_->Read(receive_buffer_, receive_size_)) {
    tools::logger()->warn( "Failed to read data from serial.");
    receive_buffer_.Clear();
    return false;
  }

  char *ptr = receive_buffer_.Ptr();
  while (ptr < receive_buffer_.Ptr() + receive_buffer_.Size()) {
    short id = *(short *)ptr;
    ptr += sizeof(short);
    auto &packet = packet_received_[id];
    memcpy(packet.Ptr(), ptr, packet.Size());
    ptr += packet.Size();
  }
  return true;
}

}  // namespace srm::message
namespace io
{
DM_IMU::DM_IMU() : queue_(5000)
{
  init_serial();
  rec_thread_ = std::thread(&DM_IMU::get_imu_data_thread, this);
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
  tools::logger()->info("[DM_IMU] initialized");
}

DM_IMU::~DM_IMU()
{
  stop_thread_ = true;
  if (rec_thread_.joinable()) {
    rec_thread_.join();
  }
  message_->Connect(false);
}

void DM_IMU::init_serial()
{
  auto yaml = tools::load("configs/standard3.yaml");
  try {
    message_.reset(srm::message::CreateMessage("stm323"));
    if (!message_) {
      tools::logger()->error("Failed to create message");
      return ;
    }
    if (!message_->Initialize("configs/standard3.yaml")) {
      tools::logger()->error("Failed to initialize message");
      return ;
    }
    message_->ReceiveRegister<srm::message::GimbalReceive>(tools::read<short>(yaml, "receive.gimbal"));
    message_->ReceiveRegister<srm::message::ShootReceive>(tools::read<short>(yaml, "receive.shoot"));
    message_->SendRegister<srm::message::GimbalSend>(tools::read<short>(yaml, "send.gimbal"));
    message_->SendRegister<srm::message::ShootSend>(tools::read<short>(yaml, "send.shoot"));
    message_->Connect(true);
    receive_packet = new srm::message::ReiceivePacket();
  } catch (const std::exception & e) {
    tools::logger()->error("[IMU] Failed to open serial: {}", e.what());
    exit(1);
  }
}

void DM_IMU::get_imu_data_thread()
{
  while (!stop_thread_) {
    if (message_) {
      message_->Receive();
      // 读取成功，更新数据
      message_->ReadData(gimbal_receive);
      message_->ReadData(shoot_receive);
      receive_packet->yaw = gimbal_receive.yaw;
      receive_packet->pitch = gimbal_receive.pitch;
      receive_packet->roll = gimbal_receive.roll;
      receive_packet->mode = gimbal_receive.mode;
      receive_packet->color = gimbal_receive.color;
      receive_packet->bullet_speed = shoot_receive.bullet_speed;
      //tools::logger()->info("Received packet: yaw= {:.1f} pitch= {:.1f} roll= {:.1f} mode= {} color= {} bullet_speed= {:.1f}",
      //gimbal_receive.yaw, gimbal_receive.pitch, gimbal_receive.roll,gimbal_receive.mode, gimbal_receive.color, shoot_receive.bullet_speed);

      // 解算yaw, pitch, roll为四元数
      auto yaw_ = receive_packet->yaw * M_PI / 180;
      auto pitch_ = receive_packet->pitch * M_PI / 180;
      auto roll_ = receive_packet->roll * M_PI / 180;
      Eigen::Vector3d ypr(yaw_, pitch_, roll_);
      Eigen::Quaterniond q;
      q = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());
      q.normalize();
      //推入数据队列
      auto timestamp = std::chrono::steady_clock::now();
      queue_.push({q, timestamp});
    }
  }
}

Eigen::Quaterniond DM_IMU::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  // 四元数插值
  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

}  // namespace io
