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
class Serial {
 public:
  Serial() = default;
  virtual ~Serial() = default;
  virtual bool Initialize() = 0;
  virtual bool Read(Packet REF_OUT data, short size) = 0;
  virtual bool Write(Packet REF_IN data, short size) = 0;
};

/// 物理串口收发
class PhySerial final : public Serial {
 public:
  PhySerial() = default;
  ~PhySerial() override;
  bool Initialize() override;
  bool Read(srm::message::Packet REF_OUT data, short size) override;
  bool Write(Packet REF_IN data, short size) override;

 private:
  std::string serial_port_;
  int serial_fd_{};
};

/// 虚拟串口收发
class SimSerial final : public Serial {
 public:
  SimSerial() = default;
  ~SimSerial() override = default;
  bool Initialize() override;
  bool Read(Packet REF_OUT data, short size) override;
  bool Write(Packet REF_IN data, short size) override;
};

/// 上下位机通信类
class StmMessage final : public BaseMessage {
  inline static auto registry = RegistrySub<BaseMessage, StmMessage>("stm32");

 public:
  StmMessage() = default;
  ~StmMessage() override = default;
  bool Initialize(const std::string&config_path) override;
  bool Connect(bool flag) override;
  bool Send() override;
  bool Receive() override;

 private:
  std::unique_ptr<Serial> serial_;
};

PhySerial::~PhySerial() { close(serial_fd_); }

bool PhySerial::Initialize() {
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

bool PhySerial::Read(Packet REF_OUT data, short size) {
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

bool PhySerial::Write(Packet REF_IN data, short size) {
  std::vector<char> buffer(sizeof(short) + size);
  memcpy(buffer.data(), &size, sizeof(short));
  memcpy(buffer.data() + sizeof(short), data.Ptr(), data.Size());

  int cnt = write(serial_fd_, buffer.data(), buffer.size());
  return cnt == buffer.size();
}

extern "C" {
void InitParam();
static int8_t CdcReceiveFs(uint8_t *Buf, uint32_t *Len);
char buffer[256];
short buffer_size = 0;

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

VisionGimbalReceive vision_gimbal_recv;
VisionShootReceive vision_shoot_recv;
VisionGimbalSend vision_gimbal_send;
VisionShootSend vision_shoot_send;

Message receive;
Message send;
short receive_size = 0;
short send_size = 0;
short send_id_list[32];
short send_id_num = 0;

void InitParam(void) {
#define RIGISTER_ID(data, id, packet) \
    data.ptr_list[id] = &(packet);    \
    data.size_list[id] = sizeof(packet);

    // STM32接收vision发来的数据
    RIGISTER_ID(receive, 1, vision_gimbal_send);
    RIGISTER_ID(receive, 2, vision_shoot_send);

    // STM32发送给vision的数据
    RIGISTER_ID(send, 1, vision_gimbal_recv);
    RIGISTER_ID(send, 2, vision_shoot_recv);
}

static int8_t CdcReceiveFs(uint8_t *Buf, uint32_t *Len) {
  if (!Buf) return -1;

  short buf_pos = 0;
  char *ptr;

  if (receive_size == 0) {
    if (buf_pos + sizeof(short) > 64) return -1;
    memcpy(&receive_size, Buf + buf_pos, sizeof(short));
    buf_pos += sizeof(short);
    buffer_size = 0;
    if (receive_size <= 0 || receive_size > 1024) {
      receive_size = 0;
      return -1;
    }
  }

  short remain_size = receive_size - buffer_size;
  if (remain_size > 0) {
    short copy_size = (remain_size > 64 - buf_pos) ? (64 - buf_pos) : remain_size;
    if (buffer_size + copy_size > sizeof(buffer)) {
      receive_size = 0;
      buffer_size = 0;
      return -1;
    }
    memcpy(buffer + buffer_size, Buf + buf_pos, copy_size);
    buffer_size += copy_size;
  }

  if (receive_size != buffer_size) return 0;

  ptr = buffer;
  while (ptr < buffer + buffer_size) {
    if (ptr + sizeof(short) > buffer + buffer_size) break;

    short id;
    memcpy(&id, ptr, sizeof(short));
    ptr += sizeof(short);

    if (id == -1) {
      receive_size = 0;
      return 0;
    } else if (id == 0) {
      send_id_num = 0;
      send_size = 0;
      while (ptr < buffer + buffer_size) {
        if (ptr + sizeof(short) > buffer + buffer_size) break;
        memcpy(&id, ptr, sizeof(short));
        ptr += sizeof(short);
        if (id < 0 || id >= 32) continue;
        send_id_list[send_id_num++] = id;
        send_size += send.size_list[id] + sizeof(short);
        if (send_id_num >= 32) break;
      }
    } else {
      if (id < 0 || id >= 32) break;
      if (ptr + receive.size_list[id] > buffer + buffer_size) break;
      memcpy(receive.ptr_list[id], ptr, receive.size_list[id]);
      ptr += receive.size_list[id];
    }
  }

  buffer_size = sizeof(short) + send_size;
  if (buffer_size > sizeof(buffer)) {
    receive_size = 0;
    return -1;
  }

  ptr = buffer;
  memcpy(ptr, &send_size, sizeof(short));
  ptr += sizeof(short);

  for (int i = 0; i < send_id_num && i < 32; i++) {
    short id = send_id_list[i];
    if (id < 0 || id >= 32 || !send.ptr_list[id]) continue;
    if (ptr + sizeof(short) + send.size_list[id] > buffer + sizeof(buffer)) break;
    memcpy(ptr, &id, sizeof(short));
    ptr += sizeof(short);
    memcpy(ptr, send.ptr_list[id], send.size_list[id]);
    ptr += send.size_list[id];
  }

  receive_size = 0;
  return 0;
}
}

bool SimSerial::Initialize() {
  InitParam();
  return true;
}

bool SimSerial::Read(Packet REF_OUT data, short size) {
  if (buffer_size < sizeof(short)) return false;
  short received_size = *(short *)buffer;
  if (received_size != size) return false;
  if (buffer_size < sizeof(short) + size) return false;

  data.Resize(size);
  memcpy(data.Ptr(), buffer + sizeof(short), size);
  return true;
}

bool SimSerial::Write(Packet REF_IN data, short size) {
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

bool StmMessage::Initialize(const std::string & config_path) {
  auto yaml = tools::load(config_path);
  auto type = tools::read<std::string>(yaml,"serial_type");
  type == "physical" ? serial_ = std::make_unique<PhySerial>() : serial_ = std::make_unique<SimSerial>();
  if (!serial_ || !serial_->Initialize()) {
    tools::logger()->error("Failed to initialize serial.");
    return false;
  }
  return true;
}

bool StmMessage::Connect(bool flag) {
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

bool StmMessage::Send() {
  if (!serial_->Write(send_buffer_, send_size_)) {
    tools::logger()->error("Failed to send data to serial.");
    send_buffer_.Clear();
    return false;
  }
  send_buffer_.Clear();
  return true;
}

bool StmMessage::Receive() {
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