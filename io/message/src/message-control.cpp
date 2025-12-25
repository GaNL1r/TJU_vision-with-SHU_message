#include <sys/fcntl.h>
#include <sys/mman.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <sys/stat.h>

#include <mutex>
#include "tools/yaml.hpp"
#include "io/message/include/srm/message/message-base.hpp"
#include "io/message/include/srm/message/packet.hpp"
#include "io/message/include/srm/common/include/srm/common.hpp"
namespace srm::message {
/// 实际上就是创建一个文件，在这个文件内共同读写
class Shm {
 public:
  Shm() = default;
  ~Shm() = default;
  bool Initialize(const std::string&config_path);
  bool Read(Packet REF_OUT data, int size);
  bool Write(Packet REF_IN data, int size);
  void Connect(bool flag) const;

 private:
  bool Lock(int fd);    ///< P操作
  bool Unlock(int fd);  ///< V操作

  const char* key_{nullptr};  ///< 共享内存的key
  int shm_fd_{-1};            ///< 共享内存的文件描述符
  char* shm_ptr_{nullptr};    ///< 共享内存起始地址的指针
  char* read_ptr_{nullptr};   ///< 读区域头指针
  char* write_ptr_{nullptr};  ///< 写区域头指针
  char* flag_ptr_{nullptr};   ///< 连接建立标志指针
  int read_fd_{-1};           ///< 读信号量描述符
  int write_fd_{-1};          ///< 写信号量描述符
};

// class SimShm final : public Shm {};

class ControlMessage final : public BaseMessage {
  inline static auto registry = RegistrySub<BaseMessage, ControlMessage>("control");

 public:
  ControlMessage() = default;
  ~ControlMessage() override;
  bool Initialize() override;
  bool Send() override;
  bool Receive() override;
  bool Connect(bool flag) override;

 private:
  std::unique_ptr<Shm> shm_;
};

bool Shm::Initialize(const std::string & config_path) {
  auto yaml = tools::load(config_path);
  const auto shm_name = tools::read<std::string>(yaml, "shm_name");
  /// read和write都有唯一键值，用来标示
  const auto read_sem = tools::read<int>(yaml, "read_sem");
  const auto write_sem = tools::read<int>(yaml, "write_sem");
  const auto shm_size = tools::read<int>(yaml, "shm_size");
  key_ = shm_name.c_str();
  /// 打开control那边打开的共享内存，control那边oflag有create权限
  shm_fd_ = shm_open(key_, O_RDWR, 0666);
  if (shm_fd_ == -1) {
    LOG(ERROR) << "Failed to open shared memory.";
    return false;
  }

  /// 映射到本进程空间
  shm_ptr_ = static_cast<char*>(mmap(nullptr, shm_size, PROT_WRITE | PROT_READ, MAP_SHARED, shm_fd_, 0));
  if (shm_ptr_ == MAP_FAILED) {
    LOG(ERROR) << "Failed to map shared memory.";
    return false;
  }
  LOG(INFO) << "Open shared memory. Address is " << reinterpret_cast<int*>(shm_ptr_);
  /// 前一半给write，后一半给read
  read_ptr_ = shm_ptr_ + shm_size / 2;
  write_ptr_ = shm_ptr_;
  /// 末尾标志
  flag_ptr_ = shm_ptr_ + shm_size - 1;
  /// 获取对应标识符，即key对应的标示数id，其实差不多跟mutex差不多
  read_fd_ = semget(read_sem, 1, 0);
  write_fd_ = semget(write_sem, 1, 0);
  return true;
}

bool Shm::Read(Packet REF_OUT data, const int size) {
  data.Resize(size);
  Lock(read_fd_);
  /// 如果想要读的东西错了
  if (size != *reinterpret_cast<int*>(read_ptr_)) {
    LOG(ERROR) << "Receiving packet size is " << *reinterpret_cast<int*>(read_ptr_) << ", expecting " << size << ".";
    Unlock(read_fd_);
    return false;
  }
  memcpy(data.Ptr(), read_ptr_ + sizeof(int), size);
  Unlock(read_fd_);
  return true;
}

bool Shm::Write(Packet REF_IN data, const int size) {
  Lock(write_fd_);
  memcpy(write_ptr_, &size, sizeof(int));
  memcpy(write_ptr_ + sizeof(int), data.Ptr(), size);
  Unlock(write_fd_);
  return true;
}

void Shm::Connect(const bool flag) const {
  /// 关闭就是假装自己所有的内存都不管了
  if (flag) {
    *flag_ptr_ = static_cast<char>(0xff);
  } else {
    *flag_ptr_ = static_cast<char>(0x00);
  }
}

bool Shm::Lock(const int fd) {
  /// 可用计数-1
  sembuf sem{0, -1, 0};
  if (semop(fd, &sem, 1) == -1) {
    LOG(ERROR) << "Failed to lock semaphore " << fd;
    return false;
  }
  return true;
}

bool Shm::Unlock(const int fd) {
  /// 可用计数+1
  sembuf sem{0, 1, 0};
  if (semop(fd, &sem, 1) == -1) {
    LOG(ERROR) << "Failed to unlock semaphore " << fd;
    return false;
  }
  return true;
}

ControlMessage::~ControlMessage() { Connect(false); }

bool ControlMessage::Initialize() {
  shm_ = std::make_unique<Shm>();
  if (!shm_) {
    LOG(ERROR) << "Failed to create shared memory.";
    return false;
  }
  if (!shm_->Initialize({})) {
    LOG(ERROR) << "Failed to initialize shared memory.";
    send_buffer_.Clear();
    return false;
  }
  return true;
}

bool ControlMessage::Send() {
  ///!感觉这边有很多判断都有点冗余
  if (!shm_->Write(send_buffer_, send_size_)) {
    LOG(ERROR) << "Failed to send data to shared memory.";
    send_buffer_.Clear();
    return false;
  }
  send_buffer_.Clear();
  return true;
}

bool ControlMessage::Receive() {
  /// 将所有buffer读进来
  if (!shm_->Read(receive_buffer_, receive_size_)) {
    LOG(ERROR) << "Failed to read data from memory.";
    receive_buffer_.Clear();
    return false;
  }
  /// 根据registry的设定，根据id一个一个读
  char* ptr = receive_buffer_.Ptr();
  while (ptr < receive_buffer_.Ptr() + receive_buffer_.Size()) {
    short id = *reinterpret_cast<short*>(ptr);
    ptr += sizeof(short);
    auto& packet = packet_received_[id];
    memcpy(packet.Ptr(), ptr, packet.Size());
    ptr += packet.Size();
  }
  return true;
}

bool ControlMessage::Connect(const bool flag) {
  if (flag) {
    /// 连接的时候先往里面传一点数据
    send_buffer_.Resize(send_size_);
    if (!shm_->Write(send_buffer_, send_size_)) {
      LOG(ERROR) << "Failed to connect to control.";
      send_buffer_.Clear();
      return false;
    }
    send_buffer_.Clear();
  }
  shm_->Connect(flag);
  return true;
}

}  // namespace srm::message