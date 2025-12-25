#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

#include <io/message/src/stm-message.cpp>
using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";
struct Data
{
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point timestamp;
};
Data data_behind_;
Data data_ahead_;
tools::ThreadSafeQueue<Data> queue_(5000);
int main(int argc, char * argv[])
{
  std::shared_ptr<srm::message::BaseMessage> message_;
  message_.reset(srm::message::CreateMessage("stm32"));

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  auto yaml = tools::load(config_path);

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::Camera camera(config_path);
  if (!message_) {
    tools::logger()->error("Failed to create message");
    return 1;
  }
  if (!message_->Initialize(config_path)) {
    tools::logger()->error("Failed to initialize message");
    return 1;
  }
  message_->ReceiveRegister<srm::message::GimbalReceive>(tools::read<short>(yaml, "receive.gimbal"));
  message_->ReceiveRegister<srm::message::ShootReceive>(tools::read<short>(yaml, "receive.shoot"));
  message_->SendRegister<srm::message::GimbalSend>(tools::read<short>(yaml, "send.gimbal"));
  message_->SendRegister<srm::message::ShootSend>(tools::read<short>(yaml, "send.shoot"));
  message_->Connect(true);
  auto *receive_packet = new srm::message::ReiceivePacket();
  message_->Receive();
  srm::message::GimbalReceive gimbal_receive{};
  srm::message::ShootReceive shoot_receive{};
  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;
  auto callback = [&](){
    auto timestamp = std::chrono::steady_clock::now();
    if (!message_->ReadData(gimbal_receive) || !message_->ReadData(shoot_receive)) {
      tools::logger()->warn("Failed to read data from STM32. Using simulator data.");
      const std::string prefix = "message.simulator";
      receive_packet->yaw = tools::read<float>(yaml, "yaw");
      receive_packet->pitch = tools::read<float>(yaml,  "pitch");
      receive_packet->roll = tools::read<float>(yaml,  "roll");
      receive_packet->bullet_speed = tools::read<float>(yaml,  "bullet_speed");
      receive_packet->mode = tools::read<int>(yaml,  "mode");
      receive_packet->color = tools::read<int>(yaml,  "color");
      tools::logger()->info("Received packet: yaw= {:.1f} pitch= {:.1f} roll= {:.1f} mode= {} color= {} bullet_speed= {:.1f}",
        gimbal_receive.yaw, gimbal_receive.pitch, gimbal_receive.roll,
        gimbal_receive.mode, gimbal_receive.color, shoot_receive.bullet_speed);
    } else {
      receive_packet->yaw = gimbal_receive.yaw;
      receive_packet->pitch = gimbal_receive.pitch;
      receive_packet->roll = gimbal_receive.roll;
      receive_packet->mode = gimbal_receive.mode;
      receive_packet->color = gimbal_receive.color;
      receive_packet->bullet_speed = shoot_receive.bullet_speed;
      tools::logger()->info("Received packet: yaw= {:.1f} pitch= {:.1f} roll= {:.1f} mode= {} color= {} bullet_speed= {:.1f}",
            gimbal_receive.yaw, gimbal_receive.pitch, gimbal_receive.roll,
            gimbal_receive.mode, gimbal_receive.color, shoot_receive.bullet_speed);
    }
    Eigen::Vector3d ypr(receive_packet->yaw, receive_packet->pitch, receive_packet->roll);
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());
    queue_.push({q, timestamp});
    mode = static_cast<io::Mode>(receive_packet->mode);
  };
  callback();
  data_behind_ = queue_.front();
  data_ahead_ = queue_.front();
  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto imu_at = [&](std::chrono::steady_clock::time_point timestamp){
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
  };
  while (!exiter.exit()) {
    camera.read(img, t);
    callback();
    q = imu_at(t - 1ms);

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    solver.set_R_gimbal2world(q);

    auto armors = detector.detect(img);

    auto targets = tracker.track(armors, t);

    auto command = aimer.aim(targets, t, 30);

    auto yaw_ = static_cast<float>(command.yaw);
    auto pitch_ = static_cast<float>(command.pitch);
    auto Is_fire = command.shoot;
    const srm::message::GimbalSend gimbal_send{yaw_, pitch_};
    const srm::message::ShootSend shoot_send{Is_fire};
    message_->WriteData(gimbal_send);
    message_->WriteData(shoot_send);
    message_->Send();
  }

  return 0;
}