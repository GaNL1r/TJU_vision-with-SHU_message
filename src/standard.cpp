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


/* 替换掉同济的上下位机通信代码，使用stm32的串口通信，但是数据结构保留一致，不然同济代码没法解算 */
struct Data
{
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point timestamp;
}; ///< 四元数和时间戳
Data data_behind_; ///< 缓存下一帧的IMU数据
Data data_ahead_; ///< 缓存上一帧的IMU数据
tools::ThreadSafeQueue<Data> queue_(5000); ///< 队列大小为5000的帧数据

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

  // 初始化串口通信模块
  if (!message_) {
    tools::logger()->error("Failed to create message");
    return 1;
  }
  if (!message_->Initialize(config_path)) {
    tools::logger()->error("Failed to initialize message");
    return 1;
  }
  // 注册接收和发送消息类型
  message_->ReceiveRegister<srm::message::GimbalReceive>(tools::read<short>(yaml, "receive.gimbal"));
  message_->ReceiveRegister<srm::message::ShootReceive>(tools::read<short>(yaml, "receive.shoot"));
  message_->SendRegister<srm::message::GimbalSend>(tools::read<short>(yaml, "send.gimbal"));
  message_->SendRegister<srm::message::ShootSend>(tools::read<short>(yaml, "send.shoot"));
  // 连接串口
  message_->Connect(true);
  // 通信内存分配
  auto *receive_packet = new srm::message::ReiceivePacket();
  message_->Receive();
  srm::message::GimbalReceive gimbal_receive{};
  srm::message::ShootReceive shoot_receive{};

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;
  /* 帧回调函数注册 */
  auto callback = [&](){
    auto timestamp = std::chrono::steady_clock::now();
    // 尝试接收数据
    message_->Receive();
    if (!message_->ReadData(gimbal_receive) || !message_->ReadData(shoot_receive)) {
      // 读取失败，使用仿真数据
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
      // 读取成功，更新数据
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
    // 解算yaw, pitch, roll为四元数
    auto yaw_ = receive_packet->yaw * M_PI / 180;
    auto pitch_ = receive_packet->pitch * M_PI / 180;
    auto roll_ = receive_packet->roll * M_PI / 180;
    Eigen::Vector3d ypr(yaw_, pitch_, roll_);
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());
    //推入数据队列
    queue_.push({q, timestamp});
    // 打弹模式更新
    mode = static_cast<io::Mode>(receive_packet->mode);
  };
  // 初始化尝试
  callback();
  data_behind_ = queue_.front();
  data_ahead_ = queue_.front();

  // 同济代码模块初始化
  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  /* 四元数插值函数，计算在指定时间戳处的四元数 */
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

  // 程序主循环
  while (!exiter.exit()) {
    // 读取图像和时间戳
    camera.read(img, t);
    // 接收下位机数据
    callback();
    // 四元数插值
    q = imu_at(t - 1ms);

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    solver.set_R_gimbal2world(q);

    auto armors = detector.detect(img);

    auto targets = tracker.track(armors, t);

    auto command = aimer.aim(targets, t, 10000);

    // 向下位机发送指令
    auto yaw_ = static_cast<float>(command.yaw);
    auto pitch_ = static_cast<float>(command.pitch);
    yaw_ = yaw_ * 180.0 / M_PI;
    pitch_ = pitch_ * 180.0 / M_PI;
    tools::logger()->info("send:{},{}",yaw_,pitch_);
    auto Is_fire = command.shoot;
    const srm::message::GimbalSend gimbal_send{yaw_, 0.0};
    const srm::message::ShootSend shoot_send{Is_fire};
    message_->WriteData(gimbal_send);
    message_->WriteData(shoot_send);
    //message_->Send();

    tools::draw_text(
      img,
      fmt::format(
        "command is {},{:.2f},{:.2f},shoot:{}", command.control, command.yaw * 57.3,
        command.pitch * 57.3, command.shoot),
      {10, 60}, {154, 50, 205});

    Eigen::Quaternion gimbal_q = q;
    tools::draw_text(
      img,
      fmt::format(
        "gimbal yaw{:.2f}", (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0]),
      {10, 90}, {255, 255, 255});

    nlohmann::json data;

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto & armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
      data["armor_center_x"] = armor.center_norm.x;
      data["armor_center_y"] = armor.center_norm.y;
    }

    auto yaw = tools::eulers(q, 2, 1, 0)[0];
    data["gimbal_yaw"] = yaw * 57.3;
    data["cmd_yaw"] = command.yaw * 57.3;
    data["shoot"] = command.shoot;

    if (!targets.empty()) {
      auto target = targets.front();

      std::vector<Eigen::Vector4d> armor_xyza_list;

      // 当前帧target更新后
      armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;

      // 卡方检验数据
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
    }

    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(30);
    if (key == 'q') break;
  }

  return 0;
}
