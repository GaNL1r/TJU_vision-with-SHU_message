#ifndef SRM_MESSAGE_INFO_H_
#define SRM_MESSAGE_INFO_H_
namespace srm::message {

/// 发送的云台数据
struct GimbalSend {
  float yaw;    ///< 绝对yaw角度
  float pitch;  ///< 绝对pitch角度
};

/// 接收的云台数据
struct GimbalReceive {
  float yaw;    ///< 当前绝对yaw角度
  float pitch;  ///< 当前绝对pitch角度
  float roll;   ///< 当前绝对roll角度
  int mode;     ///< 自瞄模式 0装甲板 1小能量机关 2大能量机关
  int color;    ///< 颜色 0红色 1蓝色 2灰色 3紫色，自身的颜色也是控制传过来的
};

/// 发送给打弹的数据
struct ShootSend {
  int fire_flag;  ///< 是否开火
};

/// 接收的打弹数据
struct ShootReceive {
  float bullet_speed;  ///< 弹速
};

/// 合并的接收数据
struct ReiceivePacket {
  float yaw;
  float pitch;
  float roll;
  int mode;            ///< 自瞄模式 0装甲板 1小能量机关 2大能量机关
  int color;           ///< 颜色 0红色 1蓝色 2灰色 3紫色
  float bullet_speed;  ///< 弹速
};

}  // namespace srm::message

#endif
