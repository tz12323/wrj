#ifndef __CONNECT_UVA_H
#define __CONNECT_UVA_H
#include "pid_t.hpp"
#include "serial_helper.hpp"
#include <atomic>
#include <mutex>
#include <queue>
class UPUavControl {
public:
  using Data = std::vector<uint8_t>;
  std::queue<Data> msg_list;
  std::atomic<bool> isFly{false};
  uint8_t settingHeight = 0;
  // 飞机油门杆量死区
  int dead_area = 50;
  // 飞机最小上升杆量
  int min_up = 53;
  // 飞机最小下降杆量
  int min_down = 47;
  // 当前高度
  std::atomic<int> current_height{0};
  UPUavControl(std::mutex &lock, std::string Port = "/dev/ttyUSB0",
               uint32_t BaudRate = 115200, uint8_t ByteSize = 8,
               char Parity = 'N', uint8_t Stopbits = 1);
  ~UPUavControl() noexcept;
  void myserial_on_connected_changed(bool is_connected);
  // void write(Data data);
  void send_msg();
  std::tuple<Data, uint16_t> generateCmd(uint8_t device, uint8_t cmd,
                                         uint8_t len, Data data);
  void setMoveAction(int16_t y, int16_t x, int16_t z, int16_t yaw);
  int get_current_height();
  void setServoPosition(uint16_t angel); // 控制云台舵机运动角度，角度区间0-90
  void move_forward(int16_t speed);
  void move_backward(int16_t speed);
  void move_left(int16_t speed);
  void move_right(int16_t speed);
  void move_up(int16_t speed);
  void move_down(int16_t speed);
  void move(int16_t y, int16_t x, int16_t z, int16_t yaw);
  void turn_left(int16_t speed);
  void turn_right(int16_t speed);
  void unlock(void);
  void stop(void);
  void onekey_takeoff(uint8_t height);
  void land(void);
  void get_air_height(void);
  void set_height(uint8_t height);
  void on_height_callback(void);
  void on_data_received(Data &data);
  void run(void);

private:
  std::mutex uav_lock;
  std::mutex& lock;
  SerialHelper ser;

  std::atomic<bool> _isConn{false};
  std::atomic<bool> _active{true};
  PID_t pid;
  std::thread send_thread;
  std::thread height_thread;
};

#endif