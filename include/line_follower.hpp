#ifndef __LINE_FOLLOWER_H
#define __LINE_FOLLOWER_H
#include "connect_uav.hpp"
#include "pid_r.hpp"
#include "pid_t.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <zbar.h>

class LineFollower {
private:
  // ---------------- 原Python成员变量 ----------------
  std::string scan_content;    // 二维码内容
  bool is_code_center;         // 二维码是否居中
  bool is_follow;              // 是否继续巡线
  bool is_forward;             // 是否沿直线行驶
  bool is_offset;              // 是否有左右偏差
  bool is_decode;              // 是否启动二维码解码
  int offset_o;                // 偏移量缓存
  int is_begin;                // 启动标志
  bool get_red;                // 是否识别到红色圆点
  bool get_qr;                 // 是否识别到二维码
  int qr_count;                // 二维码识别计数
  PID_t pid_t;                 // 偏航PID控制器（对应原PID_t）
  PID_r pid_r;                 // 横移PID控制器（对应原PID_r）
  double velocity_sum;         // 速度积分和
  double last_filter_velocity; // 上一次滤波后的速度
  double V_kp, V_ki;           // 速度环PID参数
  double Kp_row, Kd_row;       // 横移PID参数
  double Kp_yaw, Kd_yaw;       // 偏航PID参数
  double timing;               // 单次循环耗时
  double speed_yaw_global;     // 全局偏航速度
  int offset_center_d;         // 中心偏移量
  int lost5;                   // 二维码丢失计数
  int cnt;                     // 有效轮廓计数
  int out;                     // 初始前进计数
  int index = 0;
  const int no_slice = 4;                       // 图像分割块数
  const cv::Point center = cv::Point(240, 227); // 图像中心
  cv::VideoCapture cap;                         // 摄像头捕获对象
  UPUavControl airplanceApi;                    // 无人机控制实例

  /**
   * 计算轮廓质心
   * @param image 输入图像
   * @param kernel_size 卷积核大小
   * @param threshold 二值化阈值
   * @return 质心坐标（无效返回(-1,-1)）
   */
  cv::Point calculate_centroid(const cv::Mat &image, int kernel_size = 5,
                               int threshold = 127);

  /**
   * 基于RGB差值检测红色区域
   * @param image 输入BGR图像
   * @param threshold R与G/B的最小差值
   * @param min_red R通道最小值
   * @return 红色区域二值掩膜
   */
  cv::Mat detect_red_by_rgb_diff(const cv::Mat &image, int threshold = 40,
                                 int min_red = 80);

  /**
   * 获取轮廓中心
   * @param contour 输入轮廓
   * @return 轮廓中心坐标
   */
  cv::Point get_contour_center(const std::vector<cv::Point> &contour);

  /**
   * 处理单块图像，返回处理后图像和轮廓中心
   * @param image 输入单块图像
   * @return 处理后图像 + 轮廓中心
   */
  std::pair<cv::Mat, cv::Point> process(const cv::Mat &image);

  /**
   * 将图像分割为num块
   * @param im 输入图像
   * @param num 分割块数
   * @return 分割后的图像列表 + 各块轮廓中心
   */
  std::pair<std::vector<cv::Mat>, std::vector<cv::Point>>
  slice_out(const cv::Mat &im, int num);

  /**
   * 移除背景（黑线检测预处理）
   * @param image 输入灰度图像
   * @param b 是否启用背景移除
   * @return 预处理后的图像
   */
  cv::Mat remove_background(const cv::Mat &image, bool b);

  /**
   * 速度滤波函数
   * @param speed_exp 当前速度
   * @param speed_exp_d 上一次速度
   * @return 滤波后的速度
   */
  double filter(double speed_exp, double speed_exp_d);

  /**
   * 拼接分割后的图像
   * @param images 分割后的图像列表
   * @return 拼接后的完整图像
   */
  cv::Mat repack(const std::vector<cv::Mat> &images);

  /**
   * 符号函数（返回1/-1/0）
   * @param a 输入数值
   * @return 符号值
   */
  int fuhao(int a);

  /**
   * 二维码识别与无人机对准控制
   * @param image 输入彩色图像
   */
  void decode(const cv::Mat &image);

  /**
   * 黑线巡线与PID控制逻辑
   * @param image 处理后的灰度图像
   * @param center 图像中心
   * @param cont_cent 各块轮廓中心列表
   */
  void line(cv::Mat &image, const cv::Point &center,
            const std::vector<cv::Point> &cont_cent);

  /**
   * 初始化无人机API（对应原api_init）
   */
  void api_init();

public:
  // ---------------- 公有成员函数 ----------------
  /**
   * 构造函数：初始化成员变量、无人机、摄像头
   */
  LineFollower(std::mutex &lock, int index = 0,
               std::string Port = "/dev/ttyUSB0", uint32_t BaudRate = 115200,
               uint8_t ByteSize = 8, char Parity = 'N', uint8_t Stopbits = 1);

  /**
   * 析构函数：释放资源
   */
  ~LineFollower();

  /**
   * 启动摄像头识别主循环（对应原start_video）
   */
  void start_video();
  void run();
};

#endif // LINEFOLLOWER_H