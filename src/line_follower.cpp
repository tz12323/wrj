#include "line_follower.hpp"
#include "connect_uav.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

extern std::atomic<bool> g_running;

template <typename T>
T clamp(const T &value, const T &min_val, const T &max_val)
{
    if (value < min_val)
        return min_val;
    if (value > max_val)
        return max_val;
    return value;
}

// ---------------- 构造函数实现 ----------------
LineFollower::LineFollower(std::mutex &lock, int index, std::string Port,
                           uint32_t BaudRate, uint8_t ByteSize, char Parity,
                           uint8_t Stopbits)
    : scan_content(""), is_code_center(false), is_follow(true),
      is_forward(false), is_offset(false), is_decode(false), offset_o(0),
      is_begin(1), get_red(false), get_qr(false), qr_count(0),
      velocity_sum(0.0), last_filter_velocity(0.0), V_kp(0.0), V_ki(0.0),
      Kp_row(0.0), Kd_row(0.0), Kp_yaw(0.0), Kd_yaw(0.0), timing(0.0),
      speed_yaw_global(0.0), offset_center_d(0), lost5(0), cnt(0), out(0),
      index(index),
      airplanceApi(lock, Port, BaudRate, ByteSize, Parity, Stopbits) {}

// ---------------- 析构函数实现 ----------------
LineFollower::~LineFollower()
{
    // 释放资源
    if (cap.isOpened())
        cap.release();
    cv::destroyAllWindows();
}

// ---------------- 私有成员函数实现 ----------------
cv::Point LineFollower::calculate_centroid(const cv::Mat &image,
                                           int kernel_size, int threshold)
{
    cv::Mat gray, blurred, binary, closed;
    if (image.channels() > 1)
    {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        gray = image.clone();
    }

    // 高斯模糊去噪
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
    // 二值化
    cv::threshold(blurred, binary, threshold, 255, cv::THRESH_BINARY);
    // 闭运算（膨胀+腐蚀）
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    cv::morphologyEx(binary, closed, cv::MORPH_CLOSE, kernel);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(closed, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty())
        return cv::Point(-1, -1);

    // 找最大轮廓
    auto largest_contour = *std::max_element(
        contours.begin(), contours.end(),
        [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
        {
            return cv::contourArea(a) < cv::contourArea(b);
        });

    // 计算质心
    cv::Moments M = cv::moments(largest_contour);
    if (M.m00 != 0)
    {
        int cX = static_cast<int>(M.m10 / M.m00);
        int cY = static_cast<int>(M.m01 / M.m00);
        return cv::Point(cX, cY);
    }
    else
    {
        return cv::Point(-1, -1);
    }
}

void LineFollower::run()
{
    airplanceApi.run();
    std::thread api_thread([this]()
                           { this->api_init(); });
    api_thread.detach(); // 分离线程，后台运行

    // 等待初始化完成
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 启动高度监听线程
    airplanceApi.get_air_height();

    // 无人机起飞到60cm高度并悬停
    airplanceApi.onekey_takeoff(60);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    cap.open(index);
    if (!cap.isOpened())
    {
        std::cerr << "[ERROR] 摄像头打开失败！" << std::endl;
        exit(1);
    }
    start_video();
    /* for (;;)
    {
        std::this_thread::sleep_for(std::chrono::seconds(10));
    } */
}

cv::Mat LineFollower::detect_red_by_rgb_diff(const cv::Mat &image,
                                             int threshold, int min_red)
{
    cv::Mat img_float;
    image.convertTo(img_float, CV_32F);

    // 分离BGR通道（OpenCV默认BGR顺序）
    std::vector<cv::Mat> channels;
    cv::split(img_float, channels);
    cv::Mat b = channels[0], g = channels[1], r = channels[2];

    // 计算R与G/B的差值
    cv::Mat diff_rg = r - g;
    cv::Mat diff_rb = r - b;

    // 生成红色区域掩膜
    cv::Mat mask =
        ((diff_rg > threshold) & (diff_rb > threshold) & (r > min_red));
    mask.convertTo(mask, CV_8U);
    mask *= 255;

    // 形态学开运算去噪
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

    return mask;
}

cv::Point
LineFollower::get_contour_center(const std::vector<cv::Point> &contour)
{
    cv::Moments m = cv::moments(contour);
    if (m.m00 == 0)
        return cv::Point(0, 0);
    int x = static_cast<int>(m.m10 / m.m00);
    int y = static_cast<int>(m.m01 / m.m00);
    return cv::Point(x, y);
}

std::pair<cv::Mat, cv::Point> LineFollower::process(const cv::Mat &image)
{
    cv::Mat result = image.clone();
    cv::Point contour_center(0, 0);

    // 增加 try-catch 定位 process 内部错误
    try
    {
        cv::Mat thresh;
        // 二值化（反相）
        cv::threshold(image, thresh, 100, 255, cv::THRESH_BINARY_INV);

        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(thresh, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty())
        {
            // 找最大轮廓
            auto main_contour = *std::max_element(
                contours.begin(), contours.end(),
                [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            // 绘制轮廓和中心
            std::vector<std::vector<cv::Point>> contours_to_draw;
            contours_to_draw.push_back(main_contour);
            // 使用 result (image.clone) 作为绘制目标
            cv::drawContours(result, contours_to_draw, -1, cv::Scalar(150, 150, 150), 2);
            contour_center = get_contour_center(main_contour);
            cv::circle(result, contour_center, 2, cv::Scalar(150, 150, 150), 2);
        }
    }
    catch (const cv::Exception &e)
    {
        std::cerr << "OpenCV Exception in process(): " << e.what() << std::endl;
    }
    return {result, contour_center};
}

std::pair<std::vector<cv::Mat>, std::vector<cv::Point>>
LineFollower::slice_out(const cv::Mat &im, int num)
{
    std::vector<cv::Mat> sliced_imgs;
    std::vector<cv::Point> cont_cent;

    int height = im.rows;
    int width = im.cols;
    int sl = height / num;

    for (int i = 0; i < num; ++i)
    {
        int part = sl * i;
        // 裁剪图像块
        cv::Mat crop_img = im(cv::Range(part, part + sl), cv::Range(0, width));
        auto [processed_img, cent] = process(crop_img);
        sliced_imgs.push_back(processed_img);
        cont_cent.push_back(cent);
    }

    return {sliced_imgs, cont_cent};
}

cv::Mat LineFollower::remove_background(const cv::Mat &image, bool b)
{
    if (!b)
        return image.clone();

    int up = 70, lo = 0;
    cv::Mat mask;
    // 颜色范围过滤
    cv::inRange(image, lo, up, mask);

    cv::Mat result;
    cv::bitwise_and(image, image, result, mask);
    cv::bitwise_not(result, result, mask);
    result = 255 - result;

    // 图像两侧置为白色（减少边缘干扰）
    int height = result.rows;
    int width = result.cols;
    int side_width = static_cast<int>(width * 0.2);
    result(cv::Range(0, height), cv::Range(0, side_width)).setTo(255);
    result(cv::Range(0, height), cv::Range(width - side_width, width)).setTo(255);

    return result;
}

double LineFollower::filter(double speed_exp, double speed_exp_d)
{
    double a = 0.3; // 滤波系数
    double filter_velocity = a * speed_exp + (1 - a) * speed_exp_d;
    velocity_sum += filter_velocity;
    // 积分限幅
    velocity_sum = clamp(velocity_sum, -500.0, 500.0);
    last_filter_velocity = filter_velocity;
    return V_kp * filter_velocity + V_ki * velocity_sum;
}

cv::Mat LineFollower::repack(const std::vector<cv::Mat> &images)
{
    if (images.empty())
        return cv::Mat();
    cv::Mat im = images[0];
    // 垂直拼接图像
    for (size_t i = 1; i < images.size(); ++i)
    {
        cv::vconcat(im, images[i], im);
    }
    return im;
}

int LineFollower::fuhao(int a)
{
    if (a == 0)
        return 1;
    return a / abs(a);
}

void LineFollower::decode(const cv::Mat &image)
{
    auto t0 = std::chrono::high_resolution_clock::now();

    // 初始化zbar二维码扫描器
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    // 转换为灰度图
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    int width = gray.cols;
    int height = gray.rows;

    // 封装zbar图像
    zbar::Image zbar_image(width, height, "Y800", gray.data, width * height);
    int n = scanner.scan(zbar_image);

    // 更新二维码检测状态
    if (n > 0)
    {
        get_qr = true;
        qr_count++;
    }
    else if (get_qr)
    {
        lost5++;
    }

    if (get_qr)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // 处理识别到的二维码
    for (auto symbol = zbar_image.symbol_begin();
         symbol != zbar_image.symbol_end(); ++symbol)
    {
        scan_content = symbol->get_data();
        std::cout << "[QR] 识别内容: " << scan_content << std::endl;

        // 修正：zbar::Symbol获取位置的正确接口是get_locations()（复数）
        int loc_count = symbol->get_location_size(); // 获取位置点数量
        float offset_x = 0, offset_y = 0;
        for (int i = 0; i < loc_count; ++i)
        {
            offset_x += symbol->get_location_x(i); // 获取第i个点的x坐标
            offset_y += symbol->get_location_y(i); // 获取第i个点的y坐标
        }
        offset_x /= loc_count;
        offset_y /= loc_count;

        std::cout << "[QR] 偏移量: x=" << offset_x << ", y=" << offset_y
                  << std::endl;
        is_forward = false;

        // 计算偏移百分比
        float x_percent = abs(offset_x - 240) / 40.0f;
        float y_percent = abs(offset_y - 227) / 40.0f;
        int l = 0;

        // 判断二维码是否居中
        if (x_percent < 0.5 && y_percent < 0.5)
        {
            is_code_center = true;
        }
        else if (x_percent >= y_percent)
        {
            // 左右偏移：控制无人机左右移动
            if (offset_x < 220)
            {
                l = std::max(80, static_cast<int>(0.9 * (240 - offset_x)));
                airplanceApi.move_left(l);
                std::cout << "[QR] 左移: " << l << std::endl;
                is_code_center = false;
            }
            else if (offset_x > 260)
            {
                l = std::max(80, static_cast<int>(0.9 * (offset_x - 240)));
                airplanceApi.move_right(l);
                std::cout << "[QR] 右移: " << l << std::endl;
                is_code_center = false;
            }
        }
        else
        {
            // 前后偏移：控制无人机前后移动
            if (offset_y < 207)
            {
                l = std::max(80, static_cast<int>(0.9 * (227 - offset_y)));
                airplanceApi.move_forward(l);
                std::cout << "[QR] 前移: " << l << std::endl;
                is_code_center = false;
            }
            else if (offset_y > 247)
            {
                l = std::max(80, static_cast<int>(0.9 * (offset_y - 227)));
                airplanceApi.move_backward(l);
                std::cout << "[QR] 后移: " << l << std::endl;
                is_code_center = false;
            }
        }
    }

    // 二维码丢失/识别次数超限：判定为居中
    if (lost5 >= 30 || qr_count >= 50)
    {
        is_code_center = true;
    }

    // 输出调试信息
    std::cout << "[QR] 识别计数: " << qr_count << ", 丢失计数: " << lost5
              << std::endl;
    auto t1 = std::chrono::high_resolution_clock::now();
    double qr_time = std::chrono::duration<double>(t1 - t0).count();
    std::cout << "[QR] 识别耗时: " << qr_time << "s" << std::endl;
}

void LineFollower::line(cv::Mat &image, const cv::Point &center, const std::vector<cv::Point> &cont_cent)
{
    // 绘制分割线（可视化）
    cv::line(image, cv::Point(0, 65), cv::Point(480, 65), cv::Scalar(30, 30, 30),
             1);
    cv::line(image, cv::Point(0, 130), cv::Point(480, 130),
             cv::Scalar(30, 30, 30), 1);
    cv::line(image, cv::Point(0, 195), cv::Point(480, 195),
             cv::Scalar(30, 30, 30), 1);
    cv::line(image, cv::Point(240, 0), cv::Point(240, 260),
             cv::Scalar(30, 30, 30), 1);

    // 计算各块轮廓中心的偏移角度
    int line_angle1 = 0, line_angle2 = 0, line_angle3 = 0;
    if (cont_cent[0].x != 0 && cont_cent[1].x != 0)
    {
        line_angle1 = static_cast<int>(180 *
                                       atan2(cont_cent[1].x - cont_cent[0].x,
                                             cont_cent[1].y + 65 - cont_cent[0].y) /
                                       M_PI);
    }
    if (cont_cent[1].x != 0 && cont_cent[2].x != 0)
    {
        line_angle2 =
            static_cast<int>(180 *
                             atan2(cont_cent[2].x - cont_cent[1].x,
                                   cont_cent[2].y + 130 - cont_cent[1].y - 65) /
                             M_PI);
    }
    if (cont_cent[2].x != 0 && cont_cent[3].x != 0)
    {
        line_angle3 =
            static_cast<int>(180 *
                             atan2(cont_cent[3].x - cont_cent[2].x,
                                   cont_cent[3].y + 195 - cont_cent[2].y - 130) /
                             M_PI);
    }

    // 计算各点的x轴的偏移量
    int offset_x1 = cont_cent[1].x - cont_cent[0].x;
    int offset_x2 = cont_cent[2].x - cont_cent[1].x;
    int offset_x3 = cont_cent[3].x - cont_cent[2].x;

    // 计算第三个点与中心点的偏移量
    int offset_center = cont_cent[2].x - 240;

    // 绘制中心连线（可视化）
    for (size_t i = 0; i < cont_cent.size(); ++i)
    {
        cv::line(image, center, cv::Point(cont_cent[i].x, cont_cent[i].y + 65 * i),
                 cv::Scalar(100, 100, 100), 1);
    }

    // ---------------- 巡线控制逻辑 ----------------
    if (is_follow)
    {
        if (offset_center > 70)
        {
            airplanceApi.move_right(150);
            std::cout << "move_right" << std::endl;
        }
        else if (offset_center < -70)
        {
            airplanceApi.move_left(150);
            std::cout << "move_left" << std::endl;
        }
        else
        {
            airplanceApi.move_forward(150);

            if (std::abs(line_angle1) > 15 || std::abs(line_angle2) > 15 ||
                std::abs(line_angle3) > 15)
            {
                if (cont_cent[0].x != 0 && cont_cent[1].x != 0)
                {
                    if (offset_x1 > 0)
                    {
                        std::cout << "turn_left" << std::endl;
                        airplanceApi.turn_left(120);
                    }
                    else
                    {
                        std::cout << "turn_right" << std::endl;
                        airplanceApi.turn_right(120);
                    }
                }
                else
                {
                    if (cont_cent[1].x != 0 && cont_cent[2].x != 0)
                    {
                        if (offset_x2 > 0)
                        {
                            std::cout << "turn_left" << std::endl;
                            airplanceApi.turn_left(120);
                        }
                        else
                        {
                            std::cout << "turn_right" << std::endl;
                            airplanceApi.turn_right(120);
                        }
                    }
                    else
                    {
                        if (cont_cent[2].x != 0 && cont_cent[3].x != 0)
                        {
                            if (offset_x3 > 0)
                            {
                                std::cout << "turn_left" << std::endl;
                                airplanceApi.turn_left(120);
                            }
                            else
                            {
                                std::cout << "turn_right" << std::endl;
                                airplanceApi.turn_right(120);
                            }
                        }
                    }
                }
            }
        }
    }
}

void LineFollower::api_init()
{
    std::cout << "[UAV] 无人机控制线程启动" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // 控制舵机旋转90度
    airplanceApi.setServoPosition(90);
}

// ---------------- 公有成员函数实现 ----------------
void LineFollower::start_video()
{
    while (!cap.open(index))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "[ERROR] 摄像头打开失败，重试中..." << std::endl;
    }

    for (;;)
    {
        try
        {
            if (!g_running)
                break;
            auto start_time = std::chrono::high_resolution_clock::now();
            cv::Mat frame;
            cap >> frame;

            if (frame.empty())
                break;

            // 缩放图像（0.75倍）
            cv::resize(frame, frame, cv::Size(), 0.75, 0.75);

            // ---------------- 红色圆点识别 ----------------
            get_red = false;
            cv::Mat red_img = detect_red_by_rgb_diff(frame);
            std::vector<std::vector<cv::Point>> cnts;
            cv::findContours(red_img, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            if (!cnts.empty())
            {
                // 找最大红色轮廓
                auto cnt = *std::max_element(
                    cnts.begin(), cnts.end(),
                    [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                    {
                        return cv::contourArea(a) < cv::contourArea(b);
                    });

                // 最小外接圆
                cv::Point2f center_red;
                float radius;
                cv::minEnclosingCircle(cnt, center_red, radius);

                // 过滤有效红色圆点
                if (radius > 20 && center_red.y > 0 && center_red.y < 250)
                {
                    std::cout << "[RED] 识别到红色圆点" << std::endl;
                    // 修复：在单通道掩膜上画图只能用单灰度值
                    cv::circle(red_img, center_red, static_cast<int>(radius),
                               cv::Scalar(255), -1);
                    get_red = true;
                    is_decode = true; // 启动二维码识别
                }
            }

            // ---------------- 二维码识别 ----------------
            cv::Mat gray_frame;
            cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
            if (is_decode && !is_code_center)
            {
                decode(frame);
            }

            // ---------------- 二维码降落判定 ----------------
            if (scan_content == "landed" && is_code_center)
            {
                is_follow = false;
                is_forward = false;
                airplanceApi.move(0, 0, 0, 0);
                airplanceApi.land();
                break; // 降落完成，退出循环
            }

            // ---------------- 黑线识别 ----------------
            cv::Mat img = remove_background(gray_frame, true);
            auto [slices, cont_cent] = slice_out(img, no_slice);
            img = repack(slices);
            if (!get_qr)
            {
                line(img, center, cont_cent);
            }

            // ---------------- 可视化显示 ----------------
            cv::imshow("frame", img);
            cv::imshow("scan", frame);

            // ---------------- 耗时统计 ----------------
            auto end_time = std::chrono::high_resolution_clock::now();
            timing = std::chrono::duration<double>(end_time - start_time).count();
            std::cout << "[TIME] 单次循环耗时: " << timing << "s" << std::endl;

            // 退出按键：q
            if (cv::waitKey(1) == 'q')
                break;
        }
        catch (const cv::Exception &e)
        {
            std::cerr << "OpenCV Exception in main loop: " << e.what() << std::endl;
        }
    }

    // 释放资源
    cap.release();
    cv::destroyAllWindows();
}