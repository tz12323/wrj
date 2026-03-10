#include "connect_uav.hpp"
#include <algorithm>
#include <stdio.h>
#ifdef DEBUG
#define DATA_LOG(data)                \
    do                                \
    {                                 \
        printf("Data: ");             \
        for (const auto &byte : data) \
        {                             \
            printf("0x%02X ", byte);  \
        }                             \
        printf("\n");                 \
    } while (0)
#else
#define DATA_LOG(data)
#endif
UPUavControl::UPUavControl(std::mutex &lock, std::string Port,
                           uint32_t BaudRate, uint8_t ByteSize, char Parity,
                           uint8_t Stopbits)
    : lock(lock), ser(lock, Port, BaudRate, ByteSize, Parity, Stopbits) {}

UPUavControl::~UPUavControl() noexcept
{
    isFly = false;   // 停止高度线程
    _isConn = false; // 停止发送线程
}

void UPUavControl::run()
{
    ser.on_connected_changed([this](bool connected)
                             { this->myserial_on_connected_changed(connected); });
    send_thread = std::thread([this]()
                              { this->send_msg(); });
    pthread_setname_np(send_thread.native_handle(), "send_thread");
    send_thread.detach();
}

void UPUavControl::myserial_on_connected_changed(bool is_connected)
{
    if (is_connected)
    {
        printf("Connected\n");

        _isConn = true;
        ser.connect();
        ser.on_data_received([this](Data data)
                             { this->on_data_received(data); });
    }
    else
    {
        printf("DisConnected\n");
        _isConn = false;
        ser.disconnect();
    }
}
/* void UPUavControl::write(Data data)
{
    ser.write(data, true);
} */
void UPUavControl::send_msg()
{
    for (;;)
    {
        Data msg;
        bool has_message = false;

        // 只把 msg_list 的读/写操作限制在锁作用域内
        {
            std::lock_guard<std::mutex> gard(uav_lock);
            if (msg_list.size() > 0 && _isConn)
            {
                msg = msg_list.front();
                msg_list.pop();
                has_message = true;
            }
        }

        // 串口写操作在无锁时执行，避免死锁
        if (has_message)
        {
            ser.write(msg /* , false */);
            DATA_LOG(msg);
        }

#ifdef DEBUG
        else
        {
            std::lock_guard<std::mutex> gard(uav_lock);
            if (msg_list.size() > 0 && !_isConn)
            {
                auto msg_copy = msg_list;
                printf("Serial is not connected, data will not be sent!\n");
                printf("Data in msg_list:\n");
                while (!msg_copy.empty())
                {
                    DATA_LOG(msg_copy.front());
                    msg_copy.pop();
                }
                std::this_thread::sleep_for(std::chrono::microseconds(500));
            }
        }
#endif
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

std::tuple<std::vector<uint8_t>, uint16_t>
UPUavControl::generateCmd(uint8_t device, uint8_t cmd, uint8_t len, Data data)
{
    Data buffer(len + 6, 0);
    buffer[0] = 0xF5;
    buffer[1] = 0x5F;
    buffer[2] = device & 0xFF;
    buffer[3] = cmd & 0xFF;
    buffer[4] = len & 0xFF;
    std::copy_n(data.begin(),      // 源：data的起始位置
                len,               // 拷贝len个元素
                buffer.begin() + 5 // 目标：buffer的5号索引位置
    );
    uint32_t check = 0;
    for (int i = 0; i < len + 3; i++)
    {
        check += buffer[i + 2];
    }
    buffer[len + 5] = (~check) & 0xFF;
    return std::tuple<Data, uint8_t>(buffer, len + 6);
}
void UPUavControl::setMoveAction(int16_t y, int16_t x, int16_t z, int16_t yaw)
{
    Data data(8, 0);
    data[0] = y & 0xFF;
    data[1] = (y >> 8) & 0xFF;

    data[2] = x & 0xFF;
    data[3] = (x >> 8) & 0xFF;

    data[4] = z & 0xFF;
    data[5] = (z >> 8) & 0xFF;

    data[6] = yaw & 0xFF;
    data[7] = (yaw >> 8) & 0xFF;

    std::tuple<Data, uint16_t> tu = generateCmd(0x55, 0x01, 0x08, data);
    Data buffer = std::get<0>(tu);
    // uint16_t len = std::get<1>(tu);
    std::lock_guard<std::mutex> gard(uav_lock);
    msg_list.push(buffer);
}

int UPUavControl::get_current_height()
{
    std::lock_guard<std::mutex> gard(uav_lock);
    return current_height.load();
}
void UPUavControl::setServoPosition(uint16_t angel)
{
    Data data(2, 0);
    data[0] = angel & 0xFF;
    data[1] = (angel >> 8) & 0xFF;
    std::tuple<Data, uint16_t> tu = generateCmd(0x55, 0x03, 0x02, data);
    std::lock_guard<std::mutex> gard(uav_lock);
    msg_list.push(std::get<0>(tu));
}
void UPUavControl::move_forward(int16_t speed)
{
    setMoveAction(0, speed, 0, 0);
}
void UPUavControl::move_backward(int16_t speed)
{
    setMoveAction(0, -speed, 0, 0);
}
void UPUavControl::move_left(int16_t speed) { setMoveAction(-speed, 0, 0, 0); }
void UPUavControl::move_right(int16_t speed) { setMoveAction(speed, 0, 0, 0); }
void UPUavControl::move_up(int16_t speed) { setMoveAction(0, 0, speed, 0); }
void UPUavControl::move_down(int16_t speed) { setMoveAction(0, 0, speed, 0); }
void UPUavControl::move(int16_t y, int16_t x, int16_t z, int16_t yaw)
{
    setMoveAction(y, x, z, yaw);
}
void UPUavControl::turn_left(int16_t speed) { setMoveAction(0, 0, 0, -speed); }
void UPUavControl::turn_right(int16_t speed) { setMoveAction(0, 0, 0, speed); }
void UPUavControl::unlock() { setMoveAction(-500, -500, -500, 500); }
void UPUavControl::stop()
{
    setMoveAction(0, 0, 0, 0);
    setMoveAction(0, 0, -500, 0);
}
void UPUavControl::onekey_takeoff(uint8_t height)
{
    Data data(1, 0);
    data[0] = height & 0xFF;
    std::tuple<Data, uint16_t> tu = generateCmd(0x55, 0x05, 0x01, data);
    // 集成set_height函数
    {
        std::lock_guard<std::mutex> gard(uav_lock);
        msg_list.push(std::get<0>(tu));
        settingHeight = height; // 设置目标高度
        isFly = true;           // 允许发送高度查询指令
    }
}
void UPUavControl::land()
{
    isFly = false;
    Data buffer(6, 0);
    buffer[0] = 0xF5;
    buffer[1] = 0x5F;
    buffer[2] = 0x55;
    buffer[3] = 0x06;
    buffer[4] = 0;
    buffer[5] = 0xA4;
    std::lock_guard<std::mutex> gard(uav_lock);
    msg_list.push(buffer);
    msg_list.push(buffer);
}

void UPUavControl::get_air_height()
{
    height_thread = std::thread([this]()
                                { this->on_height_callback(); });
    pthread_setname_np(height_thread.native_handle(), "get_height");
    height_thread.detach();
}
void UPUavControl::set_height(uint8_t height)
{
    std::lock_guard<std::mutex> gard(uav_lock);
    settingHeight = height;
    isFly = true;
}
void UPUavControl::on_height_callback()
{
    Data buffer(6, 0);
    for (;;)
    {
        if (_isConn && isFly)
        {
            buffer[0] = 0xF5;
            buffer[1] = 0x5F;
            buffer[2] = 0x55;
            buffer[3] = 0x02;
            buffer[4] = 0;
            buffer[5] = 0xA8;
            {
                std::lock_guard<std::mutex> gard(uav_lock);
                msg_list.push(buffer);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Maintain low CPU usage when not flying
        }
    }
}
void UPUavControl::on_data_received(Data &data)
{
    // std::lock_guard<std::mutex> gard(lock);
    if (data[2] == 0x55 && data[3] == 0x06)
    {
        printf("%d\n", data[5]);
    }
    else if (data[2] == 0x55 && data[3] == 0x02 && settingHeight != 0)
    {
        uint32_t height = ((data[5] & 0xFF) | ((data[6] & 0xFF) << 8) |
                           ((data[7] & 0xFF) << 16) | ((data[8] & 0xFF) << 24));
        current_height = height;
        if (height <= 30)
        {
            height = 0;
        }
        int16_t speed = pid.pid_calculate(0.001, 0, settingHeight, height);
        if (0 < speed && speed < dead_area)
        {
            speed = speed + (min_up - speed);
        }
        if (-dead_area < speed && speed < 0)
        {
            speed = speed - (min_down - speed);
        }
        if (std::abs(int64_t(height) - int16_t(settingHeight)) < 3)
        {
            move_up(0);
            move_down(0);
        }
        else
        {
            if (int64_t(height) - int16_t(settingHeight) < 0)
            {
                move_up(speed);
            }
            else if (int64_t(height) - int16_t(settingHeight) > 0)
            {
                move_down(speed);
            }
            printf("height: %d speed: %d\n", height, speed);
        }
    }
}