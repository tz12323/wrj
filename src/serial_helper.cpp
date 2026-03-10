#include "serial_helper.hpp"
#include <chrono>
// #include <dirent.h>
#include <unistd.h>
// #include <glob.h>
// #include <fstream>
#include <regex>
// #include <filesystem>
// #include <stdexcept>
#include <iomanip>
#include <iostream>
#include <set>
#include <algorithm> // for std::find

SerialHelper::SerialHelper(std::mutex &lock, std::string Port,
                           uint32_t BaudRate, uint8_t ByteSize, char Parity,
                           uint8_t Stopbits)
    : lock(lock), port(Port) {
    switch (BaudRate) {
        case 9600:
            baudrate = LibSerial::BaudRate::BAUD_9600;
            break;
        case 19200:
            baudrate = LibSerial::BaudRate::BAUD_19200;
            break;
        case 38400:
            baudrate = LibSerial::BaudRate::BAUD_38400;
            break;
        case 57600:
            baudrate = LibSerial::BaudRate::BAUD_57600;
            break;
        case 115200:
            baudrate = LibSerial::BaudRate::BAUD_115200;
            break;
        default:
            baudrate = LibSerial::BaudRate::BAUD_115200;
            break;
    }
    switch (ByteSize) {
        case 5:
            bytesize = LibSerial::CharacterSize::CHAR_SIZE_5;
            break;
        case 6:
            bytesize = LibSerial::CharacterSize::CHAR_SIZE_6;
            break;
        case 7:
            bytesize = LibSerial::CharacterSize::CHAR_SIZE_7;
            break;
        case 8:
            bytesize = LibSerial::CharacterSize::CHAR_SIZE_8;
            break;
        default:
            bytesize = LibSerial::CharacterSize::CHAR_SIZE_DEFAULT;
            break;
    }
    switch (Stopbits) {
        case 1:
            stopbits = LibSerial::StopBits::STOP_BITS_1;
            break;
        case 2:
            stopbits = LibSerial::StopBits::STOP_BITS_2;
            break;
        default:
            stopbits = LibSerial::StopBits::STOP_BITS_DEFAULT;
            break;
    }
    switch (Parity) {
        case 'N':
            parity = LibSerial::Parity::PARITY_NONE;
            break;
        case 'O':
            parity = LibSerial::Parity::PARITY_ODD;
            break;
        case 'E':
            parity = LibSerial::Parity::PARITY_EVEN;
            break;
        default:
            parity = LibSerial::Parity::PARITY_DEFAULT;
            break;
    }
}

SerialHelper::~SerialHelper() {
    _running = false;
    disconnect();
    if (_connect_thread.joinable()) {
        _connect_thread.join();
    }
    if (_recv_thread.joinable()) {
        _recv_thread.join();
    }
}

void SerialHelper::connect() {
    std::lock_guard<std::mutex> guard(lock);
    try {
        _serial.Open(port);
        if (!_serial.IsOpen()) {
            // throw 前可以考虑不用 printf，直接在上层 catch 打印
            throw std::runtime_error("串口打开失败：" + port);
        }
        _serial.SetBaudRate(baudrate);
        _serial.SetCharacterSize(bytesize);
        _serial.SetParity(parity);
        _serial.SetStopBits(stopbits);
    } catch (const std::exception &e) {
        std::cerr << "串口连接失败 (" << e.what() << ")" << std::endl;
        _is_connected = false;
        return;
    }
    _is_connected = true;
}

void SerialHelper::disconnect() {
    if (_is_connected) {
        std::lock_guard<std::mutex> gard(lock);
        _serial.Close();
        _is_connected = false; // 补充：关闭后置为 false
    }
}

std::vector<uint8_t> SerialHelper::hex_to_bytes(const std::string &hex) {
    Data bytes;
    std::istringstream iss(hex);
    std::string byteStr;
    while (iss >> std::setw(2) >> byteStr) {
        uint8_t byte = static_cast<uint8_t>(std::stoi(byteStr, nullptr, 16));
        bytes.push_back(byte);
    }
    return bytes;
}

void SerialHelper::write(const Data &data) {
    std::lock_guard<std::mutex> gard(lock);
    if (_is_connected) {
        // 串口已连接，直接写
        _serial.Write(data);
    }
}

void SerialHelper::on_connected_changed(const ConnectedCallback &func) {
    if (_connect_thread.joinable()) {
        _connect_thread.join();
    }
    // 【修复3】不要用引用捕获 `&func` 传给分离的线程，改用值传递捕获 `func`
    _connect_thread = std::thread([this, func]() { this->_on_connected_changed(func); });
    // 【修复2】线程名字长度必须严格小于16个字符 (15字符+1结束符)
    pthread_setname_np(_connect_thread.native_handle(), "ser_conn_thread");
}

void SerialHelper::_on_connected_changed(const ConnectedCallback &func) {
    _is_connected_temp = false;

    while (_running) {
        bool need_callback = false;
        bool current_status = false;

        try {
            { // 【修复4】缩小锁的作用域
                // 【修复1】直接检查设备文件是否存在，避免遍历所有串口导致 ioctl 异常
                bool port_exists = (access(port.c_str(), F_OK) == 0);

                std::lock_guard<std::mutex> gard(lock);
                _is_connected = port_exists;

                // 状态变化才回调
                if (_is_connected_temp != _is_connected) {
                    need_callback = true;
                    current_status = _is_connected;
                }

                _is_connected_temp = _is_connected.load();
            } // 锁在这里释放
        } catch (const std::exception &e) {
            std::cerr << "[SerialHelper] 检测设备时异常: " << e.what() << std::endl;
            // 发生异常时认为设备已拔出断开
            if (_is_connected_temp) {
                need_callback = true;
                current_status = false;
                _is_connected_temp = false;
            }
        } catch (...) {
            std::cerr << "[SerialHelper] 检测设备时发生未知异常" << std::endl;
        }

        // 【修复4】把 func 回调挪到锁的外部执行，防止由于回调里面重入串口写函数造成死锁
        if (need_callback) {
            func(current_status);
        }

        // 0.5 秒
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

void SerialHelper::on_data_received(const DataReceived &func) {
    if (_recv_thread.joinable()) {
        _recv_thread.join();
    }
    // 【修复3】不要用引用捕获 `&func` 传给分离的线程，改用值传递捕获 `func`
    _recv_thread = std::thread([this, func]() { this->_on_data_received(func); });
    // 【修复2】缩短线程名避免越界失败
    pthread_setname_np(_recv_thread.native_handle(), "ser_recv_thread");
}

// 串口数据接收线程
void SerialHelper::_on_data_received(const DataReceived &func) {
    while (_running) {
        bool has_data = false;
        Data received_data;

        try {
            { // 【修复4】缩小锁的作用域
                std::lock_guard<std::mutex> gard(lock);
                if (_is_connected) {
                    // 检查数据是否可读（非阻塞）
                    if (_serial.IsDataAvailable()) {
                        // 读取所有可用字节
                        while (_serial.IsDataAvailable()) {
                            char byte;
                            _serial.ReadByte(byte, 20); // ReadByte 会阻塞
                            received_data.push_back(static_cast<uint8_t>(byte));
                        }

                        if (!received_data.empty() && received_data.size() >= 9) {
                            uint32_t height =
                                (static_cast<uint32_t>(received_data[5]) & 0xFF) |
                                ((static_cast<uint32_t>(received_data[6]) & 0xFF) << 8) |
                                ((static_cast<uint32_t>(received_data[7]) & 0xFF) << 16) |
                                ((static_cast<uint32_t>(received_data[8]) & 0xFF) << 24);
                            current_height = height;
                            
                            has_data = true;
                        }
                    }
                }
            } // 锁在这里释放
        } catch (const LibSerial::ReadTimeout &timeout) {
            // 超时属于正常现象，静默处理即可
        } catch (const std::exception &e) {
            std::cerr << "[SerialHelper] 接收数据异常: " << e.what() << std::endl;
            std::lock_guard<std::mutex> gard(lock);
            _is_connected = false;
            _serial.Close();
            break;
        } catch (...) {
            std::cerr << "[SerialHelper] 接收数据时发生未知异常" << std::endl;
            break;
        }

        // 【修复4】在锁外执行回调，防止在 func 内部调用 write 导致双重加锁死锁
        if (has_data) {
            func(received_data);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}