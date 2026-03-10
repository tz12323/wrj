#include "line_follower.hpp"
#include <jsoncpp/json/json.h>
#include <string.h>
#include <csignal>
#include <memory>
// GCC 8及以下版本可能需要 experimental
#if __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#error "Could not find <filesystem> or <experimental/filesystem>"
#endif

std::unique_ptr<LineFollower> LF;

// 全局标志，用于处理信号
std::atomic<bool> g_running{true};

void signal_handler(int signal)
{
  if (signal == SIGINT)
  {
    g_running = false;
    LF->~LineFollower();
    printf("检测到系统中断，退出成功");
    exit(0);
  }
}

typedef struct
{
  std::string Port;
  uint32_t BaudRate;
  uint8_t ByteSize;
  char Parity;
  uint8_t Stopbits;
} Serial;

std::tuple<Serial, int> json_file_parse(const std::string &filepath)
{
  Serial ser;
  int camera_index = 0;
  std::fstream ifs(filepath);
  if (!ifs.is_open())
  {
    std::cerr << "Error: could not open file " << filepath << std::endl;
    return std::make_tuple(ser, camera_index);
  }
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string errs;
  bool parse_ok = Json::parseFromStream(builder, ifs, &root, &errs);

  // 解析结果判断
  if (!parse_ok)
  {
    std::cerr << "Error: parse json failed -> "
              << errs << std::endl;
    ifs.close();
    return std::make_tuple(ser, camera_index);
  }
  ifs.close(); // 解析完成，关闭文件流

  if (root.isMember("port") && root["port"].isString())
  {
    ser.Port = root["port"].asString();
  }
  if (root.isMember("baudrate") && root["baudrate"].isInt())
  {
    ser.BaudRate = static_cast<uint32_t>(root["baudrate"].asInt());
  }
  if (root.isMember("bytesize") && root["bytesize"].isInt())
  {
    ser.ByteSize = static_cast<uint8_t>(root["bytesize"].asInt());
  }
  if (root.isMember("parity") && root["parity"].isString())
  {
    ser.Parity = root["parity"].asString()[0];
  }
  if (root.isMember("stopbits") && root["stopbits"].isInt())
  {
    ser.Stopbits = static_cast<uint8_t>(root["stopbits"].asInt());
  }
  if (root.isMember("camera_index") && root["camera_index"].isInt())
  {
    camera_index = root["camera_index"].asInt();
  }
  printf("Configuration file '%s' parsed successfully.\n", filepath.c_str());
  return std::make_tuple(ser, camera_index);
}

std::tuple<Serial, int> parameter_parse(int argc, char **argv)
{
  Serial args{"/dev/ttyUSB0", 115200, 8, 'N', 1};
  int index = 0;
  for (int i = 1; i < argc; i++)
  {
    if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0)
    {
      std::cout
          << "Usage: ./uav_controller [--port PORT] [--baudrate BAUDRATE] "
             "[--bytesize BYTESIZE] [--parity PARITY] [--stopbits STOPBITS]"
          << std::endl;
      std::cout << "Default: --port /dev/ttyUSB0 --baudrate 115200 --bytesize "
                   "8 --parity N --stopbits 1"
                << std::endl;
      exit(0);
    }
    if (strcmp(argv[i], "--port") == 0)
    {
      args.Port = std::string(argv[i + 1]);
      i++;
    }
    else if (strcmp(argv[i], "--baudrate") == 0)
    {
      args.BaudRate = std::stoi(argv[i + 1]);
      i++;
    }
    else if (strcmp(argv[i], "--bytesize") == 0)
    {
      args.ByteSize = static_cast<uint8_t>(std::stoi(argv[i + 1]));
      i++;
    }
    else if (strcmp(argv[i], "--parity") == 0)
    {
      args.Parity = argv[i + 1][0];
      i++;
    }
    else if (strcmp(argv[i], "--stopbits") == 0)
    {
      args.Stopbits = static_cast<uint8_t>(std::stoi(argv[i + 1]));
      i++;
    }
    else if (strcmp(argv[i], "--index") == 0)
    {
      index = std::stoi(argv[i + 1]);
      i++;
    }
    else if (strcmp(argv[i], "--path") == 0)
    {
      auto [args, index] = json_file_parse(std::string(argv[i + 1]));
      i++;
    }
  }
  auto args_tuple = std::make_tuple(args, index);
  return args_tuple;
}

int main(int argc, char **argv)
{
  std::signal(SIGINT, signal_handler);
  std::mutex lock;

  if (argc == 1)
  {
    if (!fs::exists("config.json"))
    {
      LF = std::make_unique<LineFollower>(lock);
      LF->run();
    }
    else
    {
      auto [ser, camera_index] = json_file_parse("config.json");
      printf("Serial Port: %s\n", ser.Port.c_str());
      printf("Baud Rate: %u\n", ser.BaudRate);
      printf("Byte Size: %u\n", ser.ByteSize);
      printf("Parity: %c\n", ser.Parity);
      printf("Stop Bits: %u\n", ser.Stopbits);
      printf("Camera Index: %d\n", camera_index);
      LF = std::make_unique<LineFollower>(lock, camera_index, ser.Port, ser.BaudRate, ser.ByteSize,
                                          ser.Parity, ser.Stopbits);
      LF->run();
    }
  }
  else
  {
    auto [args, index] = parameter_parse(argc, argv);
    printf("Serial Port: %s\n", args.Port.c_str());
    printf("Baud Rate: %u\n", args.BaudRate);
    printf("Byte Size: %u\n", args.ByteSize);
    printf("Parity: %c\n", args.Parity);
    printf("Stop Bits: %u\n", args.Stopbits);
    printf("Camera Index: %d\n", index);
    LF = std::make_unique<LineFollower>(lock, index, args.Port, args.BaudRate, args.ByteSize,
                                        args.Parity, args.Stopbits);
    LF->run();
  }
}
