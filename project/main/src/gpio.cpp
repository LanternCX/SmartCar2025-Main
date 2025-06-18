#include"gpio.hpp"

// gpioNum为使用的GPIO编号
GPIO::GPIO(int gpioNum_) : gpioNum(gpioNum_), fd(-1)
{
    gpioPath = "/sys/class/gpio/gpio" + std::to_string(gpioNum);

    // 导出 GPIO
    if (!writeToFile("/sys/class/gpio/export", std::to_string(gpioNum)))
    {
        throw std::runtime_error("Failed to export GPIO " + std::to_string(gpioNum));
    }

    // 打开 value 文件，读写方式
    fd = open((gpioPath + "/value").c_str(), O_RDWR);
    if (fd == -1)
    {
        throw std::runtime_error("Failed to open GPIO value file: " + std::string(strerror(errno)));
    }
}

// 析构函数
GPIO::~GPIO(void)
{
    close(fd); // 关闭文件描述符
}

// 设置GPIO方向，out为输出，in为输入
bool GPIO::setDirection(const std::string &direction)
{
    return writeToFile(gpioPath + "/direction", direction);
}

bool GPIO::setEdge(const std::string &edge)
{
    return writeToFile(gpioPath + "/edge", edge);
}

// 设置 GPIO 输出值
bool GPIO::setValue(bool value)
{
    // 使用文件描述符写入 GPIO 值 ('1' 或 '0')
    const char *val_str = value ? "1" : "0";
    if (write(fd, val_str, 1) != 1)
    {
        std::cerr << "Failed to write GPIO value: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

// 读取 GPIO 输入值
bool GPIO::readValue(void)
{
    char value;
    lseek(fd, 0, SEEK_SET); // 重置文件偏移量
    if (read(fd, &value, 1) != 1)
    {
        std::cerr << "Failed to read GPIO value: " << strerror(errno) << std::endl;
        return false;
    }
    return value == '1'; // 如果读取的值为 '1'，则返回 true，否则返回 false
}

// 获取 GPIO 文件描述符
int GPIO::getFileDescriptor(void) const
{
    return fd;
}

bool GPIO::writeToFile(const std::string &path, const std::string &value)
{
    std::ofstream file(path);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file " << path << ": " << strerror(errno) << std::endl;
        return false;
    }
    file << value;
    return file.good();
}

