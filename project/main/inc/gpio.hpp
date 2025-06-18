#ifndef GPIO_H
#define GPIO_H

#include"base.hpp"

class GPIO
{
public:
    GPIO(int gpioNum_);
    ~GPIO(void);

    int getFileDescriptor(void) const;               // 获取 GPIO 文件描述符
    bool setDirection(const std::string &direction); // 设置GPIO方向，out为输出，in为输入
    bool setValue(bool value);                       // 设置 GPIO 输出值
    bool readValue(void);                            // 读取 GPIO 输入值
    bool setEdge(const std::string &edge);

private:
    int gpioNum;
    int fd;
    std::string gpioPath;

    bool writeToFile(const std::string &path, const std::string &value);
};


#endif