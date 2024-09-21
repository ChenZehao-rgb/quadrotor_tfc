
#include <stdio.h>                     // 标准输入输出库
#include <termios.h>                   // 终端I/O接口
#include <unistd.h>                    // POSIX 操作系统 API 接口
#include <stdbool.h>                   // 定义 bool 类型
#include <errno.h>                     // 错误码定义
#include <drivers/drv_hrt.h>           // PX4 高精度定时器相关的驱动程序头文件
#include <string.h>                    // 字符串操作函数
#include <systemlib/err.h>             // 系统错误处理库（可能已经被替代，不推荐使用）
#include <nuttx/config.h>              // NuttX 配置
#include <fcntl.h>                     // 文件控制定义
#include <sys/types.h>                 // 定义数据类型，如 `size_t`
#include <sys/stat.h>                  // 文件状态定义
#include <poll.h>                      // 多路复用输入输出
#include <px4_platform_common/tasks.h> // PX4 平台任务控制相关头文件


__EXPORT int rw_uart_main(int argc, char *argv[]); // 导出rw_uart_main 函数，使得它在PX4模块系统中可调用
                                                   // __EXPORT 是一个宏，通常用来指定要导出的函数或变量，使其在其他模块中可见
static int uart_init(char * uart_name); // 串口初始化函数声明
static int set_uart_baudrate(const int fd, unsigned int baud); // 设置串口波特率函数声明

int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed; // 存储波特率

    switch (baud)
    {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;  // 终端I/O控制配置结构体

    int termios_state; // 存储 termios 函数的返回状态

    /* fill the struct for the new configuration */
    tcgetattr(fd, &uart_config); // 获取当前终端的配置参数，存储到 uart_config 中
    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR; // 清除 ONLCR 标志，以避免 LF 转换成 CR-LF
    /* no parity, one stop bit */
    uart_config.c_cflag &= ~(CSTOPB | PARENB); // 设置无奇偶校验，1个停止位
    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) // 设置输入波特率
    {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) // 设置输出波特率
    {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) // 应用新的终端配置
    {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}


int uart_init(char * uart_name) // 串口初始化函数
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY); // 打开串口设备

    if (serial_fd < 0) // 如果打开失败
    {
        err(1, "failed to open port: %s", uart_name); // 打印错误信息
        return false;
    }
    return serial_fd; // 返回文件描述符
}

int rw_uart_main(int argc, char *argv[])
{
    char data = '0'; // 临时存储单字节数据
    char buffer[4] = ""; // 存储从串口读取的4字节数据
    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */
    int uart_read = uart_init("/dev/ttyS2"); // 初始化串口2
    if(false == uart_read)return -1; // 如果初始化失败，返回错误
    if(false == set_uart_baudrate(uart_read,115200)) // 设置串口波特率为115200
    {
        printf("set_uart_baudrate is failed\n"); // 如果设置失败，打印错误信息
        return -1;
    }
    printf("uart init is successful\n"); // 初始化成功的提示信息

    while(true) // 无线循环
    {
        read(uart_read,&data,1); // 从串口读一个字节
        if(data == 'R') // 如果读取的字节是 R
        {
            for(int i = 0;i <4;++i) // 读取接下来的4个字节
            {
                read(uart_read,&data,1);
                buffer[i] = data;
                data = '0'; // 重置 data 为 0
            }
            printf("%s\n",buffer); // 打印读取到的4个字节数据
        }
    }

    return 0;
}
