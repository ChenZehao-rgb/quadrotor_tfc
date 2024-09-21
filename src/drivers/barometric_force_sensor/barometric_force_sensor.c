
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
// #include "barometric_force_sensor_topic.h"
#include <uORB/topics/barometric_force_sensor.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/time.h>
#include <stdint.h>
#include <uORB/uORB.h>


/* 定义主题 */
// ORB_DEFINE(barometric_force_sensor,
//             struct barometric_force_sensor_data_s,
//             sizeof(struct barometric_force_sensor_data_s),
//             "char[4] datastr; int data",
//             static_cast<uint8_t>(ORB_ID::barometric_force_sensor)); // 将主题注册到uORB系统中，以便其他模块可以订阅和发布


static bool thread_should_exit = false; // 标志变量，用于指示守护线程是否应该退出
static bool thread_running = false; // 标志变量，指示守护线程是否正在运行
static int daemon_task; // 保存守护线程的任务句柄，用于管理线程的创建和销毁

__EXPORT int barometric_force_sensor_main(int argc, char *argv[]); // 导出barometric_force_sensor_main 函数，使得它在PX4模块系统中可调用
                                                   // __EXPORT 是一个宏，通常用来指定要导出的函数或变量，使其在其他模块中可见
int barometric_force_sensor_thread_main(int argc, char *argv[]); // 声明守护线程的函数，该函数用于执行串口读取和数据发布的实际任务

static int uart_init(char * uart_name); // 串口初始化函数声明
static int set_uart_baudrate(const int fd, unsigned int baud); // 设置串口波特率函数声明
static void usage(const char *reason); // 声明一个静态函数，用于打印使用帮助信息

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
                                                        // 以读写模式（O_RDWR）和非控制终端模式（O_NOCTTY）打开串口设备，返回文件描述符serial_fd

    if (serial_fd < 0) // 如果打开失败
    {
        err(1, "failed to open port: %s", uart_name); // 打印错误信息
        return false;
    }
    return serial_fd; // 成功返回串口设备的文件描述符
}

static void usage(const char *reason) // 定义一个静态函数，用于打印命令行用法帮助信息
{
    if (reason) // 如果reason不为空，打印错误信息
    {
        fprintf(stderr, "%s\n", reason);
    }
    fprintf(stderr, "usage: barometric_force_sensor {start|stop|status} [param]\n\n"); // 打印正确的使用方式
    exit(1); // 以状态码1退出程序，表示异常终止
}

int barometric_force_sensor_main(int argc, char *argv[]) // 处理命令行参数并启动、停止或查看守护进程状态
{
    if (argc < 2) // 如果命令行参数少于两个，调佣sage函数并打印错误信息
    {
        usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) // 检查命令行参数是否为“start”，如果是则执行守护进程启动逻辑
    {
        if (thread_running) // 如果线程已经运行，打印 already running 信息并退出
        {
            warnx("already running\n");
            return 0;
        }
        thread_should_exit = false; // 定义一个守护进程
        daemon_task = px4_task_spawn_cmd("barometric_force_sensor", // 调用px4_task_spawn_cmd函数创建守护进程任务，函数参数
                                                // 包括任务名称、调度策略、优先级、堆栈大小、入口函数以及参数列表
                    SCHED_DEFAULT,
                    SCHED_PRIORITY_DEFAULT, // SCHED_PRIORITY_MAX - 5, 调度优先级
                    2000, // 堆栈分配大小
                    barometric_force_sensor_thread_main,
                    (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) // 检查命令行参数是否为“stop”，如果是则执行停止守护进程的逻辑
    {
        thread_should_exit = true; // 通知守护线程退出
        return 0;
    }

    if (!strcmp(argv[1], "status")) // 检查命令行参数是否为“status”，如果是则执行查询守护进程状态的逻辑
    {
        if (thread_running) // 如果线程正在运行，打印running
        {
            warnx("running");
        }
        else // 否则，打印stopped
        {
            warnx("stopped");
        }
        return 0;
    }

    usage("unrecognized command"); // 如果命令行参数不是start、stopped或status，调用usage打印unrecognized command错误信息
    return 1;
}

int barometric_force_sensor_thread_main(int argc, char *argv[])
{
    if (argc < 2)
    {
        errx(1, "need a serial port name as argument, eg: barometric_force_sensor start /dev/ttyS2");
    }

    const char *uart_name = argv[1]; // 获取串口设备名称

    warnx("opening port %s", uart_name); // 打印要打开的串口设备名称

    char data = '0'; // 临时存储单字节数据
    char buffer[16] = ""; // 存储从串口读取的4字节数据
    // int force_1 = 0;
    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */
    int uart_read = uart_init((char*)uart_name); // 初始化串口2
    if(false == uart_read)return -1; // 如果初始化失败，返回错误
    if(false == set_uart_baudrate(uart_read,115200)) // 设置串口波特率为115200
    {
        printf("set_uart_baudrate is failed\n"); // 如果设置失败，打印错误信息
        return -1;
    }
    printf("uart init is successful\n"); // 初始化成功的提示信息

    thread_running = true; // 表示线程正在运行

    /* 初始化数据结构体 */
    struct barometric_force_sensor_s sensordata; // 自定义消息结构体
    memset(&sensordata, 0, sizeof(sensordata)); // 结构体清零
    /* 公告主题 */
    orb_advert_t barometric_force_sensor_pub = orb_advertise(ORB_ID(barometric_force_sensor), &sensordata);

    while(!thread_should_exit) // 无线循环
    {
        read(uart_read,&data,1); // 从串口读一个字节
        if(data == 'R') // 如果读取的字节是 R
        {
            for(int i = 0;i <16;++i) // 读取接下来的4个字节
            {
                read(uart_read,&data,1);
                buffer[i] = data;
                data = '0'; // 重置 data 为 0
            }
            sensordata.data1 = (buffer[0]-'0') * 1000 + (buffer[1]-'0') * 100 + (buffer[2]-'0') * 10 + (buffer[3]-'0');
            sensordata.data2 = (buffer[4]-'0') * 1000 + (buffer[5]-'0') * 100 + (buffer[6]-'0') * 10 + (buffer[7]-'0');
            sensordata.data3 = (buffer[8]-'0') * 1000 + (buffer[9]-'0') * 100 + (buffer[10]-'0') * 10 + (buffer[11]-'0');
            sensordata.data4 = (buffer[12]-'0') * 1000 + (buffer[13]-'0') * 100 + (buffer[14]-'0') * 10 + (buffer[15]-'0');
            // strncpy(sensordata.data_,buffer,16); // 将读取的数据复制到sensordata.datastr中
            // sensordata.data = atoi(sensordata.datastr); // 将字符串转换为证书存入sensordata.data

            // force_1 = buffer[0] * 1000 + buffer[1] * 100 + buffer[2] * 10 + buffer[3];
            // force_1 = atoi(buffer);
            // printf("force_sensor_1: %dg\n",force_1); // 打印读取到的4个字节数据
            // printf("force_sensor_1: %dg\n",sensordata.data); // 打印读取到的4个字节数据
            orb_publish(ORB_ID(barometric_force_sensor), barometric_force_sensor_pub, &sensordata); // 用orb_publish函数发布新的传感器数据
            // if (ret < 0)
            // {
            //     PX4_ERR("Failed to publish sensor data: %d", ret);
            // }
            // else
            // {
            //     PX4_INFO("Successd to publish sensor data: %d", ret);
            // }
            // px4_sleep(1);
        }
    }

    warnx("exiting"); // 打印退出消息
    thread_running = false; // 停止线程
    close(uart_read); // 关闭串口设备文件描述符

    fflush(stdout); // 刷新标准输出流，确保所有输出被写入
    return 0;
}
