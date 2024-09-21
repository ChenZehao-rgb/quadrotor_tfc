#include "barometricforce.hpp"
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

BarometricForce::BarometricForce() :
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
    _force_pub(orb_advertise(ORB_ID(sensor_force), nullptr))
{
}

BarometricForce::~BarometricForce()
{
    if (_serial_fd >= 0) {
        ::close(_serial_fd);
    }
}

int BarometricForce::init()
{
    _serial_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (_serial_fd < 0) {
        PX4_ERR("Failed to open serial port: %s", strerror(errno));
        return -1;
    }

    if (configure_serial_port(B9600) != 0) {
        return -1;
    }

    ScheduleOnInterval(10000); // 10 ms interval

    return 0;
}

void BarometricForce::Run()
{
    char buffer[512]; // Adjust buffer size as needed
    int n = ::read(_serial_fd, buffer, sizeof(buffer));

    if (n > 0) {
        // Process MAVLink message
        for (int i = 0; i < n; ++i) {
            uint8_t byte = buffer[i];
            mavlink_message_t msg;
            mavlink_status_t status;

            if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
                // MAVLink message received
                if (msg.msgid == MAVLINK_MSG_ID_CUSTOM_FORCE) {
                    mavlink_custom_force_t custom_force;
                    mavlink_msg_custom_force_decode(&msg, &custom_force);

                    // Example: Extracting and publishing force sensor data
                    sensor_force_s force_data{};
                    force_data.timestamp = hrt_absolute_time();
                    force_data.force1 = custom_force.force1;
                    force_data.force2 = custom_force.force2;
                    force_data.force3 = custom_force.force3;
                    force_data.force4 = custom_force.force4;

                    orb_publish(ORB_ID(sensor_force), _force_pub, &force_data);
                }
                // Add handling for other message types as needed
            }
        }
    }
}

int BarometricForce::configure_serial_port(int baudrate)
{
    struct termios config;

    if (tcgetattr(_serial_fd, &config) < 0) {
        PX4_ERR("Failed to get serial port attributes: %s", strerror(errno));
        return -1;
    }

    config.c_cflag |= (CLOCAL | CREAD);
    config.c_cflag &= ~CSIZE;
    config.c_cflag |= CS8;
    config.c_cflag &= ~PARENB;
    config.c_cflag &= ~CSTOPB;
    config.c_cflag &= ~CRTSCTS;

    cfsetispeed(&config, baudrate);
    cfsetospeed(&config, baudrate);

    if (tcsetattr(_serial_fd, TCSANOW, &config) < 0) {
        PX4_ERR("Failed to set serial port attributes: %s", strerror(errno));
        return -1;
    }

    return 0;
}

extern "C" __EXPORT int barometric_force_main(int argc, char *argv[])
{
    BarometricForce driver;

    if (driver.init() != 0) {
        PX4_ERR("Failed to initialize the BarometricForce driver");
        return -1;
    }

    px4::ScheduleWorker(worker, 1);

    return 0;
}
