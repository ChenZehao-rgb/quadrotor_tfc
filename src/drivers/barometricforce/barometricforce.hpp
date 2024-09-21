
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_force.h>
#include <fcntl.h>
#include <termios.h>

class BarometricForce : public px4::ScheduledWorkItem
{
public:
    BarometricForce();
    virtual ~BarometricForce();

    int init();
    void Run() override;

private:
    int _serial_fd{-1};
    char _port[20]{"/dev/ttyS1"};
    orb_advert_t _force_pub{nullptr};
    int configure_serial_port(int baudrate);
};
