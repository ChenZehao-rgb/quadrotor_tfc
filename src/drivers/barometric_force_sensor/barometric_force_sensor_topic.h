#ifndef __BAROMETRIC_FORCE_SENSOR_TOPIC_H_
#define __BAROMETRIC_FORCE_SENSOR_TOPIC_H_

#include <stdint.h>
#include <uORB/uORB.h>


/*声明主题，主题名自定义*/
ORB_DECLARE(barometric_force_sensor);

/* 定义要发布的数据结构体 */
struct barometric_force_sensor_data_s{
    char datastr[4];        //原始数据
    int data;               //解析数据，单位：g
};

#endif
