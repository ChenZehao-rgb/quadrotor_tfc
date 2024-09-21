set(MODULE_CFLAGS)
px4_add_module(
    MODULE drivers__barometric_force_sensor
    MAIN barometric_force_sensor
    COMPILE_FLAGS
        -Os
    SRCS
        barometric_force_sensor.c
    DEPENDS
       #platforms__common
    )
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
